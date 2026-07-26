//
// Created by ynz on 2026/1/10.
//

#include "PNG_Task.h"

#include "CallBack_Task.h"
#include "filter.h"
#include "IMU.h"
#include "pid.h"
#include "stm32g4xx_hal.h"
#include "surface_control_task.h"
#include "vision_ins.h"
#include <math.h>

PNG_Data_t PNG_Data;

/* 前向声明:公共 Vc/视线率计算(供 PNG_Apply_Lead_Yaw/Pitch 调用) */
static void png_common_calc(float *corr_yaw, float *corr_pitch);

/* 速度PN开关默认值:先上 yaw(=1)、pitch 后续(=0)、稳健 Vc 缩放档(Mode0);三者均可 Vofa/调试器在线改 */
uint8_t PNG_Yaw_Flag   = 1;
uint8_t PNG_Pitch_Flag = 0;  /* 第一阶段暂不启用 Pitch PNG，只上 Yaw */
uint8_t PNG_Mode       = 0;  /* 0=Mode0(Vc缩放/vision_los_rate,稳健) 1=Mode1(EKF p×v真PN) */

void PNG_Init(PNG_Data_t* PNG_Data)
{
	/*Field of View*/
	PNG_Data->FOV = SENSOR_FOV * M_PI / 180 / SENSOR_TOTAL_PIXEL_WIDTH;
	PNG_Data->los_GYRO[NOW][X] = 0 ;
	PNG_Data->los_GYRO[NOW][Y] = 0 ;
	PNG_Data->N_R = N_rate;
	/* Vofa 观测字段清零 */
	PNG_Data->vc_used = 0.0f;
	PNG_Data->los_rate_used[PITCH] = 0.0f;
	PNG_Data->los_rate_used[YAW]   = 0.0f;
	PNG_Data->lead_corr[PITCH] = 0.0f;
	PNG_Data->lead_corr[YAW]   = 0.0f;
}

/* === 速度比例导引超前 ===
 * 在 Guidance_Terminal 已用「视觉锁存 + 斜坡 Target_Slew」得到的视觉跟踪目标角之上,叠加一段 PN 超前量,
 * 把惯性视线率λ̇按接近速度 Vc 缩放后驱动目标提前(λ̇→0=碰撞航线)。只在识别成功段调用(调用点已在
 * RECOGNIZE_SUCCESS 分支内)。纯叠加、有界、可标志位关断 → 速度变差也不会失控,且 flag=0 时与原碎片代码等价。
 *
 * Mode0(默认,稳健):用现成世界系视线率 vision_los_rate[deg/s](已限幅),增益 = PNG_K_VC·Vc;
 *                   速度只通过 Vc 进来(EKF 最稳健的标量输出),λ̇ 仍用经过验证的视觉信号。
 * Mode1(EKF全量,验证可用后切):用 vins_out.p_world×vins_out.v_world 算 1kHz 世界系视线率,N·Vc·λ̇/K_Dyn → 超前角,
 *                   信号更平滑、更接近教科书真 PN,但更依赖速度质量。p×v 的符号/轴映射 ★台架待定★。*/
void PNG_Apply_Lead(Surface_t* Surface , IMU_DATA_t* IMU_Data)
{
	(void)IMU_Data;   /* 预留(Mode1 如需机体量再用);当前世界系量直接取 vins_*/

	/* 1) Mode0 第一阶段：固定标称速度，不使用 EKF Vc（EKF 速度可信度低） */
	float corr_yaw, corr_pitch;
	if (PNG_Mode == 0)
	{
		float vc = V_NOM_MS;
		if (vc < PNG_VC_MIN) vc = PNG_VC_MIN;
		if (vc > PNG_VC_MAX) vc = PNG_VC_MAX;
		PNG_Data.vc_used = vc;

		/* Mode0:世界系 Euler 视线率(deg/s,已被 LOS_RATE_LIMIT_DPS 限幅) */
		float rate_yaw   = vision_los_rate[YAW];
		float rate_pitch = vision_los_rate[PITCH];
		PNG_Data.los_rate_used[YAW]   = rate_yaw;
		PNG_Data.los_rate_used[PITCH] = rate_pitch;
		float k = PNG_K_VC * vc;                 /* Vc 缩放增益 */
		corr_yaw   = PNG_MODE0_YAW_SIGN * k * rate_yaw;
		corr_pitch = k * rate_pitch;
	}
	else
	{
		/* Mode1: Vc 从 EKF 取，未锁定退回 V_NOM_MS */
		float vc = vins_out.locked ? fabsf(vins_out.vc) : V_NOM_MS;
		if (vc < PNG_VC_MIN) vc = PNG_VC_MIN;
		if (vc > PNG_VC_MAX) vc = PNG_VC_MAX;
		PNG_Data.vc_used = vc;
		/* Mode1:EKF 世界系几何视线率 ω=(p×v)/|p|² (rad/s)。
		 * p = vins_out.p_world = 镖−靶(世界 ENU:X右/东,Y前/北,Z上);v = vins_out.v_world。
		 * yaw 面(绕世界 Z/上):ω_z = (px·vy − py·vx)/|p|²。
		 * pitch 面(俯仰角速率):ρ=√(px²+py²),  λ̇_el = (vz·ρ² − pz·(px·vx+py·vy)) / (|p|²·ρ)。
		 * ★符号/轴映射台架单轴验证★:转动视线看超前是否同向,反了在此翻 SIGN。*/
		float px = vins_out.p_world[X], py = vins_out.p_world[Y], pz = vins_out.p_world[Z];
		float vx = vins_out.v_world[X], vy = vins_out.v_world[Y], vz = vins_out.v_world[Z];
		float p2  = px*px + py*py + pz*pz;
		float rho = sqrtf(px*px + py*py);
		float w_yaw = 0.0f, w_pitch = 0.0f;
		if (p2 > 1e-4f && rho > 1e-4f)
		{
			w_yaw   = (px*vy - py*vx) / p2;                                  /* rad/s,绕世界 Z */
			w_pitch = (vz*rho*rho - pz*(px*vx + py*vy)) / (p2 * rho);        /* rad/s,俯仰角率 */
		}
		const float YAW_SIGN   = (+1.0f);   /* ★台架待定★ */
		const float PITCH_SIGN = (+1.0f);   /* ★台架待定★ */
		w_yaw   *= YAW_SIGN;
		w_pitch *= PITCH_SIGN;
		PNG_Data.los_rate_used[YAW]   = w_yaw   * (180.0f / (float)M_PI);
		PNG_Data.los_rate_used[PITCH] = w_pitch * (180.0f / (float)M_PI);
		/* 真 PN:corr_deg = N·Vc·λ̇(rad/s)·57.3/K_Dyn(K_Dyn 含加速度→角的气动折算) */
		float k = PNG_Data.N_R * vc * (180.0f / (float)M_PI) / (float)K_Dyn;
		corr_yaw   = k * w_yaw;
		corr_pitch = k * w_pitch;
	}

	/* 3) YAW:flag 开则叠加,限幅后 -= */
	if (PNG_Yaw_Flag)
	{
		abs_limit(&corr_yaw, PNG_LEAD_LIMIT_DEG);
		PNG_Data.lead_corr[YAW] = corr_yaw;
		Surface->target_angle_Euler[NOW][YAW] -= corr_yaw;
	}
	else
	{
		PNG_Data.lead_corr[YAW] = 0.0f;
	}

	/* 4) PITCH:仅俯冲到位(<pitch_control_limit_deg)才主动制导 pitch(与锁存条件一致)。
	 *    flag=0 时退回原固定增益超前(PN_LEAD_K·λ̇),叠加常值配平 AOA_TRIM → 与改前逐字等价。*/
	// if (Surface->current_angle_Euler[NOW][PITCH] < pitch_control_limit_deg)
	// {
		float corr_p = PNG_Pitch_Flag ? corr_pitch
		                              : (PN_LEAD_K * vision_los_rate[PITCH]);
		abs_limit(&corr_p, PNG_LEAD_LIMIT_DEG);
		PNG_Data.lead_corr[PITCH] = corr_p;
		Surface->target_angle_Euler[NOW][PITCH] -= corr_p + AOA_TRIM_DEG;
	// }
	// else
	// {
	// 	PNG_Data.lead_corr[PITCH] = 0.0f;
	// }
}

/* ============================================================================
 * PNG_Apply_Lead_Yaw / PNG_Apply_Lead_Pitch — 拆分接口
 *
 * 供 Guidance_Terminal 分别门控:俯冲未到位时只调 Yaw 版,Pitch 不喂。
 * 公共的 Vc/视线率计算抽取为 png_common_calc(),避免代码重复。
 * ============================================================================ */

/* 公共计算:Vc + 视线率 + 每轴 corr 值(写 PNG_Data,输出 corr_yaw/corr_pitch) */
static void png_common_calc(float *corr_yaw, float *corr_pitch)
{
	/* Mode0 第一阶段：固定标称速度，不使用 EKF Vc（EKF 速度可信度低） */
	if (PNG_Mode == 0)
	{
		float vc = V_NOM_MS;
		if (vc < PNG_VC_MIN) vc = PNG_VC_MIN;
		if (vc > PNG_VC_MAX) vc = PNG_VC_MAX;
		PNG_Data.vc_used = vc;

		float rate_yaw   = vision_los_rate[YAW];
		float rate_pitch = vision_los_rate[PITCH];
		PNG_Data.los_rate_used[YAW]   = rate_yaw;
		PNG_Data.los_rate_used[PITCH] = rate_pitch;
		float k = PNG_K_VC * vc;
		*corr_yaw   = PNG_MODE0_YAW_SIGN * k * rate_yaw;
		*corr_pitch = k * rate_pitch;
	}
	else
	{
		float vc = vins_out.locked ? fabsf(vins_out.vc) : V_NOM_MS;
		if (vc < PNG_VC_MIN) vc = PNG_VC_MIN;
		if (vc > PNG_VC_MAX) vc = PNG_VC_MAX;
		PNG_Data.vc_used = vc;
		float px = vins_out.p_world[X], py = vins_out.p_world[Y], pz = vins_out.p_world[Z];
		float vx = vins_out.v_world[X], vy = vins_out.v_world[Y], vz = vins_out.v_world[Z];
		float p2  = px*px + py*py + pz*pz;
		float rho = sqrtf(px*px + py*py);
		float w_yaw = 0.0f, w_pitch = 0.0f;
		if (p2 > 1e-4f && rho > 1e-4f)
		{
			w_yaw   = (px*vy - py*vx) / p2;
			w_pitch = (vz*rho*rho - pz*(px*vx + py*vy)) / (p2 * rho);
		}
		const float YAW_SIGN   = (+1.0f);
		const float PITCH_SIGN = (+1.0f);
		w_yaw   *= YAW_SIGN;
		w_pitch *= PITCH_SIGN;
		PNG_Data.los_rate_used[YAW]   = w_yaw   * (180.0f / (float)M_PI);
		PNG_Data.los_rate_used[PITCH] = w_pitch * (180.0f / (float)M_PI);
		float k = PNG_Data.N_R * vc * (180.0f / (float)M_PI) / (float)K_Dyn;
		*corr_yaw   = k * w_yaw;
		*corr_pitch = k * w_pitch;
	}
}

void PNG_Apply_Lead_Yaw(Surface_t* Surface, IMU_DATA_t* IMU_Data)
{
	(void)IMU_Data;
	float corr_yaw, corr_pitch;
	png_common_calc(&corr_yaw, &corr_pitch);

	if (PNG_Yaw_Flag)
	{
		abs_limit(&corr_yaw, PNG_LEAD_LIMIT_DEG);
		PNG_Data.lead_corr[YAW] = corr_yaw;
		Surface->target_angle_Euler[NOW][YAW] -= corr_yaw;
	}
	else
	{
		PNG_Data.lead_corr[YAW] = 0.0f;
	}
}

void PNG_Apply_Lead_Pitch(Surface_t* Surface, IMU_DATA_t* IMU_Data)
{
	(void)IMU_Data;
	float corr_yaw, corr_pitch;
	png_common_calc(&corr_yaw, &corr_pitch);

	/* 仅俯冲到位时才叠加 pitch 超前(调用方已做门控,此处再守一道) */
	// if (Surface->current_angle_Euler[NOW][PITCH] < pitch_control_limit_deg)
	// {
		float corr_p = PNG_Pitch_Flag ? corr_pitch
		                              : (PN_LEAD_K * vision_los_rate[PITCH]);
		abs_limit(&corr_p, PNG_LEAD_LIMIT_DEG);
		PNG_Data.lead_corr[PITCH] = corr_p;
		Surface->target_angle_Euler[NOW][PITCH] -= corr_p + AOA_TRIM_DEG;
	// }
	// else
	// {
	// 	PNG_Data.lead_corr[PITCH] = 0.0f;
	// }
}
