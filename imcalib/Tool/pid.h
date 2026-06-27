#ifndef __pid_H
#define __pid_H


#include "stm32G4xx_hal.h"

enum
{
    POSITION_PID,
    DELTA_PID,
};

enum
{
	PID,
	FFC
};

/* 前馈控制器(FeedForward):对目标做一/二阶差分前馈,与 pid_calc 解耦后叠加。
 * out = num1·(rin−lastRin) + num2·(rin−2·lastRin+perRin);num1/num2=0 时恒 0(当前未启用)。*/
typedef struct	__FFC_t
{
	float FFCTar;        /* 前馈目标(预留) */
	float rin;           /* 本拍输入(目标值) */
	float lastRin;       /* 上一拍输入 */
	float perRin;        /* 上上拍输入(二阶差分用) */
	float FFC_pos_out;   /* 前馈输出 */
	float set_point;     /* 设定点(预留) */

	float num1;          /* 一阶(速度)前馈增益 */
	float num2;          /* 二阶(加速度)前馈增益 */

}FFC_t;

typedef struct __pid_t
{
	float p;             /* 比例 / 积分 / 微分增益 */
	float i;
	float d;

	uint8_t pid_mode;    /* POSITION_PID(位置式) 或 DELTA_PID(增量式) */
	float MaxOutput;     /* 输出限幅 */
	float IntegralLimit; /* 积分限幅(抗 windup) */

	FFC_t xFeedForward;  /* 内嵌前馈器(当前未启用) */

	float set[3];        /* 目标 / 反馈 / 误差的 [NOW,LAST,LLAST] 历史 */
	float get[3];
	float err[3];

	float pout;          /* 本拍 P / I / D 分量 */
	float iout;
	float dout;

	float pos_out;       /* 位置式:本拍输出 / 上一拍输出 */
	float last_pos_out;
	float delta_u;       /* 增量式:本拍增量 / 累加输出 / 上一拍输出 */
	float delta_out;
	float last_delta_out;

	float max_err;       /* 误差>此值则输出 0(=0 关闭);本项目全 0 不触发 */
	float deadband;      /* 死区半宽,配 Deadband_Soften 连续软化 */
	uint8_t angle_wrap;   /* 1=误差按角度环绕到(-180,180];atan2 周期角的角度外环(roll/yaw)用 */


	void (*f_param_init)(struct __pid_t *pid,    /* 参数初始化函数指针 */
					uint8_t pid_mode,
					float maxOutput,
					float integralLimit,
					float p,
					float i,
					float d);

}pid_t;



/*����PID�ṹ���ʼ������*/
void PID_struct_init(pid_t* pid, uint8_t mode, float maxout, float intergral_limit, float kp, float ki, float kd, float  ff_param1, float  ff_param2);
void FeedForwardParamInit(FFC_t *FFC, float param1, float param2);
void pid_init(void);
static void pid_param_init(pid_t *pid,uint8_t mode,float maxout,float intergral_limit,float kp,float ki,float kd );
/*PID���㺯��*/
float pid_calc(pid_t* pid, float get, float set , float delta_time);
float Deadband_Soften(float err, float deadband);
float Angle_Wrap_180(float deg);
void Euler_pid_Cale(float delta_time_z);
float FeedForwardController(FFC_t *FFC,float target,float num1,float num2);
extern pid_t surface_control_pid[2][3];
void abs_limit(float *a, float ABS_MAX);
float Near_By_Process(float set , float get , float Near_By_Value);

extern pid_t surface_control_pid[2][3], mahony_pid[3];
extern pid_t vel_pursuit_pid[3];  /* 索引[PITCH,ROLL(未用),YAW] 速度方向外环 PID;必须[3]不能[2]——YAW=2,vel_pursuit_pid[YAW]即下标2,[2]会越界踩到 temp[] */
extern float temp[3];
#endif

