#include "pid.h"
#include <math.h>
#include "user_lib.h"
#include "surface_control_task.h"
#include "IMU.h"
#include "common_defs.h"
#include "CallBack_Task.h"

pid_t surface_control_pid[2][3], mahony_pid[3];
pid_t vel_pursuit_pid[3];  /* 索引[PITCH,ROLL(未用),YAW];必须[3]不能[2]——YAW=2,vel_pursuit_pid[YAW]即下标2,[2]会越界踩到 temp[] */

float temp[3];
/**********************************************************************************************************************
 * @brief   ABS
 * @param
 * @retval
**********************************************************************************************************************/
float ABS(float num)
{
	float value = (num<0) ? -num : num;

	return value;
}

void abs_limit(float *a, float ABS_MAX)
{
    /* NaN/Inf 拦截(2026-08-12):比较对 NaN 全 false,NaN 会原样穿透一路到 __HAL_TIM_SET_COMPARE。
     * 非有限输入 → 归 0(仅异常时改变行为,正常路径不变)。 */
    if (!(*a * 0.0f == 0.0f))
    {
        *a = 0.0f;
        return;
    }
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void pid_init(void)
{
// //镖体1
    PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,200,50,                        0.25f,   0.0f,  0.00f                    ,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,200,50,                        0.35f,   0.0f,  0.00000f                    ,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,200,20,                        0.5f,    0.0f,  0.00000f                    ,0.0f,0.0f);


    PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,AXIS_LIMIT_PITCH ,0,          0.80f,    0.0f,   0.00f                ,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,AXIS_LIMIT_ROLL  ,0,          1.0f,     0.0f,   0.00f                ,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,AXIS_LIMIT_YAW   ,0,          1.30f,    0.0f,   0.0f                 ,0.0f,0.0f);
//
// // //镖体1

//     PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,200,50,                        0.2f,   0.0f,  0.05f                    ,0.0f,0.0f);
//     PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,100,50,                        1.8f,   0.5f,  0.5f                    ,0.0f,0.0f);
//     PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,200,50,                        0.55,   0.5f,  0.2f                      ,0.0f,0.0f);


//     PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,AXIS_LIMIT_PITCH,10,           0.30f,  0.0f,   0.00f                ,0.0f,0.0f);
//     PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,AXIS_LIMIT_ROLL,10,            0.2f,   0.0f,   0.00f                 ,0.0f,0.0f);
//     PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,AXIS_LIMIT_YAW ,10,             0.5f,   0.0f,   0.0f                    ,0.0f,0.0f);
//镖体2
    // PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,200,50,8.0f,0.00f,0.005f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,1200,10,2.00f,0.00f,0.03f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,1200,25,7.0f,0.5f,0.02f,0.3f,0.3f);
                                                                                                                                                               
    // PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,AXIS_LIMIT_PITCH,4,0.800f,0.00f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,AXIS_LIMIT_ROLL,10,0.45f,0.0f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,AXIS_LIMIT_YAW,4,0.4f,0.00f,0.0f,0.3f,0.3f);

// // //镖体3
    // PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,200,50,5.0f,1.00f,0.50f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,100,10,0.7f,0.02f,0.008f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,500,20,6.0f,0.5f,0.02f,0.3f,0.3f);

    // PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,AXIS_LIMIT_PITCH,4,0.500f,0.00f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,AXIS_LIMIT_ROLL,5,0.17f,0.001f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,AXIS_LIMIT_YAW,10,0.40f,0.0f,0.0f,0.3f,0.3f);

    surface_control_pid[Angle][PITCH].deadband  = 1.0f;
    surface_control_pid[Angle][ROLL].deadband   = 2.0f;
    surface_control_pid[Angle][YAW].deadband    = 0.5f;
    surface_control_pid[Gyro][PITCH].deadband   = 0.0f;
    surface_control_pid[Gyro][ROLL].deadband    = 0.0f;//3.0                    
    surface_control_pid[Gyro][YAW].deadband     = 2.0f;

    /* 角度外环 roll/yaw 是 atan2 周期角,误差需环绕到(-180,180];pitch 是 asin∈[-90,90] 不需要。
     * 内环(Gyro)是角速度、不是周期角,保持 0。
     * 当前 angle_wrap=0(未启用环绕):跨 ±180° 边界时可能出现假误差,但实际飞镖不太会转到那里。
     * 需要时置 1 即开(函数 Angle_Wrap_180 已就绪)。*/
    surface_control_pid[Angle][ROLL].angle_wrap = 0;   /* 0=未启用,需要时改 1 */
    surface_control_pid[Angle][YAW].angle_wrap  = 0;   /* 0=未启用,需要时改 1 */

    PID_struct_init(&mahony_pid[X]  ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[Y]  ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[Z]  ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);

    /* 速度方向外环 PID: 速度方向追目标方向 → 输出 body 目标角
     * MaxOutput=45°(body 目标角限幅), 参数先保守起步,台架微调 */
    PID_struct_init(&vel_pursuit_pid[PITCH], POSITION_PID, 45, 10, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    PID_struct_init(&vel_pursuit_pid[YAW],   POSITION_PID, 45, 10, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/* ============================================================================
 * Euler_pid_Cale — 串级PID:角度外环 → 角速度内环 → 力矩输出
 *
 * 陀螺轴映射(IMU.c): gx→PITCH, gy→ROLL, -gz→YAW (机体系原始测量)
 * 欧拉轴映射:         [0]=PITCH, [1]=ROLL,  [2]=YAW  (世界系四元数导出)
 *
 * 当前做法:角度外环输出(世界系欧拉角速率)直接喂内环(机体系陀螺)。
 * 小角度/小滚转时耦合可忽略;大滚转时需加欧拉运动学变换(见下方注释)。
 *
 * ★ 坐标系变换(TODO):陀螺轴序非标准ZYX(gx=pitch,gy=roll,gz=yaw),
 *   需按实际轴序推导变换矩阵后再启用。下方保留框架代码。
 * ============================================================================ */
void Euler_pid_Cale(float delta_time_z)
{
    if(Guidance_State>Stable&&Vision_Rx_Data.Vision_recognize_flag==RECOGNIZE_SUCCESS)
    {
        
        surface_control_pid[Angle][YAW].deadband    = 0.0f;
        temp[PITCH] = pid_calc( &surface_control_pid[Angle][PITCH],Surface.current_angle_Euler[NOW][PITCH],Surface.target_angle_Euler[NOW][PITCH],delta_time_z);
        Surface.output_gyro_Euler[NOW][PITCH] = pid_calc(&surface_control_pid[Gyro][PITCH],Surface.current_gyro_Euler[NOW][PITCH],temp[PITCH],delta_time_z);

        
        temp[YAW] = pid_calc( &surface_control_pid[Angle][YAW],Surface.current_angle_Euler[NOW][YAW],Surface.target_angle_Euler[NOW][YAW],delta_time_z);
        Surface.output_gyro_Euler[NOW][YAW] = pid_calc(&surface_control_pid[Gyro][YAW],Surface.current_gyro_Euler[NOW][YAW],temp[YAW],delta_time_z);
    }
    else
    {
               
        temp[YAW] = pid_calc( &surface_control_pid[Angle][YAW],Surface.current_angle_Euler[NOW][YAW],Surface.target_angle_Euler[NOW][YAW],delta_time_z);
        Surface.output_gyro_Euler[NOW][YAW] = pid_calc(&surface_control_pid[Gyro][YAW],Surface.current_gyro_Euler[NOW][YAW],temp[YAW],delta_time_z);
 
    }
         temp[ROLL] = pid_calc( &surface_control_pid[Angle][ROLL],Surface.current_angle_Euler[NOW][ROLL],Surface.target_angle_Euler[NOW][ROLL],delta_time_z);
        Surface.output_gyro_Euler[NOW][ROLL] = pid_calc(&surface_control_pid[Gyro][ROLL],Surface.current_gyro_Euler[NOW][ROLL]/2.0f,temp[ROLL],delta_time_z);

    /* ---- 坐标系变换(当前关闭,待轴序标定后启用) ----
     * 陀螺轴: PITCH=gx, ROLL=gy, YAW=-gz (IMU.c)
     * 对应机体轴: gx=ω_pitch, gy=ω_roll, gz=-ω_yaw
     * 逆变换(世界系→机体系),设 φ=roll, θ=pitch:
     *   ω_pitch = φ̇ − sin(θ)·ψ̇
     *   ω_roll  = cos(φ)·θ̇ + sin(φ)·cos(θ)·ψ̇
     *   ω_yaw   = −(−sin(φ)·θ̇ + cos(φ)·cos(θ)·ψ̇)   ← 取负对齐 -gz 约定
     *
     * 台架验证:固定 pitch/yaw 目标,手动滚转 20°,观察是否出现意外 yaw 漂移。
     * 若有→启用变换;若无→当前直通已够用(小角度近似)。
     * ------------------------------------------------ */
}

float Near_By_Process(float set , float get , float Near_By_Value)
{
    if (set-get>Near_By_Value)
        set = 2*Near_By_Value-(set-get);
    else if (set-get<-Near_By_Value)
        set = 2*Near_By_Value+(set-get);
    return set;
}

/**********************************************************************************************************************
 * @brief   死区软化:把"误差落入死区即输出突跳为 0"的硬切断改为连续过渡
 * @param   err       原始误差
 * @param   deadband  死区半宽(<=0 表示不设死区)
 * @retval  软化后误差:|err|<=deadband 时返回 0;越过死区后从 0 连续增长(扣掉死区量)
 * @note    边界处输出连续(无阶跃),避免小误差时舵机在 0 与 P*deadband 间反复跳变引起的
 *          抖动/极限环。P/I/D 统一使用本函数输出。
**********************************************************************************************************************/
float Deadband_Soften(float err, float deadband)
{
    if (deadband <= 0.0f) return err;          /* 未设死区,原样返回 */
    if (err >  deadband)  return err - deadband;
    if (err < -deadband)  return err + deadband;
    return 0.0f;                               /* 死区内:连续归零 */
}

/**********************************************************************************************************************
 * @brief   角度误差环绕:周期角(atan2 ∈[-180,180])跨 ±180° 边界时 set-get 会出现 ~360° 假跳变,
 *          反向猛打致自旋;把误差归一到 (-180,180]。仅对角度外环 roll/yaw 经 angle_wrap 开启。
**********************************************************************************************************************/
float Angle_Wrap_180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

float pid_calc(pid_t* pid, float get, float set , float delta_time)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;
    /* 角度环绕:周期角(atan2 ∈[-180,180])跨 ±180° 会出现 ~360° 假跳变、反向猛打致自旋;
     * 按通道开关 angle_wrap 把误差归一到(-180,180]。放在 max_err 判断前,免得假跳触发 max_err。*/
    if (pid->angle_wrap) pid->err[NOW] = Angle_Wrap_180(pid->err[NOW]);
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
        return 0;

    /* 死区软化:在死区以内，是认为误差为零*/
    float e_now  = Deadband_Soften(pid->err[NOW],  pid->deadband);
    float e_last = Deadband_Soften(pid->err[LAST], pid->deadband);

    if(pid->pid_mode == POSITION_PID)
    {
        pid->pout  = pid->p * e_now;
        pid->iout += pid->i * e_now*delta_time;
        /* D 项:当前用误差微分 d*(e_now−e_last)/dt。
         * 注:原设计(2026-06-10)是对测量(反馈)微分 −d*(get[NOW]−get[LAST])/dt(消除目标阶跃的微分冲击 kick),
         * 但调试中切回了误差微分。当前视觉目标已用 LPF 平滑(非阶跃),微分冲击被源头消解,暂无抖动。
         * 测量微分代码保留注释,需要时可切回。*/
        if (pid->d == 0.0f || delta_time < 1e-6f)
        {
            pid->dout = 0.0f;
        }
        else
        {
            // 测量微分(对 feedback 求导,目标阶跃不进 D): pid->dout = -pid->d * (pid->get[NOW] - pid->get[LAST]) / delta_time;
            pid->dout = pid->d * (e_now - e_last)/delta_time;
        }

        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;

        abs_limit(&(pid->pos_out), pid->MaxOutput);

        pid->last_pos_out = pid->pos_out;
    }
    else if(pid->pid_mode == DELTA_PID)
    {
        float e_llast = Deadband_Soften(pid->err[LLAST], pid->deadband);
        pid->pout = pid->p * (e_now - e_last);
        pid->iout = pid->i * e_now*delta_time;
        pid->dout = pid->d * (e_now - 2*e_last + e_llast);

        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;

        abs_limit(&(pid->delta_out), pid->MaxOutput);

        pid->last_delta_out = pid->delta_out;
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;

}

float FeedForwardController(FFC_t *FFC,float target,float num1,float num2)
{

    FFC->rin=target;
    float result; float old_last = FFC->lastRin;
    result=num1*(FFC->rin-FFC->lastRin)+num2*(FFC->rin-2.0f*FFC->lastRin+FFC->perRin);
    FFC->lastRin= FFC->rin;
    FFC->perRin= old_last;
    FFC->FFC_pos_out=result;
    return result;
}

void PID_struct_init(
    pid_t* pid,uint8_t mode,
    float maxout,float intergral_limit,
    float kp,float ki,float kd,
    float  ff_param1,float  ff_param2)
{	/*init function pointer*/
    pid->f_param_init = pid_param_init;
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
    FeedForwardParamInit(&pid->xFeedForward, ff_param1, ff_param2);
}

void FeedForwardParamInit(FFC_t *FFC, float param1, float param2)
{
    FFC->num1 = param1;
    FFC->num2 = param2;
}

static void pid_param_init(
    pid_t *pid,uint8_t mode,
    float maxout,float intergral_limit,
    float kp,float ki,float kd )
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
