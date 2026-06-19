#include "pid.h"
#include <math.h>
#include "user_lib.h"
#include "surface_control_task.h"
#include "IMU.h"

pid_t surface_control_pid[2][3], mahony_pid[3];

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
    if(*a > ABS_MAX)  
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
 
void pid_init(void)
{
// // //镖体1
    // PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,200,50,8.0f,1.00f,0.50f,0.3f,0.3f);  
    // PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,1200,10,2.00f,0.00f,0.02f,0.3f,0.3f); 
    // PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,1200,20,7.0f,0.5f,0.02f,0.3f,0.3f);

    // PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,AXIS_LIMIT_PITCH,4,0.800f,0.00f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,AXIS_LIMIT_ROLL,10,0.2f,0.0f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,AXIS_LIMIT_YAW,4,0.35f,0.00f,0.0f,0.3f,0.3f);
//镖体2
    PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,200,50,8.0f,0.00f,0.005f,0.3f,0.3f);  
    PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,1200,10,2.00f,0.00f,0.03f,0.3f,0.3f); 
    PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,1200,25,7.0f,0.5f,0.02f,0.3f,0.3f);

    PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,AXIS_LIMIT_PITCH,4,0.800f,0.00f,0.00f,0.3f,0.3f);
    PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,AXIS_LIMIT_ROLL,10,0.45f,0.0f,0.00f,0.3f,0.3f);
    PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,AXIS_LIMIT_YAW,4,0.4f,0.00f,0.0f,0.3f,0.3f);
    
// // //镖体3
    // PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,200,50,5.0f,1.00f,0.50f,0.3f,0.3f);  
    // PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,100,10,0.7f,0.02f,0.008f,0.3f,0.3f); 
    // PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,500,20,6.0f,0.5f,0.02f,0.3f,0.3f);

    // PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,AXIS_LIMIT_PITCH,4,0.500f,0.00f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,AXIS_LIMIT_ROLL,5,0.17f,0.001f,0.00f,0.3f,0.3f);
    // PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,AXIS_LIMIT_YAW,10,0.40f,0.0f,0.0f,0.3f,0.3f);
     
    surface_control_pid[Angle][PITCH].deadband  = 1.0f ;
    surface_control_pid[Angle][ROLL].deadband   = 1.0f;                     
    surface_control_pid[Angle][YAW].deadband    = 0.2f;
    surface_control_pid[Gyro][PITCH].deadband   = 1.0f;
    surface_control_pid[Gyro][ROLL].deadband    = 1.0f;//3.0
    surface_control_pid[Gyro][YAW].deadband     = 0.0f;

    /* 角度外环 roll/yaw 是 atan2 周期角,误差需环绕到(-180,180];pitch 是 asin∈[-90,90] 不需要。
     * 内环(Gyro)是角速度、不是周期角,保持 0。*/
    surface_control_pid[Angle][ROLL].angle_wrap = 0;
    surface_control_pid[Angle][YAW].angle_wrap  = 0;
    
    PID_struct_init(&mahony_pid[PITCH] ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[ROLL]  ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[YAW]   ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
}

void Euler_pid_Cale(float delta_time_z)
{
    if(Guidance_State>Stable)
    {
        temp[PITCH] = pid_calc( &surface_control_pid[Angle][PITCH],Surface.current_angle_Euler[NOW][PITCH],Surface.target_angle_Euler[NOW][PITCH],delta_time_z);
        Surface.output_gyro_Euler[NOW][PITCH] = pid_calc(&surface_control_pid[Gyro][PITCH],Surface.current_gyro_Euler[NOW][PITCH],temp[PITCH],delta_time_z);
        // temp[PITCH] += FeedForwardController(&surface_control_pid[Angle][PITCH].xFeedForward,Surface.target_angle_Euler[NOW][PITCH],surface_control_pid[Angle][PITCH].xFeedForward.num1,surface_control_pid[Angle][PITCH].xFeedForward.num2);
        // Surface.output_gyro_Euler[NOW][PITCH] += FeedForwardController(&surface_control_pid[Gyro][PITCH].xFeedForward,temp[PITCH],surface_control_pid[Gyro][PITCH].xFeedForward.num1,surface_control_pid[Gyro][PITCH].xFeedForward.num2);

   }
        
        temp[YAW] = pid_calc( &surface_control_pid[Angle][YAW],Surface.current_angle_Euler[NOW][YAW],Surface.target_angle_Euler[NOW][YAW],delta_time_z);
        Surface.output_gyro_Euler[NOW][YAW] = pid_calc(&surface_control_pid[Gyro][YAW],Surface.current_gyro_Euler[NOW][YAW],temp[YAW],delta_time_z);
        // temp[YAW] += FeedForwardController(&surface_control_pid[Angle][YAW].xFeedForward,Surface.target_angle_Euler[NOW][YAW],surface_control_pid[Angle][YAW].xFeedForward.num1,surface_control_pid[Angle][YAW].xFeedForward.num2);
        // Surface.output_gyro_Euler[NOW][YAW] += FeedForwardController(&surface_control_pid[Gyro][YAW].xFeedForward,temp[YAW],surface_control_pid[Gyro][YAW].xFeedForward.num1,surface_control_pid[Gyro][YAW].xFeedForward.num2);
         temp[ROLL] = pid_calc( &surface_control_pid[Angle][ROLL],Surface.current_angle_Euler[NOW][ROLL],Surface.target_angle_Euler[NOW][ROLL],delta_time_z);
        Surface.output_gyro_Euler[NOW][ROLL] = pid_calc(&surface_control_pid[Gyro][ROLL],Surface.current_gyro_Euler[NOW][ROLL],temp[ROLL],delta_time_z);
        // temp[ROLL] += FeedForwardController(&surface_control_pid[Angle][ROLL].xFeedForward,Surface.target_angle_Euler[NOW][ROLL],surface_control_pid[Angle][ROLL].xFeedForward.num1,surface_control_pid[Angle][ROLL].xFeedForward.num2);
        // Surface.output_gyro_Euler[NOW][ROLL] += FeedForwardController(&surface_control_pid[Gyro][ROLL].xFeedForward,temp[ROLL],surface_control_pid[Gyro][ROLL].xFeedForward.num1,surface_control_pid[Gyro][ROLL].xFeedForward.num2);

   
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
        /* D 项对"测量(反馈)"微分,而非对"误差"微分:误差 e=set−get,对 e 微分时 set 的阶跃
         * (视觉~20Hz 锁存、目标每 50ms 跳一次)会被 ÷dt(=1e-3) 放大成巨大脉冲(微分冲击 derivative kick),
         * 正是"制导段一加 D 就抖、纯陀螺自稳段不抖"的根因。改为只对 get 微分(姿态/角速度是物理连续量、
         * 不会瞬跳),D 只对真实运动求导=纯阻尼;符号取负(de/dt 中的 −d(get)/dt 项)。
         * 不经死区软化:死区内 P 不拉、保留 D 阻尼压住残余运动(目标附近"只阻尼不硬推"更稳)。零额外延迟、非低通。*/
        if (pid->d == 0.0f || delta_time < 1e-6f)
        {
            pid->dout = 0.0f;
        }
        else
        {
            pid->dout = -pid->d * (pid->get[NOW] - pid->get[LAST]) / delta_time;
            // pid->dout = pid->d * (e_now - e_last)/delta_time;
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

