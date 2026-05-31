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
    PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,60,4,0.7f,0.00f,0.0f,0.3f,0.7f);
    PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,60,4,0.7f,0.00f,0.0f,0.3f,0.7f);
    PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,60,4,1.7f,0.00f,0.0f,0.3f,0.7f);

    PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,60,4,0.13f,0.0f,0.00f,0.3f,0.7f);
    PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,60,4,0.13f,0.0f,0.00f,0.3f,0.7f);
    PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,60,4,0.33f,0.0f,0.00f,0.3f,0.7f);

    surface_control_pid[Angle][PITCH].deadband  = 3.0f;
    surface_control_pid[Angle][ROLL].deadband   = 3.0f;
    surface_control_pid[Angle][YAW].deadband    = 3.0f;
    surface_control_pid[Gyro][PITCH].deadband   = 5.0f;
    surface_control_pid[Gyro][ROLL].deadband    = 5.0f;
    surface_control_pid[Gyro][YAW].deadband     = 5.0f;
    
    PID_struct_init(&mahony_pid[PITCH] ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[ROLL]  ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[YAW]   ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
}

void Euler_pid_Cale(float delta_time_z)
{
    for (int i = 0;i<3;i++)
    {
        pid_t *outer = &surface_control_pid[Angle][i];   /* 外环:角度环 */
        pid_t *inner = &surface_control_pid[Gyro][i];    /* 内环:角速度环 */

        /*num1，num2非零 时，前馈才有效果*/
        temp[i] = pid_calc( outer,Surface.current_angle_Euler[NOW][i],Surface.target_angle_Euler[NOW][i],delta_time_z);
        temp[i] += FeedForwardController(&outer->xFeedForward,Surface.target_angle_Euler[NOW][i],outer->xFeedForward.num1,outer->xFeedForward.num2);
        Surface.output_gyro_Euler[NOW][i] = pid_calc( inner,Surface.current_gyro_Euler[NOW][i],temp[i],delta_time_z);
        Surface.output_gyro_Euler[NOW][i] += FeedForwardController(&inner->xFeedForward,temp[i],inner->xFeedForward.num1,inner->xFeedForward.num2);
    }
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

float pid_calc(pid_t* pid, float get, float set , float delta_time)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
        return 0;

    /* 死区软化:在死区以内，是认为误差为零*/
    float e_now  = Deadband_Soften(pid->err[NOW],  pid->deadband);
    float e_last = Deadband_Soften(pid->err[LAST], pid->deadband);

    if(pid->pid_mode == POSITION_PID)
    {
        pid->pout  = pid->p * e_now;
        pid->iout += pid->i * e_now*delta_time;
        if (pid->d == 0.0f)
        {
            pid->dout = 0.0f;
        }
        else
        {
            pid->dout = pid->d * (e_now - e_last)/delta_time;
            if (delta_time < 1e-6f) pid->dout = 0.0f;
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

