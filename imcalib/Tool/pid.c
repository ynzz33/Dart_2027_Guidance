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
    PID_struct_init(&surface_control_pid[Angle][PITCH] ,POSITION_PID,10,4,0.1f,0.0f,0.0f,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Angle][ROLL]  ,POSITION_PID,10,4,0.1f,0.0f,0.0f,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Angle][YAW]   ,POSITION_PID,10,4,0.1f,0.0f,0.0f,0.0f,0.0f);

    // 角速度环PID参数 - 更注重响应速度
    PID_struct_init(&surface_control_pid[Gyro][PITCH]  ,POSITION_PID,10,4,0.1f  ,0.0f  ,0.0f,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Gyro][ROLL]   ,POSITION_PID,10,4,0.1f  ,0.0f  ,0.0f,0.0f,0.0f);
    PID_struct_init(&surface_control_pid[Gyro][YAW]    ,POSITION_PID,10,4,0.1f  ,0.0f  ,0.0f,0.0f,0.0f);

    // 设置控制死区，减少小误差时的舵机抖动
    surface_control_pid[Angle][PITCH].deadband = 0.5f;
    surface_control_pid[Angle][ROLL].deadband = 0.5f;
    surface_control_pid[Angle][YAW].deadband = 0.8f;
    surface_control_pid[Gyro][PITCH].deadband = 1.0f;
    surface_control_pid[Gyro][ROLL].deadband = 1.0f;
    surface_control_pid[Gyro][YAW].deadband = 1.5f;
    PID_struct_init(&mahony_pid[PITCH] ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[ROLL]  ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
    PID_struct_init(&mahony_pid[YAW]   ,POSITION_PID,mahony_MAXOUT,mahony_i_maxout,mahony_Kp,mahony_Ki,mahony_Kd,0.0f,0.0f);
}

void Euler_pid_Cale(float delta_time_z)
{
    for (int i = 0;i<3;i++)
    {
        temp[i] = pid_calc( &surface_control_pid[Angle][i],Surface.current_angle_Euler[NOW][i],Surface.target_angle_Euler[NOW][i],delta_time_z);
        Surface.output_gyro_Euler[NOW][i] = pid_calc( &surface_control_pid[Gyro][i],Surface.current_gyro_Euler[NOW][i],temp[i],delta_time_z);
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

float pid_calc(pid_t* pid, float get, float set , float delta_time)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if(pid->pid_mode == POSITION_PID)
    {
        pid->pout  = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW]*delta_time;
        if (pid->d == 0.0f)
        {
            pid->dout = 0.0f;
        }
        else
        {
            pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST])/delta_time;
            if (delta_time < 1e-6f) pid->dout = 0.0f;
        }

        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;

        abs_limit(&(pid->pos_out), pid->MaxOutput);

        pid->last_pos_out = pid->pos_out;
    }
    else if(pid->pid_mode == DELTA_PID)
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW]*delta_time;
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);

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

    FFC->rin=target;									//将目标值赋到rin上
    float result;
    result=num1*(FFC->rin-FFC->lastRin)+num2*(FFC->rin-2*FFC->lastRin+FFC->perRin);
    FFC->lastRin= FFC->rin;						//更新
    FFC->perRin= FFC->lastRin;
    FFC->FFC_pos_out=result;					//将输出值赋到结构体输出值中
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

