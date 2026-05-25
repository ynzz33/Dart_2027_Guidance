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

typedef struct	__FFC_t
{
	float FFCTar;
	float rin;
	float lastRin;
	float perRin;
	float FFC_pos_out;
	float set_point;

	double num1;
	double num2;

}FFC_t;

typedef struct __pid_t
{
	float p;
	float i;
	float d;

	uint8_t pid_mode;
	float MaxOutput;
	float IntegralLimit;

	FFC_t * xFeedForward;

	float set[3];
	float get[3];
	float err[3];

	float pout;
	float iout;
	float dout;

	float pos_out;
	float last_pos_out;
	float delta_u;
	float delta_out;
	float last_delta_out;

	float max_err;
	float deadband;


	void (*f_param_init)(struct __pid_t *pid,
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
void Euler_pid_Cale(float delta_time_z);
float FeedForwardController(FFC_t *FFC,float target,float num1,float num2);
extern pid_t surface_control_pid[2][3];
void abs_limit(float *a, float ABS_MAX);
float Near_By_Process(float set , float get , float Near_By_Value);

extern pid_t surface_control_pid[2][3], mahony_pid[3];
extern float temp[3];
#endif

