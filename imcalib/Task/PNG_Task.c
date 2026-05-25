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

PNG_Data_t PNG_Data;

void PNG_Init(PNG_Data_t* PNG_Data)
{
	/*Field of View*/
	PNG_Data->FOV = SENSOR_FOV * M_PI / 180 / SENSOR_TOTAL_PIXEL_WIDTH;
	PNG_Data->los_GYRO[NOW][X] = 0 ;
	PNG_Data->los_GYRO[NOW][Y] = 0 ;
	PNG_Data->N_R = N_rate;
}

void PNG_Guidance(Vision_Rx_Buf_t* Vision_Data , PNG_Data_t* PNG_Data , Surface_t* Surface , IMU_DATA_t* IMU_Data)
{

	PNG_Data->los_GYRO[LAST][X] = PNG_Data->los_GYRO[NOW][X];
	PNG_Data->los_GYRO[LAST][Y] = PNG_Data->los_GYRO[NOW][Y];
	PNG_Data->los_ANGLE[LAST][X] = PNG_Data->los_ANGLE[NOW][X];
	PNG_Data->los_ANGLE[LAST][Y] = PNG_Data->los_ANGLE[NOW][Y];
	if (Vision_Data == NULL || PNG_Data == NULL || Surface == NULL || IMU_Data == NULL)
	{
		return;
	}
	//视线角
	PNG_Data->los_ANGLE[NOW][X] = Vision_Data->x[NOW] * PNG_Data->FOV;
	PNG_Data->los_ANGLE[NOW][Y] = Vision_Data->y[NOW] * PNG_Data->FOV;

	PNG_Data->los_ANGLE[NOW][X] = KalmanFilter( &PNG_angle_Kalman_Filter[X],PNG_Data->los_ANGLE[NOW][X], 0.01,0.5);
	PNG_Data->los_ANGLE[NOW][Y] = KalmanFilter( &PNG_angle_Kalman_Filter[Y],PNG_Data->los_ANGLE[NOW][Y], 0.01,0.5);

	//通过帧之间解算视线角速度
	PNG_Data->los_GYRO[NOW][X] = (PNG_Data->los_ANGLE[NOW][X] - PNG_Data->los_ANGLE[LAST][X]) * SAMPLE_RATE;
	PNG_Data->los_GYRO[NOW][Y] = (PNG_Data->los_ANGLE[NOW][Y] - PNG_Data->los_ANGLE[LAST][Y]) * SAMPLE_RATE;

	PNG_Data->los_GYRO[NOW][X] = KalmanFilter( &PNG_gyro_Kalman_Filter[X],PNG_Data->los_GYRO[NOW][X], 0.01,0.5);
	PNG_Data->los_GYRO[NOW][Y] = KalmanFilter( &PNG_gyro_Kalman_Filter[Y],PNG_Data->los_GYRO[NOW][Y], 0.01,0.5);
	//减去自身得到相对角速度
	PNG_Data->los_GYRO[NOW][X] -= IMU_Data->G_Rad[NOW][X];
	PNG_Data->los_GYRO[NOW][Y] -= IMU_Data->G_Rad[NOW][Y];

	PNG_Data->los_vector[X] = cosf(PNG_Data->los_ANGLE[NOW][X]) * cosf(PNG_Data->los_ANGLE[NOW][Y]);
	PNG_Data->los_vector[Y] = sinf(PNG_Data->los_ANGLE[NOW][X]) * cosf(PNG_Data->los_ANGLE[NOW][Y]);
	PNG_Data->los_vector[Z] = sinf(PNG_Data->los_ANGLE[NOW][Y]);

	PNG_Data->V_c = -(IMU_Data->Velocity[Body][NOW][X] * PNG_Data->los_vector[X] +
					  IMU_Data->Velocity[Body][NOW][Y] * PNG_Data->los_vector[Y] +
					  IMU_Data->Velocity[Body][NOW][Z] * PNG_Data->los_vector[Z]);

	if (fabsf(PNG_Data->V_c) < Vc_min)
	{
		PNG_Data->V_c = (PNG_Data->V_c >= 0) ? Vc_min : -Vc_min;
	}

	Surface-> target_angle_Euler[NOW][PITCH] = PNG_Data->los_GYRO[NOW][Y] * PNG_Data->N_R * PNG_Data->V_c / K_Dyn * 180.f/M_PI;
	Surface-> target_angle_Euler[NOW][YAW]   = PNG_Data->los_GYRO[NOW][X] * PNG_Data->N_R * PNG_Data->V_c / K_Dyn * 180.f/M_PI;


	abs_limit( &Surface-> target_angle_Euler[NOW][PITCH],MAX_PNG_OUT );
	abs_limit( &Surface-> target_angle_Euler[NOW][YAW]  ,MAX_PNG_OUT );
}
