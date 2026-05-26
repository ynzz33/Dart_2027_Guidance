#ifndef _FILTER_H
#define _FILTER_H


#include "arm_math.h"
#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"
#include "common_defs.h"

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//�������ת��
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

typedef struct
{
	float raw_value;
	float filtered_value[3];
	mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter3_t;

typedef struct
{
	float raw_value;
	float filtered_value[3];
	float xhat_data[3], xhatminus_data[3], z_data[3],Pminus_data[9], K_data[9];
	float P_data[9];
	float AT_data[9], HT_data[9];
	float A_data[9];
	float H_data[9];
	float Q_data[9];
	float R_data[9];
} kalman_filter3_init_t;


typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

typedef struct
{
	float x_last;
	float p_last;
	float out;
} one_kalman_filter_init_t;

extern one_kalman_filter_init_t ADC_Battery_Kalman_Filter,IMU_Kalman_Filter[2][3],PNG_gyro_Kalman_Filter[2],PNG_angle_Kalman_Filter[2],ACC_WORLD_Kalman_Filter[3];
extern kalman_filter3_t IMU_Kalman_Filter_3;
extern kalman_filter3_init_t IMU_Kalman_Filter_3_Init;
void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
float  KalmanFilter(one_kalman_filter_init_t * data,float ResrcData,float ProcessNoise_Q,float MeasureNoise_R);
float Low_Pass_Filter(float now_data,float last_data,float k);
void kalman_filter3_init(kalman_filter3_t *F, kalman_filter3_init_t *I);
void Kalman_Vel_Init(void);
void Kalman_Vel_Calc(float acc_x, float acc_y,float acc_z);
float *kalman_filter3_calc(kalman_filter3_t *F, float signal1, float signal2, float signal3);
float *kalman_filter3_imu_calc(kalman_filter3_t *F, float acc_x, float acc_y, float acc_z, float gyr_x, float gyr_y, float gyr_z);
#endif

