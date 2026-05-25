#include "filter.h"

#include "IMU.h"

float I_data[9] = { 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1};
mat I_mat;
float I_data_4[4] = { 1, 0,
					0, 1};
mat I_mat_4;
float matrix_value1;
float matrix_value2;
float matrix_value3;
one_kalman_filter_init_t ADC_Battery_Kalman_Filter,IMU_Kalman_Filter[2][3],PNG_gyro_Kalman_Filter[2],PNG_angle_Kalman_Filter[2],ACC_WORLD_Kalman_Filter[3];

#if 1/*初始化*/
kalman_filter_t V_KF_x,V_KF_y,V_KF_z;
kalman_filter_init_t V_KF_Init_x = {
	// 初始状态：发射前静止，速度v=0，加速度a=0
	.xhat_data =          {0.0f, 0.0f},
	.xhatminus_data =     {0.0f, 0.0f},
	.z_data =             {0.0f, 0.0f},

	// 初始协方差：设大一点加速收敛
	.P_data =               {10.0f, 0.0f,
								0.0f, 10.0f},
	.Pminus_data =          {10.0f, 0.0f,
								0.0f, 10.0f},
	.K_data =               {0.0f, 0.0f,
								0.0f, 0.0f},

	// 观测矩阵 H：只观测加速度（状态的第二个元素）
	.H_data =               {0.0f, 0.0f,
								0.0f, 1.0f},
	.HT_data =              {0.0f, 0.0f,
								0.0f, 1.0f},

	// 初始转移矩阵 A（会在运行时用dt更新）
	.A_data =               {1.0f, dT,
								0.0f, 1.0f},
	.AT_data =              {1.0f, 0.0f,
								0.0f, 1.0f},

	// 过程噪声 Q：越小越信任匀加速模型
	.Q_data =               {0.0001f, 0.0f,
								0.0f, 0.001f},

	// 测量噪声 R：越小越信任加速度计
	.R_data =               {100.0f, 0.0f,  // 速度无观测，设大
								0.0f, 1.0f}   // 加速度观测噪声
};kalman_filter_init_t V_KF_Init_y = {
	// 初始状态：发射前静止，速度v=0，加速度a=0
	.xhat_data =          {0.0f, 0.0f},
	.xhatminus_data =     {0.0f, 0.0f},
	.z_data =             {0.0f, 0.0f},

	// 初始协方差：设大一点加速收敛
	.P_data =               {10.0f, 0.0f,
								0.0f, 10.0f},
	.Pminus_data =          {10.0f, 0.0f,
								0.0f, 10.0f},
	.K_data =               {0.0f, 0.0f,
								0.0f, 0.0f},

	// 观测矩阵 H：只观测加速度（状态的第二个元素）
	.H_data =               {0.0f, 0.0f,
								0.0f, 1.0f},
	.HT_data =              {0.0f, 0.0f,
								0.0f, 1.0f},

	// 初始转移矩阵 A（会在运行时用dt更新）
	.A_data =               {1.0f, dT,
								0.0f, 1.0f},
	.AT_data =              {1.0f, 0.0f,
								0.0f, 1.0f},

	// 过程噪声 Q：越小越信任匀加速模型
	.Q_data =               {0.0001f, 0.0f,
								0.0f, 0.001f},

	// 测量噪声 R：越小越信任加速度计
	.R_data =               {100.0f, 0.0f,  // 速度无观测，设大
								0.0f, 1.0f}   // 加速度观测噪声
};
kalman_filter_init_t V_KF_Init_z = {
	// 初始状态：发射前静止，速度v=0，加速度a=0
	.xhat_data =          {0.0f, 0.0f},
	.xhatminus_data =     {0.0f, 0.0f},
	.z_data =             {0.0f, 0.0f},

	// 初始协方差：设大一点加速收敛
	.P_data =               {10.0f, 0.0f,
								0.0f, 10.0f},
	.Pminus_data =          {10.0f, 0.0f,
								0.0f, 10.0f},
	.K_data =               {0.0f, 0.0f,
								0.0f, 0.0f},

	// 观测矩阵 H：只观测加速度（状态的第二个元素）
	.H_data =               {0.0f, 0.0f,
								0.0f, 1.0f},
	.HT_data =              {0.0f, 0.0f,
								0.0f, 1.0f},

	// 初始转移矩阵 A（会在运行时用dt更新）
	.A_data =               {1.0f, dT,
								0.0f, 1.0f},
	.AT_data =              {1.0f, 0.0f,
								0.0f, 1.0f},

	// 过程噪声 Q：越小越信任匀加速模型
	.Q_data =               {0.0001f, 0.0f,
								0.0f, 0.001f},

	// 测量噪声 R：越小越信任加速度计
	.R_data =               {100.0f, 0.0f,  // 速度无观测，设大
								0.0f, 1.0f}   // 加速度观测噪声
};
#endif
#if 1/*对速度的二阶卡尔曼预测*/
void Kalman_Vel_Init(void)
{
	kalman_filter_init(&V_KF_x,&V_KF_Init_x);
	kalman_filter_init(&V_KF_y,&V_KF_Init_y);
	kalman_filter_init(&V_KF_z,&V_KF_Init_z);
}

void Kalman_Vel_Calc(float acc_x, float acc_y,float acc_z)
{
	IMU_Data.Velocity[World][LAST][X] = IMU_Data.Velocity[World][NOW][X];
	IMU_Data.Velocity[World][LAST][Y] = IMU_Data.Velocity[World][NOW][Y];
	IMU_Data.Velocity[World][LAST][Z] = IMU_Data.Velocity[World][NOW][Z];

	float *res_x = kalman_filter_calc(&V_KF_x, 0, acc_x);
	float *res_y = kalman_filter_calc(&V_KF_y, 0, acc_y);
	float *res_z = kalman_filter_calc(&V_KF_z, 0, acc_z);

	IMU_Data.Velocity[World][NOW][X] = res_x[1];
	IMU_Data.Velocity[World][NOW][Y] = res_y[1];
	IMU_Data.Velocity[World][NOW][Z] = res_z[1];

}
#endif

#if 1  //实验室的卡尔曼滤波例程
/*
 * @brief 卡尔曼结构体初始化
 */
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{

  mat_init(&I_mat_4,2,2,(float *)I_data_4);
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
  mat_init(&F->z,2,1,(float *)I->z_data);								//创建观测输出矩阵
  mat_init(&F->A,2,2,(float *)I->A_data);								//创建转移矩阵
  mat_init(&F->H,2,2,(float *)I->H_data);								//创建输出矩阵
  mat_init(&F->Q,2,2,(float *)I->Q_data);								//创建过程误差矩阵
  mat_init(&F->R,2,2,(float *)I->R_data);								//创建测量误差矩阵
  mat_init(&F->P,2,2,(float *)I->P_data);								//创建上一次时刻协方差预测矩阵
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);					//创建当前时刻的协防差预测矩阵
  mat_init(&F->K,2,2,(float *)I->K_data);								//创建滤波增益矩阵
  mat_init(&F->AT,2,2,(float *)I->AT_data);							//创建转移矩阵的转置矩阵
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);							//创建输出矩阵的转置矩阵
  mat_trans(&F->H, &F->HT);//  matrix_value2 = F->A.pData[1];
}
// xhatminus==x(k|k-1) 基于上一次时刻的本次时刻状态的预测值
// xhat==X(k-1|k-1)    上一时刻状态卡尔曼输出的最优结果
// Pminus==p(k|k-1)    基于上一次时刻的本次时刻协方差的预测值
// P==p(k-1|k-1)       上一次时刻的协方差的最优结果
// AT==A'
// HT==H'
// K==kg(k)    I=1
/**
  *@param 卡尔曼参数结构体
  *@param 角度
  *@param 速度
*/
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  static float TEMP_data[4] = {0, 0, 0, 0};
  static float TEMP_data21[2] = {0, 0};
  static mat TEMP,TEMP21;
	static uint8_t is_temp_init = 0;
	if (!is_temp_init) {
		mat_init(&TEMP,    2, 2, (float *)TEMP_data);
		mat_init(&TEMP21,  2, 1, (float *)TEMP_data21);
		is_temp_init = 1;
	}
  F->z.pData[0] = signal1;//z(k),
  F->z.pData[1] = signal2;//z(k)

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);//
  mat_mult(&F->Pminus, &F->HT, &TEMP);//
  mat_mult(&TEMP, &F->P, &F->K);//

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&I_mat_4, &F->P, &TEMP);//
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}
/**
  * @brief  一维卡尔曼滤波
  * @param  none
  * @retval none
  * @other 	Q：过程噪声，Q增大，动态响应变快，收敛稳定性变坏
			R：测量噪声，R增大，动态响应变慢，收敛稳定性变好
  * @note   2023.5.14
*/
float KalmanFilter(one_kalman_filter_init_t * data,const float ResrcData,float ProcessNoise_Q,float MeasureNoise_R)
{
   float R = MeasureNoise_R;
   float Q = ProcessNoise_Q;
   float x_mid;
   float x_now;
   float p_mid ;
   float p_now;
   float kg;
   x_mid=data->x_last;
   p_mid=data->p_last+Q;
   kg=p_mid/(p_mid+R);
   x_now=x_mid+kg*(ResrcData-x_mid);

   p_now=(1-kg)*p_mid;
   data->p_last = p_now;
   data->x_last = x_now;
   return x_now;
}
float Low_Pass_Filter(float now_data,float last_data,float k)
{
    now_data=k*now_data+(1-k)*last_data;
    return now_data;
}
#if 0 /*对加速度的一阶三维卡尔曼滤波*/
kalman_filter3_t IMU_Kalman_Filter_3;
kalman_filter3_init_t IMU_Kalman_Filter_3_Init = {
  // 初始状态 (静止时设为0)
  .xhat_data =          {0.0f, 0.0f, 0.0f},
  .xhatminus_data =     {0.0f, 0.0f, 0.0f},
  .z_data =             {0.0f, 0.0f, 0.0f},

  // 初始协方差
  .P_data =               {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .Pminus_data =          {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .K_data =               {0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f},

  // 观测矩阵 (单位矩阵)
  .H_data =               {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .HT_data =              {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},

  // 初始转移矩阵 (会在运行时动态更新)
  .A_data =               {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .AT_data =              {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},

  // 过程噪声
  .Q_data =               {0.01f, 0.0f, 0.0f,
                              0.0f, 0.01f, 0.0f,
                              0.0f, 0.0f, 0.01f},

  // 测量噪声
  .R_data =               {0.1f, 0.0f, 0.0f,
                              0.0f, 0.1f, 0.0f,
                              0.0f, 0.0f, 0.1f}
};
#endif
/*
 * @brief 卡尔曼结构体初始化
 */
void kalman_filter3_init(kalman_filter3_t *F, kalman_filter3_init_t *I)
{
  mat_init(&I_mat,3,3,(float *)I_data);
  mat_init(&F->xhat,3,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,3,1,(float *)I->xhatminus_data);
  mat_init(&F->z,3,1,(float *)I->z_data);								//创建观测输出矩阵
  mat_init(&F->A,3,3,(float *)I->A_data);								//创建转移矩阵
  mat_init(&F->H,3,3,(float *)I->H_data);								//创建输出矩阵
  mat_init(&F->Q,3,3,(float *)I->Q_data);								//创建过程误差矩阵
  mat_init(&F->R,3,3,(float *)I->R_data);								//创建测量误差矩阵
  mat_init(&F->P,3,3,(float *)I->P_data);								//创建上一次时刻协方差预测矩阵
  mat_init(&F->Pminus,3,3,(float *)I->Pminus_data);					//创建当前时刻的协防差预测矩阵
  mat_init(&F->K,3,3,(float *)I->K_data);								//创建滤波增益矩阵
  mat_init(&F->AT,3,3,(float *)I->AT_data);							//创建转移矩阵的转置矩阵
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,3,3,(float *)I->HT_data);							//创建输出矩阵的转置矩阵
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}
// xhatminus==x(k|k-1) 基于上一次时刻的本次时刻状态的预测值
// xhat==X(k-1|k-1)    上一时刻状态卡尔曼输出的最优结果
// Pminus==p(k|k-1)    基于上一次时刻的本次时刻协方差的预测值
// P==p(k-1|k-1)       上一次时刻的协方差的最优结果
// AT==A'
// HT==H'
// K==kg(k)    I=1
/**
  *@param 卡尔曼参数结构体(imu特化)
  *@param 角速度
  *@param 加速度
*/
float *kalman_filter3_imu_calc(kalman_filter3_t *F, float acc_x, float acc_y, float acc_z, float gyr_x, float gyr_y, float gyr_z)
{
  float TEMP_data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  float TEMP_data21[3] = {0, 0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,3,3,(float *)TEMP_data);//
  mat_init(&TEMP21,3,1,(float *)TEMP_data21);//

  F->z.pData[0] = acc_x;//z(k)
  F->z.pData[1] = acc_y;//z(k)
  F->z.pData[2] = acc_z;//z(k)

  float A_data[9] = {
    1.0f        , gyr_z * dT      ,-gyr_y * dT,
   -gyr_z * dT  , 1.0f            , gyr_x * dT,
    gyr_y * dT  ,-gyr_x * dT     , 1.0f
};
  // 更新转移矩阵 A
  mat_init(&F->A, 3, 3, (float *)A_data);
  // 更新转移矩阵的转置 AT
  mat_trans(&F->A, &F->AT);

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);//
  mat_mult(&F->Pminus, &F->HT, &TEMP);//
  mat_mult(&TEMP, &F->P, &F->K);//

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&I_mat, &F->P, &TEMP);//
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];
  matrix_value3 = F->xhat.pData[2];

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  F->filtered_value[2] = F->xhat.pData[2];
  return F->filtered_value;
}
/*@brief 一阶低通滤波*/

#endif
