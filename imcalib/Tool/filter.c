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

#if 1/*��ʼ��*/
kalman_filter_t V_KF_x,V_KF_y,V_KF_z;
kalman_filter_init_t V_KF_Init_x = {
	// ��ʼ״̬������ǰ��ֹ���ٶ�v=0�����ٶ�a=0
	.xhat_data =          {0.0f, 0.0f},
	.xhatminus_data =     {0.0f, 0.0f},
	.z_data =             {0.0f, 0.0f},

	// ��ʼЭ������һ���������
	.P_data =               {10.0f, 0.0f,
								0.0f, 10.0f},
	.Pminus_data =          {10.0f, 0.0f,
								0.0f, 10.0f},
	.K_data =               {0.0f, 0.0f,
								0.0f, 0.0f},

	// �۲���� H��ֻ�۲���ٶȣ�״̬�ĵڶ���Ԫ�أ�
	.H_data =               {0.0f, 0.0f,
								0.0f, 1.0f},
	.HT_data =              {0.0f, 0.0f,
								0.0f, 1.0f},

	// ��ʼת�ƾ��� A����������ʱ��dt���£�
	.A_data =               {1.0f, dT,
								0.0f, 1.0f},
	.AT_data =              {1.0f, 0.0f,
								0.0f, 1.0f},

	// �������� Q��ԽСԽ�����ȼ���ģ��
	.Q_data =               {0.0001f, 0.0f,
								0.0f, 0.001f},

	// �������� R��ԽСԽ���μ��ٶȼ�
	.R_data =               {100.0f, 0.0f,  // �ٶ��޹۲⣬���
								0.0f, 1.0f}   // ���ٶȹ۲�����
};kalman_filter_init_t V_KF_Init_y = {
	// ��ʼ״̬������ǰ��ֹ���ٶ�v=0�����ٶ�a=0
	.xhat_data =          {0.0f, 0.0f},
	.xhatminus_data =     {0.0f, 0.0f},
	.z_data =             {0.0f, 0.0f},

	// ��ʼЭ������һ���������
	.P_data =               {10.0f, 0.0f,
								0.0f, 10.0f},
	.Pminus_data =          {10.0f, 0.0f,
								0.0f, 10.0f},
	.K_data =               {0.0f, 0.0f,
								0.0f, 0.0f},

	// �۲���� H��ֻ�۲���ٶȣ�״̬�ĵڶ���Ԫ�أ�
	.H_data =               {0.0f, 0.0f,
								0.0f, 1.0f},
	.HT_data =              {0.0f, 0.0f,
								0.0f, 1.0f},

	// ��ʼת�ƾ��� A����������ʱ��dt���£�
	.A_data =               {1.0f, dT,
								0.0f, 1.0f},
	.AT_data =              {1.0f, 0.0f,
								0.0f, 1.0f},

	// �������� Q��ԽСԽ�����ȼ���ģ��
	.Q_data =               {0.0001f, 0.0f,
								0.0f, 0.001f},

	// �������� R��ԽСԽ���μ��ٶȼ�
	.R_data =               {100.0f, 0.0f,  // �ٶ��޹۲⣬���
								0.0f, 1.0f}   // ���ٶȹ۲�����
};
kalman_filter_init_t V_KF_Init_z = {
	// ��ʼ״̬������ǰ��ֹ���ٶ�v=0�����ٶ�a=0
	.xhat_data =          {0.0f, 0.0f},
	.xhatminus_data =     {0.0f, 0.0f},
	.z_data =             {0.0f, 0.0f},

	// ��ʼЭ������һ���������
	.P_data =               {10.0f, 0.0f,
								0.0f, 10.0f},
	.Pminus_data =          {10.0f, 0.0f,
								0.0f, 10.0f},
	.K_data =               {0.0f, 0.0f,
								0.0f, 0.0f},

	// �۲���� H��ֻ�۲���ٶȣ�״̬�ĵڶ���Ԫ�أ�
	.H_data =               {0.0f, 0.0f,
								0.0f, 1.0f},
	.HT_data =              {0.0f, 0.0f,
								0.0f, 1.0f},

	// ��ʼת�ƾ��� A����������ʱ��dt���£�
	.A_data =               {1.0f, dT,
								0.0f, 1.0f},
	.AT_data =              {1.0f, 0.0f,
								0.0f, 1.0f},

	// �������� Q��ԽСԽ�����ȼ���ģ��
	.Q_data =               {0.0001f, 0.0f,
								0.0f, 0.001f},

	// �������� R��ԽСԽ���μ��ٶȼ�
	.R_data =               {100.0f, 0.0f,  // �ٶ��޹۲⣬���
								0.0f, 1.0f}   // ���ٶȹ۲�����
};
#endif
#if 1/*���ٶȵĶ��׿�����Ԥ��*/
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

	/* 状态 x=[v,a], A=[[1,dT],[0,1]] 使 v+=dT*a; H=[[0,0],[0,1]] 用测量 acc 校正 a。
	 * 故速度=状态 0=filtered_value[0](积分得来),非 [1](=加速度)。原取 res[1] 是 bug。*/
	IMU_Data.Velocity[World][NOW][X] = res_x[0];
	IMU_Data.Velocity[World][NOW][Y] = res_y[0];
	IMU_Data.Velocity[World][NOW][Z] = res_z[0];

}
/* 俯冲入段一次性锚定世界速度初值(否则纯加速度积分从零会缺发射前进速度,使弹道角γ无意义)。
 * 把速度态 xhat[0] 直接置为入参,加速度态 xhat[1] 清零;同时回写 Velocity[World][NOW] 使本拍即生效。
 * 入参由调用方按"姿态前向单位向量×标称速度 V_NOM"算出(见 IMU.c IMU_Attitude_Algorithm)。*/
void Kalman_Vel_Set(float vx, float vy, float vz)
{
	V_KF_x.xhat.pData[0] = vx;  V_KF_x.xhat.pData[1] = 0.0f;
	V_KF_y.xhat.pData[0] = vy;  V_KF_y.xhat.pData[1] = 0.0f;
	V_KF_z.xhat.pData[0] = vz;  V_KF_z.xhat.pData[1] = 0.0f;
	IMU_Data.Velocity[World][NOW][X] = vx;
	IMU_Data.Velocity[World][NOW][Y] = vy;
	IMU_Data.Velocity[World][NOW][Z] = vz;
}
#endif

#if 1  //ʵ���ҵĿ������˲�����
/*
 * @brief �������ṹ���ʼ��
 */
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{

  mat_init(&I_mat_4,2,2,(float *)I_data_4);
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
  mat_init(&F->z,2,1,(float *)I->z_data);								//�����۲��������
  mat_init(&F->A,2,2,(float *)I->A_data);								//����ת�ƾ���
  mat_init(&F->H,2,2,(float *)I->H_data);								//�����������
  mat_init(&F->Q,2,2,(float *)I->Q_data);								//��������������
  mat_init(&F->R,2,2,(float *)I->R_data);								//��������������
  mat_init(&F->P,2,2,(float *)I->P_data);								//������һ��ʱ��Э����Ԥ�����
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);					//������ǰʱ�̵�Э����Ԥ�����
  mat_init(&F->K,2,2,(float *)I->K_data);								//�����˲��������
  mat_init(&F->AT,2,2,(float *)I->AT_data);							//����ת�ƾ����ת�þ���
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);							//������������ת�þ���
  mat_trans(&F->H, &F->HT);//  matrix_value2 = F->A.pData[1];
}
// xhatminus==x(k|k-1) ������һ��ʱ�̵ı���ʱ��״̬��Ԥ��ֵ
// xhat==X(k-1|k-1)    ��һʱ��״̬��������������Ž��
// Pminus==p(k|k-1)    ������һ��ʱ�̵ı���ʱ��Э�����Ԥ��ֵ
// P==p(k-1|k-1)       ��һ��ʱ�̵�Э��������Ž��
// AT==A'
// HT==H'
// K==kg(k)    I=1
/**
  *@param �����������ṹ��
  *@param �Ƕ�
  *@param �ٶ�
*/
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  /* 每个滤波器实例独立的临时变量，避免三轴互相覆盖 */
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP, TEMP21;
  mat_init(&TEMP,    2, 2, (float *)TEMP_data);
  mat_init(&TEMP21,  2, 1, (float *)TEMP_data21);
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
  * @brief  һά�������˲�
  * @param  none
  * @retval none
  * @other 	Q������������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
			R������������R���󣬶�̬��Ӧ�����������ȶ��Ա��
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
#if 0 /*�Լ��ٶȵ�һ����ά�������˲�*/
kalman_filter3_t IMU_Kalman_Filter_3;
kalman_filter3_init_t IMU_Kalman_Filter_3_Init = {
  // ��ʼ״̬ (��ֹʱ��Ϊ0)
  .xhat_data =          {0.0f, 0.0f, 0.0f},
  .xhatminus_data =     {0.0f, 0.0f, 0.0f},
  .z_data =             {0.0f, 0.0f, 0.0f},

  // ��ʼЭ����
  .P_data =               {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .Pminus_data =          {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .K_data =               {0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f},

  // �۲���� (��λ����)
  .H_data =               {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .HT_data =              {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},

  // ��ʼת�ƾ��� (��������ʱ��̬����)
  .A_data =               {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},
  .AT_data =              {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f},

  // ��������
  .Q_data =               {0.01f, 0.0f, 0.0f,
                              0.0f, 0.01f, 0.0f,
                              0.0f, 0.0f, 0.01f},

  // ��������
  .R_data =               {0.1f, 0.0f, 0.0f,
                              0.0f, 0.1f, 0.0f,
                              0.0f, 0.0f, 0.1f}
};
#endif
/*
 * @brief �������ṹ���ʼ��
 */
void kalman_filter3_init(kalman_filter3_t *F, kalman_filter3_init_t *I)
{
  mat_init(&I_mat,3,3,(float *)I_data);
  mat_init(&F->xhat,3,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,3,1,(float *)I->xhatminus_data);
  mat_init(&F->z,3,1,(float *)I->z_data);								//�����۲��������
  mat_init(&F->A,3,3,(float *)I->A_data);								//����ת�ƾ���
  mat_init(&F->H,3,3,(float *)I->H_data);								//�����������
  mat_init(&F->Q,3,3,(float *)I->Q_data);								//��������������
  mat_init(&F->R,3,3,(float *)I->R_data);								//��������������
  mat_init(&F->P,3,3,(float *)I->P_data);								//������һ��ʱ��Э����Ԥ�����
  mat_init(&F->Pminus,3,3,(float *)I->Pminus_data);					//������ǰʱ�̵�Э����Ԥ�����
  mat_init(&F->K,3,3,(float *)I->K_data);								//�����˲��������
  mat_init(&F->AT,3,3,(float *)I->AT_data);							//����ת�ƾ����ת�þ���
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,3,3,(float *)I->HT_data);							//������������ת�þ���
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}
// xhatminus==x(k|k-1) ������һ��ʱ�̵ı���ʱ��״̬��Ԥ��ֵ
// xhat==X(k-1|k-1)    ��һʱ��״̬��������������Ž��
// Pminus==p(k|k-1)    ������һ��ʱ�̵ı���ʱ��Э�����Ԥ��ֵ
// P==p(k-1|k-1)       ��һ��ʱ�̵�Э��������Ž��
// AT==A'
// HT==H'
// K==kg(k)    I=1
/**
  *@param �����������ṹ��(imu�ػ�)
  *@param ���ٶ�
  *@param ���ٶ�
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
  // ����ת�ƾ��� A
  mat_init(&F->A, 3, 3, (float *)A_data);
  // ����ת�ƾ����ת�� AT
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
/*@brief һ�׵�ͨ�˲�*/

#endif
