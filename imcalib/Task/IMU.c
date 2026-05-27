//
// Created by ynz on 2025/11/17.
//

#include "IMU.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "mytype.h"
#include "spi.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "filter.h"
#include "math.h"
#include "pid.h"
#include "surface_control_task.h"
u8 Current_Sensor,Current_Use_Flag,receiveflag = 0;
IMU_DATA_t IMU_Data = {0};
uint32_t IMU_Cnt = 0,control_cnt = 0;
void IMU_Attitude_Algorithm(void)
{
#if 0 /*对加速度的一阶三维卡尔曼滤波*/
    float *filtered_acc = kalman_filter3_imu_calc(&IMU_Kalman_Filter_3, IMU_Data.A[NOW][X], IMU_Data.A[NOW][Y], IMU_Data.A[NOW][Z], IMU_Data.G_Rad[NOW][PITCH], IMU_Data.G_Rad[NOW][ROLL], IMU_Data.G_Rad[NOW][YAW]);

    // 更新滤波后的加速度值
    IMU_Data.temp[NOW][X] = filtered_acc[0];
    IMU_Data.temp[NOW][Y] = filtered_acc[1];
    IMU_Data.temp[NOW][Z] = filtered_acc[2];
    float acc_norm_t = sqrt(IMU_Data.temp[NOW][X]*IMU_Data.temp[NOW][X]+
                          IMU_Data.temp[NOW][Y]*IMU_Data.temp[NOW][Y]+
                          IMU_Data.temp[NOW][Z]*IMU_Data.temp[NOW][Z]);
    if (acc_norm_t<0.00001)
    {
        IMU_Data.temp[NOW][X] = 0;
        IMU_Data.temp[NOW][Y] = 0;
        IMU_Data.temp[NOW][Z] = 0;
    }
    else
    {
        IMU_Data.temp[NOW][X]/=acc_norm_t;
        IMU_Data.temp[NOW][Y]/=acc_norm_t;
        IMU_Data.temp[NOW][Z]/=acc_norm_t;
    }
#endif
#if 1 /*对速度的二阶卡尔曼*/
    // Kalman_Vel_Calc();
#endif
#if 1   /*局部变量定义*/
    float
    /* Q_theory_W       */  q0 = IMU_Data.Q[NOW][0],
    /* Q_theory_X       */  q1 = IMU_Data.Q[NOW][1],
    /* Q_theory_Y       */  q2 = IMU_Data.Q[NOW][2],
    /* Q_theory_Z       */  q3 = IMU_Data.Q[NOW][3],
    /* A_Real_X         */  a_raw_x = IMU_Data.A[NOW][X],
    /* A_Real_Y         */  a_raw_y = IMU_Data.A[NOW][Y],
    /* A_Real_Z         */  a_raw_z = IMU_Data.A[NOW][Z],
    /*A_NORMED          */  ax_normed = IMU_Data.A_Normed[NOW][X],
    /*A_NORMED          */  ay_normed = IMU_Data.A_Normed[NOW][Y],
    /*A_NORMED          */  az_normed = IMU_Data.A_Normed[NOW][Z],
    /* G_theory_X       */  tx,
    /* G_theory_Y       */  ty,
    /* G_theory_Z       */  tz,
    /* G_Real_X         */  gx = IMU_Data.G_Rad[NOW][X],
    /* G_Real_Y         */  gy = IMU_Data.G_Rad[NOW][Y],
    /* G_Real_Z         */  gz = IMU_Data.G_Rad[NOW][Z],
    /* 重力加速度,方便归一*/  gravity = GRAVITY_MS2;
#endif
#if 1   /*mahony补偿*/
    /*逆旋转矩阵的转化*/
    IMU_Data.R_matrix_T[0][0] = 1-2*(q2*q2+q3*q3);      IMU_Data.R_matrix_T[0][1] = 2*(q1*q2+q0*q3);        IMU_Data.R_matrix_T[0][2] = 2*(q1*q3-q0*q2);
    IMU_Data.R_matrix_T[1][0] = 2*(q1*q2-q0*q3);        IMU_Data.R_matrix_T[1][1] = 1-2*(q1*q1+q3*q3);      IMU_Data.R_matrix_T[1][2] = 2*(q2*q3+q0*q1);
    IMU_Data.R_matrix_T[2][0] = 2*(q1*q3+q0*q2);        IMU_Data.R_matrix_T[2][1] = 2*(q2*q3-q0*q1);        IMU_Data.R_matrix_T[2][2] = 1-2*(q1*q1+q2*q2);
    tx = IMU_Data.R_matrix_T[0][2] ;
    ty = IMU_Data.R_matrix_T[1][2] ;
    tz = IMU_Data.R_matrix_T[2][2] ;
    /*实际加速度归一化*/
    float mahony_temp[3] = {0};
    float acc_norm = sqrtf(a_raw_x*a_raw_x+a_raw_y*a_raw_y+a_raw_z*a_raw_z);
    if (acc_norm<0.001f)
    {
        ax_normed = 0;
        ay_normed = 0;
        az_normed = 0;
    }
    else
    {
        ax_normed=a_raw_x/acc_norm;
        ay_normed=a_raw_y/acc_norm;
        az_normed=a_raw_z/acc_norm;
    }
    /*计算误差*/
        mahony_temp[X] = pid_calc(&mahony_pid[X],az_normed*ty-ay_normed*tz,0,dT);
        mahony_temp[Y] = pid_calc(&mahony_pid[Y],ax_normed*tz-az_normed*tx,0,dT);
        mahony_temp[Z] = pid_calc(&mahony_pid[Z],ay_normed*tx-ax_normed*ty,0,dT);
    gx += mahony_temp[X];
    gy += mahony_temp[Y];
    gz += mahony_temp[Z];

    /*计算修正后的陀螺仪数据（比例+积分补偿）*/
#endif
#if 0   /*ekf补偿,还没做*/



#endif
#if 1
    /*世界加速度与
      世界速度与
      机体速度*/
    float ax_no_gravity = a_raw_x - gravity * IMU_Data.R_matrix_T[0][2];
    float ay_no_gravity = a_raw_y - gravity * IMU_Data.R_matrix_T[1][2];
    float az_no_gravity = a_raw_z - gravity * IMU_Data.R_matrix_T[2][2];
    for (int i = 0; i<3 ;i++)
    {
        IMU_Data.A_World[NOW] [i]  =  ( IMU_Data.R_matrix_T[0][i]*ax_no_gravity +
                                        IMU_Data.R_matrix_T[1][i]*ay_no_gravity +
                                        IMU_Data.R_matrix_T[2][i]*az_no_gravity );
        // IMU_Data.A_World[NOW] [i] = KalmanFilter( &ACC_WORLD_Kalman_Filter[i],IMU_Data.A_World[NOW][i],0.1f,5.0f );
    }
    /*世界速度*/
    // Kalman_Vel_Calc(IMU_Data.A_World[NOW][X],IMU_Data.A_World[NOW][Y],IMU_Data.A_World[NOW][Z]);
    // /*机体速度*/
    // IMU_Data.Velocity[Body][NOW][X] = IMU_Data.R_matrix_T[0][0] * IMU_Data.Velocity[World][NOW][X] +
    //                                   IMU_Data.R_matrix_T[0][1] * IMU_Data.Velocity[World][NOW][Y] +
    //                                   IMU_Data.R_matrix_T[0][2] * IMU_Data.Velocity[World][NOW][Z];
    // IMU_Data.Velocity[Body][NOW][Y] = IMU_Data.R_matrix_T[1][0] * IMU_Data.Velocity[World][NOW][X] +
    //                                   IMU_Data.R_matrix_T[1][1] * IMU_Data.Velocity[World][NOW][Y] +
    //                                   IMU_Data.R_matrix_T[1][2] * IMU_Data.Velocity[World][NOW][Z];
    // IMU_Data.Velocity[Body][NOW][Z] = IMU_Data.R_matrix_T[2][0] * IMU_Data.Velocity[World][NOW][X] +
    //                                   IMU_Data.R_matrix_T[2][1] * IMU_Data.Velocity[World][NOW][Y] +
    //                                   IMU_Data.R_matrix_T[2][2] * IMU_Data.Velocity[World][NOW][Z];

#endif


#if 1   /*四元数解算及欧拉角转换*/

    /*四元数积分*/
    q0 -= (0.5f*dT*(q1*gx + q2*gy + q3*gz));
    q1 += (0.5f*dT*(q0*gx + q2*gz - q3*gy));
    q2 += (0.5f*dT*(q0*gy - q1*gz + q3*gx));
    q3 += (0.5f*dT*(q0*gz + q1*gy - q2*gx));
    /*四元数更归一化*/
    float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_norm<0.001f)
    {
        q0 = 1;
        q1 = 0;
        q2 = 0;
        q3 = 0;
    }
    else
    {
        q0 /=q_norm;
        q1 /=q_norm;
        q2 /=q_norm;
        q3 /=q_norm;
    }
    /*欧拉角转换*/
    IMU_Data.Euler[NOW][PITCH]  = asinf (2.0f*(q0*q2-q3*q1));
    IMU_Data.Euler[NOW][YAW]    = atan2f(2.0f*(q0*q3+q1*q2),1.0f-2.0f*(q3*q3+q2*q2));
    IMU_Data.Euler[NOW][ROLL]   = atan2f(2.0f*(q0*q1+q3*q2),1.0f-2.0f*(q2*q2+q1*q1));

    IMU_Data.Euler[NOW][PITCH]  =   RAD2DEG(IMU_Data.Euler[NOW][PITCH]);
    IMU_Data.Euler[NOW][ROLL]   =   RAD2DEG(IMU_Data.Euler[NOW][ROLL]);
    IMU_Data.Euler[NOW][YAW]    =   RAD2DEG(IMU_Data.Euler[NOW][YAW]);
#endif
#if 1   /*历史值记录，数据更新*/
    // if (IMU_Data.Euler[NOW][PITCH] < 0)
    // {
    //     IMU_Data.Euler[NOW][PITCH] += 180;
    // }
    // if (IMU_Data.Euler[NOW][ROLL] < 0)
    // {
    //     IMU_Data.Euler[NOW][ROLL]  += 360;
    // }
    // if (IMU_Data.Euler[NOW][YAW] < 0 )
    // {
    //     IMU_Data.Euler[NOW][YAW]   += 360;
    // }
    IMU_Data.Q[NOW][0]  = q0 ;
    IMU_Data.Q[NOW][1]  = q1 ;
    IMU_Data.Q[NOW][2]  = q2 ;
    IMU_Data.Q[NOW][3]  = q3 ;
    IMU_Data.A_Normed[NOW][X]  = ax_normed  ;
    IMU_Data.A_Normed[NOW][Y]  = ay_normed  ;
    IMU_Data.A_Normed[NOW][Z]  = az_normed  ;
    /*更新*/
    IMU_Data.A_theory[NOW][X] = tx;
    IMU_Data.A_theory[NOW][Y] = ty;
    IMU_Data.A_theory[NOW][Z] = tz;
    for (int i = 0;i<4;i++)
    {
        IMU_Data.Q[LAST][i]=IMU_Data.Q[NOW][i];
    }
    for (int i = 0;i<3;i++)
    {
        Surface.current_angle_Euler[NOW][i]=IMU_Data.Euler[NOW][i];
        IMU_Data.Euler   [LAST][i]=IMU_Data.Euler[NOW][i];
        IMU_Data.A_theory[LAST][i] = IMU_Data.A_theory[NOW][i];
        IMU_Data.Velocity[World][LAST][i] = IMU_Data.Velocity[World][NOW][i];
        IMU_Data.Velocity[Body][LAST][i] = IMU_Data.Velocity[Body][NOW][i];
        IMU_Data.A_World [LAST][i] = IMU_Data.A_World[NOW][i];
    }
#endif
}
#if 1
/*imu底层通信*/
void BMX055_CS_ACC_Select(void)
{
    HAL_GPIO_WritePin( ACC_CS_PORT ,ACC_CS_PIN, GPIO_PIN_RESET);
}

void BMX055_CS_GYR_Select(void)
{
    HAL_GPIO_WritePin( GYR_CS_PORT ,GYR_CS_PIN, GPIO_PIN_RESET);
}

void BMX055_CS_MAG_Select(void)
{
    HAL_GPIO_WritePin( MAG_CS_PORT ,MAG_CS_PIN, GPIO_PIN_RESET);
}

void BMX055_CS_Select(uint8_t sensor)
{
    switch(sensor)
    {
        case ACC:
        {
            BMX055_CS_ACC_Select();
        }break;

        case GYR:
        {
            BMX055_CS_GYR_Select();
        }break;

        case MAG:
        {
            BMX055_CS_MAG_Select();
        }break;
    }
}

void BMX055_CS_ACC_Free(void)
{
    HAL_GPIO_WritePin( ACC_CS_PORT ,ACC_CS_PIN, GPIO_PIN_SET);
}

void BMX055_CS_GYR_Free(void)
{
    HAL_GPIO_WritePin( GYR_CS_PORT ,GYR_CS_PIN, GPIO_PIN_SET);
}

void BMX055_CS_MAG_Free(void)
{
    HAL_GPIO_WritePin( MAG_CS_PORT ,MAG_CS_PIN, GPIO_PIN_SET);
}
void BMX055_CS_HIGH_ACC_Free(void)
{
    HAL_GPIO_WritePin( HIGH_ACC_CS_PORT ,HIGH_ACC_CS_PIN, GPIO_PIN_SET);
}
void ALL_CS_Free(void)
{
    BMX055_CS_ACC_Free();
    BMX055_CS_GYR_Free();
    BMX055_CS_MAG_Free();
	  BMX055_CS_HIGH_ACC_Free();
}
void BMX055_CS_Free(uint8_t sensor)
{
    switch(sensor)
    {
        case ACC:
        {
            BMX055_CS_ACC_Free();
        }break;

        case GYR:
        {
            BMX055_CS_GYR_Free();
        }break;

        case MAG:
        {
            BMX055_CS_MAG_Free();
        }break;
    }
}
void BMX055_Read(uint8_t Sensor,uint8_t Reg_Addr)
{

        BMX055_CS_Select(Sensor);

        uint8_t tx_buf[7] = {0};
        uint8_t rx_buf[7] = {0};
        tx_buf[0] = Reg_Addr|0x80;


        HAL_SPI_TransmitReceive( &hspi2,tx_buf,rx_buf,7 ,5000);


        BMX055_CS_Free(Sensor);

          switch(Sensor)
            {
                case ACC:
                {
                    IMU_Data.A[NOW][X] = -(int16_t)(rx_buf[2]<<8|rx_buf[1]) * ACC_LSB_16G;
                    IMU_Data.A[NOW][Y] = -(int16_t)(rx_buf[4]<<8|rx_buf[3]) * ACC_LSB_16G;
                    IMU_Data.A[NOW][Z] = -(int16_t)(rx_buf[6]<<8|rx_buf[5]) * ACC_LSB_16G;
                    for (int k = 0; k < 3; k++)
                    {
                        if (isnan(IMU_Data.A[NOW][k]) || fabsf(IMU_Data.A[NOW][k]) > ACC_SAT_G)
                            IMU_Data.A[NOW][k] = IMU_Data.A[LAST][k];
                    }
                    IMU_Data.A[NOW][X] = KalmanFilter(&IMU_Kalman_Filter[ACC][X],IMU_Data.A[NOW][X],IMU_KF_Q,IMU_KF_R);
                    IMU_Data.A[NOW][Y] = KalmanFilter(&IMU_Kalman_Filter[ACC][Y],IMU_Data.A[NOW][Y],IMU_KF_Q,IMU_KF_R);
                    IMU_Data.A[NOW][Z] = KalmanFilter(&IMU_Kalman_Filter[ACC][Z],IMU_Data.A[NOW][Z],IMU_KF_Q,IMU_KF_R);
                    for (int k = 0; k < 3; k++) IMU_Data.A[LAST][k] = IMU_Data.A[NOW][k];

                }break;
                case GYR:
                {
                    IMU_Data.G[NOW][PITCH] = ((int16_t)(rx_buf[2]<<8|rx_buf[1])) / GYRO_LSB_2000DPS;
                    IMU_Data.G[NOW][ROLL ] = ((int16_t)(rx_buf[4]<<8|rx_buf[3])) / GYRO_LSB_2000DPS;
                    IMU_Data.G[NOW][YAW  ] = ((int16_t)(rx_buf[6]<<8|rx_buf[5])) / GYRO_LSB_2000DPS;
                    for (int k = 0; k < 3; k++)
                    {
                        if (isnan(IMU_Data.G[NOW][k]) || fabsf(IMU_Data.G[NOW][k]) > GYRO_SAT_DPS)
                            IMU_Data.G[NOW][k] = IMU_Data.G[LAST][k];
                    }
                    if (IMU_Data.calib_done)
                    {
                        for (int k = 0; k < 3; k++)
                            IMU_Data.G[NOW][k] -= IMU_Data.G_Offset[k];
                    }
                    IMU_Data.G[NOW][PITCH] = KalmanFilter(&IMU_Kalman_Filter[GYR][PITCH],IMU_Data.G[NOW][PITCH],IMU_KF_Q,IMU_KF_R);
                    IMU_Data.G[NOW][ROLL ] = KalmanFilter(&IMU_Kalman_Filter[GYR][ROLL ],IMU_Data.G[NOW][ROLL ],IMU_KF_Q,IMU_KF_R);
                    IMU_Data.G[NOW][YAW  ] = KalmanFilter(&IMU_Kalman_Filter[GYR][YAW  ],IMU_Data.G[NOW][YAW  ],IMU_KF_Q,IMU_KF_R);
                    for (int i = 0;i<3;i++)
                    {
                        IMU_Data.G_Rad[NOW][i] = DEG2RAD(IMU_Data.G[NOW][i]);
                        Surface.current_gyro_Euler[NOW][i] = IMU_Data.G[NOW][i];
                        IMU_Data.G[LAST][i] = IMU_Data.G[NOW][i];
                    }
                    receiveflag++;
                }break;
                case MAG:
                {
                    IMU_Data.M[NOW][X] = -(int16_t)((rx_buf[2] << 5) | (rx_buf[1]>>3));
                    IMU_Data.M[NOW][Y] = -(int16_t)((rx_buf[4] << 5) | (rx_buf[3]>>3));
                    IMU_Data.M[NOW][Z] = -(int16_t)((rx_buf[6] << 7) | (rx_buf[5]>>1))*0.3125f;
                }break;

            }

}

void BMX055_Write(uint8_t Sensor,uint8_t Reg_Addr,uint8_t data)
{

        BMX055_CS_Select(Sensor);

        for (volatile int i = 0; i < 10; i++);

        uint8_t tx_buf[2] = {Reg_Addr,data};
        uint8_t rx_buf[2] = {0,0};

        HAL_SPI_TransmitReceive( &hspi2,tx_buf,rx_buf,2 ,5000);

        BMX055_CS_Free(Sensor);

}

void BMX055_Init_Acc_Gyr(void)
{
     //配置范围与ODR+-16g,250Hz
     BMX055_Write( ACC,0x11,0x00 );
     BMX055_Write( ACC,0X0F,0X0C );
     BMX055_Write( ACC,0x10,0x1F );
     //+-2000.400hz
     BMX055_Write( GYR,0x11,0x00 );
     BMX055_Write( GYR,0x0F,0x01 );
     BMX055_Write( GYR,0x10,0x02 );

}
void BMX055_Init_Mag(void)
{
    BMX055_Write(MAG, 0x4B, 0x01);
    // HAL_Delay(10); // 手册要求：NVM数据加载延迟≥5ms
    BMX055_Write(MAG, 0x4C, 0x38);
    // 预期返回值：0x40，如果不是则通信有问题
}

void IMU_Data_Read(void)
{
    BMX055_Read(ACC,0X02);
    BMX055_Read(GYR,0X02);
     
    // BMX055_Read(MAG,0X42);
    // 应用三维卡尔曼滤波 (使用修改后的函数)
}

void IMU_Init(void)
{
    BMX055_Init_Acc_Gyr();

    // BMX055_Init_Mag();
}

/*
 * 静态零偏校准:上电后保持飞镖静止 2 秒,采样均值作为 gyro/acc 零偏。
 * 校准期间 calib_done=0,IMU_Data_Read 不减偏,保证采样的是真实零位。
 */
void IMU_Calibrate(void)
{
    const uint16_t N = 2000;
    float gsum[3] = {0}, asum[3] = {0};
    IMU_Data.calib_done = 0;
    for (uint16_t n = 0; n < N; n++)
    {
        IMU_Data_Read();
        for (int i = 0; i < 3; i++)
        {
            gsum[i] += IMU_Data.G[NOW][i];
            asum[i] += IMU_Data.A[NOW][i];
        }
        osDelay(1);
    }
    for (int i = 0; i < 3; i++)
    {
        IMU_Data.G_Offset[i] = gsum[i] / (float)N;
        IMU_Data.A_Offset[i] = asum[i] / (float)N;
    }
    IMU_Data.A_Offset[Z] -= 1.0f;
    IMU_Data.calib_done = 1;
}



#endif
