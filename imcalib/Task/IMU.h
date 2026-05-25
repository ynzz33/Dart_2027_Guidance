//
// Created by ynz on 2025/11/17.
//
#include "mytype.h"
#ifndef IMU_H
#define IMU_H


#define ACC_CS_PORT GPIOB
#define ACC_CS_PIN GPIO_PIN_12

#define GYR_CS_PORT GPIOA
#define GYR_CS_PIN GPIO_PIN_8

#define HIGH_ACC_CS_PORT GPIOB
#define HIGH_ACC_CS_PIN GPIO_PIN_11

#define MAG_CS_PORT GPIOA
#define MAG_CS_PIN GPIO_PIN_11

#define dT 0.001f

#define mahony_MAXOUT 10.00f
#define mahony_i_maxout 0.0f
#define mahony_Kp 5.0f
#define mahony_Ki 0.00f
#define mahony_Kd 0.5f

#define IMU_KF_Q 1.0f
#define IMU_KF_R 5000.0f


// #define M_PI 3.14159265358979323846
#define M_PI 3.14159

enum
{
    ACC = 0,
    GYR = 1,
    MAG = 2,
};

enum
{
    PITCH,
    ROLL,
    YAW,
};

enum
{
    X  ,
    Y  ,
    Z  ,
};

enum
{
    NOW,
    LAST,
    LLAST
};

enum
{
    World,
    Body,
};
typedef struct
{
    //2000°，+-16g
    float G[2][3];//gyr
    float A[2][3];//acc
    float M[2][3];//mag
    float G_Rad[2][3];//GYR_Data_Rad
    float Q[2][4];//四元数定义顺序是W.X.Y.Z
    float Euler[2][3];
    float R_matrix_T[3][3];
    float A_Normed[2][3];
    float A_theory[2][3];
    float A_World[2][3];
    float Velocity[2][2][3];
    float temp[2][3];
}
IMU_DATA_t;


extern IMU_DATA_t IMU_Data;
void IMU_Init(void);
void BMX055_Write(uint8_t Sensor,uint8_t Reg_Addr,uint8_t data);
void BMX055_Read(uint8_t Sensor,uint8_t Reg_Addr);
void IMU_Data_Read(void);
void IMU_Attitude_Algorithm(void);
void ALL_CS_Free(void);
extern uint32_t IMU_Cnt,control_cnt;
#endif //IMU_H
