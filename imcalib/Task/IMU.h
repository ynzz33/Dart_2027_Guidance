//
// Created by ynz on 2025/11/17.
//
#include "mytype.h"
#include "common_defs.h"
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

/* Mahony 互补滤波:预测(陀螺积分)-校正(加速度拉回重力方向),即"更聪明的输入滤波" */
#define mahony_MAXOUT   10.00f   /* 修正量限幅,防加速度突变把姿态拉飞 */
#define mahony_i_maxout  1.00f   /* 积分限幅 */
#define mahony_Kp        10.0f    /* 加速度校正强度:大→快速消陀螺漂移但易被振动带歪;小→抗扰好但收敛慢 */
#define mahony_Ki        0.01f   /* 估计陀螺零偏残差,通常 0.005~0.02 */
#define mahony_Kd        0.0f    /* 必须为 0:标准 Mahony 只有 PI,D 项会放大噪声 */

#define GYR_KF_Q 1.0f
#define GYR_KF_R 1000.0f
#define ACC_KF_Q 1.0f
#define ACC_KF_R 1000.0f

/* === 传感器→机体系符号 (机体 X=右/东, Y=前/北, Z=上/天, 右手 ENU) ===
 * 每轴正方向只能台架实测锁定(BMX055 加速度/陀螺两片封装轴向本就不同);默认按当前硬件,
 * 按「验证表」做单轴动作,某轴方向反了就把对应符号翻 ±1。
 *
 * 加速度物理映射(寄存器序 rx2,1=accX / rx4,3=accY / rx6,5=accZ):
 *   A[X](右)=accY寄存器, A[Y](前)=accX寄存器, A[Z](上)=accZ寄存器 —— 见 BMX055_Read,静止 A[Z]≈+g(上为正)。 */
#define ACC_SIGN_X  (+1.0f)   /* 机体X 右/东:  + = 向右加速度为正 */
#define ACC_SIGN_Y  (+1.0f)   /* 机体Y 前/北:  + = 向前(发射方向)加速度为正 */
#define ACC_SIGN_Z  (-1.0f)   /* 机体Z 上/天:  使静止读数 = +g(上为正) */

/* 陀螺喂四元数的机体角速度(右手 rad/s): gx(绕右)=G[PITCH]=chipGyrY, gy(绕前)=G[ROLL]=chipGyrX(纵轴),
 * gz(绕上)=G[YAW]=chipGyrZ。右手系下 +gx=抬头、+gy=右滚、+gz=左偏(故上报 yaw 取 −gz)。 */
#define GYR_SIGN_X  (+1.0f)   /* 绕机体X(右): + 应 = 抬头(右手) */
#define GYR_SIGN_Y  (+1.0f)   /* 绕机体Y(前): + 应 = 右滚(右手) */
#define GYR_SIGN_Z  (-1.0f)   /* 绕机体Z(上): + 应 = 左偏(右手);上报 yaw 右+ = −此值 */

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
    float G_Offset[3];
    float A_Offset[3];
    uint8_t calib_done;
}
IMU_DATA_t;


extern IMU_DATA_t IMU_Data;
void IMU_Init(void);
void IMU_Calibrate(void);
void BMX055_Write(uint8_t Sensor,uint8_t Reg_Addr,uint8_t data);
void BMX055_Read(uint8_t Sensor,uint8_t Reg_Addr);
void IMU_Data_Read(void);
void IMU_Attitude_Algorithm(void);
void ALL_CS_Free(void);
extern uint32_t IMU_Cnt,control_cnt;
#endif //IMU_H
