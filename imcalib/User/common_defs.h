//
// 公共宏定义（dT / M_PI / 单位换算 / 传感器量程）
// 所有需要这些常量的模块都 #include 它，避免重复宏定义
//

#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#ifndef dT
#define dT          0.001f
#endif

#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif

#define RAD2DEG(x)  ((x) * 57.29577951308232f)
#define DEG2RAD(x)  ((x) * 0.017453292519943295f)


#define IMU_SAMPLE_HZ        1000.0f
#define CTRL_PERIOD_MS       1
#define GYRO_LSB_2000DPS     16.4f          /* BMX055/BMI088 陀螺共用 */
#define GRAVITY_MS2          9.80665f

#define GYRO_SAT_DPS         1900.0f


/* 速度全局限幅(m/s):飞镖标称~7m/s,20 只截真正的 IMU 积分发散,不削正常飞行波动。
 * EKF publish() 内 abs_limit 钳位 X[3..5];IMU.c 冗余兜底非 EKF 路径。 */
#define VEL_MAX_MS            20.0f

#endif //COMMON_DEFS_H
