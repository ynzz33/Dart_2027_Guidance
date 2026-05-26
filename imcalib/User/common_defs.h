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
#define GYRO_LSB_2000DPS     16.4f
#define ACC_LSB_16G          (1.0f/(16.0f*128.0f))
#define GRAVITY_MS2          9.80665f

#define GYRO_SAT_DPS         1900.0f
#define ACC_SAT_G            15.5f

#endif //COMMON_DEFS_H
