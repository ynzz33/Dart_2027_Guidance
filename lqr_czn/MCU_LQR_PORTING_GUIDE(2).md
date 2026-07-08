# 飞镖 LQR 姿态控制器单片机移植说明（K_d(V) 方程调度版）

本文档用于指导 AI 或嵌入式工程师把 `dart_attitude_LQR_v1.m` 生成的离散 LQR 姿态控制器移植到单片机 C 代码中。

当前版本不再导出速度查表，而是像 HerKules 一样用 `matlabFunction` 生成速度方程函数。MATLAB 会按 `3.0~5.0 m/s`、`0.1 m/s` 步长重算 `dlqr`，再把结果拟合为 `K_d(V)` 方程。

## 1. 控制接口

状态顺序必须固定：

```c
x[0] = roll_err;   // rad
x[1] = pitch_err;  // rad
x[2] = yaw_err;    // rad
x[3] = p;          // rad/s
x[4] = q;          // rad/s
x[5] = r;          // rad/s
```

控制律：

```c
u = -K_d(V) * x
```

输出顺序：

```c
delta_cmd[0] = delta1; // 右上翼面
delta_cmd[1] = delta2; // 左上翼面
delta_cmd[2] = delta3; // 左下翼面
delta_cmd[3] = delta4; // 右下翼面
```

所有角度用 `rad`，角速度用 `rad/s`，速度 `V` 用 `m/s`。

## 2. MATLAB 生成的方程文件

运行 `dart_attitude_LQR_v1.m` 后会生成：

```matlab
LQR_K_Dart.m      % 连续 LQR K(V)
LQR_K_Dart_d.m    % 离散 LQR K_d(V)，单片机主要移植这个
LQR_A_Dart.m      % 连续 A(V)
LQR_B_Dart.m      % 连续 B(V)
LQR_A_Dart_d.m    % 离散 A_d(V)
LQR_B_Dart_d.m    % 离散 B_d(V)
```

使用方式：

```matlab
K_d = LQR_K_Dart_d(V);
```

有效拟合范围：

```text
3.0 m/s <= V <= 5.0 m/s
```

如果修改惯量、气动参数、`Q/R`、速度范围、气动恢复/阻尼开关，必须重新运行脚本并重新生成这些函数。

## 3. C 端移植骨架

推荐把 `LQR_K_Dart_d.m` 中生成的方程翻译到 `DartLqr_GetKdByEquation()`，或者用 MATLAB Coder 生成 C。

```c
#include <stdint.h>

#define DART_LQR_STATE_NUM 6
#define DART_LQR_SERVO_NUM 4

#define DART_LQR_V_MIN 3.0f
#define DART_LQR_V_MAX 5.0f

static const float DART_DELTA_MAX_RAD = 1.0471975511965976f; // +/-60 deg

static inline float clamp_f32(float x, float min_val, float max_val)
{
    if (x > max_val) {
        return max_val;
    }
    if (x < min_val) {
        return min_val;
    }
    return x;
}

static void DartLqr_GetKdByEquation(float V, float K[DART_LQR_SERVO_NUM][DART_LQR_STATE_NUM])
{
    float Vc = clamp_f32(V, DART_LQR_V_MIN, DART_LQR_V_MAX);

    // TODO: 把 LQR_K_Dart_d.m 里的 K_d_sym 方程移植到这里。
    // 输出 K[4][6]。
    (void)Vc;
}

void DartLqr_UpdateScheduled(float V, const float x[DART_LQR_STATE_NUM], float delta_cmd[DART_LQR_SERVO_NUM])
{
    uint8_t i;
    uint8_t j;
    float K[DART_LQR_SERVO_NUM][DART_LQR_STATE_NUM];

    DartLqr_GetKdByEquation(V, K);

    for (i = 0; i < DART_LQR_SERVO_NUM; i++) {
        float u = 0.0f;

        for (j = 0; j < DART_LQR_STATE_NUM; j++) {
            u -= K[i][j] * x[j];
        }

        delta_cmd[i] = clamp_f32(u, -DART_DELTA_MAX_RAD, DART_DELTA_MAX_RAD);
    }
}
```

## 4. 实时任务调用

```c
void ControlTask_1kHz(void)
{
    float x[6];
    float delta_cmd[4];
    float V_mps;

    x[0] = roll_err_rad;
    x[1] = pitch_err_rad;
    x[2] = yaw_err_rad;
    x[3] = gyro_p_radps;
    x[4] = gyro_q_radps;
    x[5] = gyro_r_radps;

    DartLqr_UpdateScheduled(V_mps, x, delta_cmd);

    // 后面做舵机零位、安装方向、rad->deg/PWM/CAN 映射。
}
```

暂时没有实时速度时，可以先固定：

```c
V_mps = 4.0f;
```

## 5. 对拍要求

移植后必须和 MATLAB 对拍：

```matlab
V = 4.0;
x = [deg2rad(15); deg2rad(10); deg2rad(10); 0; 0; 0];
u = -LQR_K_Dart_d(V) * x;
u = min(max(u, -deg2rad(60)), deg2rad(60));
```

C 端在相同 `V` 和 `x` 下，限幅前后的结果都应一致。再用 `V=4.25` 做一次非中心速度点对拍。

## 6. 上车注意

- `G` 符号、舵机编号、舵机安装方向必须逐片核对。
- `V < 3` 时夹到 `3 m/s`，`V > 5` 时夹到 `5 m/s`。
- IMU 未初始化、速度无效、姿态异常时应输出 0 舵偏或进入保护。
- 实物翼面若不能承受 `60 deg`，必须按真实机械/气动极限重设限幅。
