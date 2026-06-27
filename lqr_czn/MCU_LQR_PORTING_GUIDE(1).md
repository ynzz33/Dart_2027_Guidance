# 飞镖 LQR 姿态控制器单片机移植说明

本文档用于指导 AI 或嵌入式工程师把 `dart_attitude_LQR_v1.m` 生成的离散 LQR 姿态控制器移植到单片机 C 代码中。

当前控制器是第一版纯姿态内环，只负责姿态镇定，不负责落点外环、不做速度增益调度、不估计气动参数。

## 1. 控制器功能

控制目标：把飞镖三轴姿态角偏差收敛到 0。

输入状态：

```c
x = [roll_err, pitch_err, yaw_err, p, q, r]
```

输出控制量：

```c
u = [delta1, delta2, delta3, delta4]
```

控制律：

```c
u = -K_d * x
```

其中：

- `roll_err`、`pitch_err`、`yaw_err` 是相对期望姿态或配平姿态的角度偏差，单位必须是 `rad`。
- `p`、`q`、`r` 是 IMU 输出的机体角速度，单位必须是 `rad/s`。
- `delta1` 到 `delta4` 是四个舵机目标偏转角，单位是 `rad`。
- 控制周期为 `Ts = 0.001 s`，即 1 kHz。

## 2. 坐标系和状态顺序

单片机代码中必须严格使用以下状态顺序：

```c
x[0] = roll_err;   // d_phi,   rad
x[1] = pitch_err;  // d_theta, rad
x[2] = yaw_err;    // d_psi,   rad
x[3] = p;          // roll rate,  rad/s
x[4] = q;          // pitch rate, rad/s
x[5] = r;          // yaw rate,   rad/s
```

角度和角速度都按右手定则。

如果 IMU 输出是 degree 或 degree/s，必须先转换：

```c
rad = deg * 0.017453292519943295f;
```

## 3. 舵机编号和输出顺序

MATLAB 脚本中的舵机编号约定如下：

```text
delta1 = 右上翼面
delta2 = 左上翼面
delta3 = 左下翼面
delta4 = 右下翼面
```

输出顺序必须保持：

```c
servo_cmd[0] = delta1;
servo_cmd[1] = delta2;
servo_cmd[2] = delta3;
servo_cmd[3] = delta4;
```

如果实物舵机编号不同，只能在单片机的舵机映射层换线，不要随意改 LQR 的矩阵行顺序。

## 4. 当前 MATLAB 导出的 K_d

当前脚本占位参数下导出的增益为：

```c
static const float dart_lqr_K[4][6] = {
    {0.68200338607834365f, 4.1093401356116086f, 4.1102227833110927f, 0.048926647296854101f, 0.25014331276162821f, 0.25020888306918981f},
    {0.68200338607853273f, -4.1093401356115509f, 4.1102227833111717f, 0.048926647296854343f, -0.25014331276162804f, 0.25020888306919059f},
    {0.68200338607854405f, -4.1093401356116068f, -4.1102227833110936f, 0.04892664729685426f, -0.25014331276162771f, -0.25020888306918965f},
    {0.68200338607835786f, 4.10934013561155f, -4.1102227833111824f, 0.048926647296854094f, 0.25014331276162804f, -0.25020888306919065f}
};
```

注意：这些数值来自当前 MATLAB 脚本中的占位惯量、占位气动参数和 Q/R。替换 SolidWorks 转动惯量、实际速度和调参权重后，必须重新运行 MATLAB 并更新这组 K。

## 5. C 端核心计算函数

推荐先移植成一个独立函数，不要一开始就混进姿态解算或舵机驱动中。

```c
#include <stdint.h>

#define DART_LQR_STATE_NUM 6
#define DART_LQR_SERVO_NUM 4

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

void DartLqr_Update(const float x[DART_LQR_STATE_NUM], float delta_cmd[DART_LQR_SERVO_NUM])
{
    for (uint8_t i = 0; i < DART_LQR_SERVO_NUM; i++) {
        float u = 0.0f;

        for (uint8_t j = 0; j < DART_LQR_STATE_NUM; j++) {
            u -= dart_lqr_K[i][j] * x[j];
        }

        delta_cmd[i] = clamp_f32(u, -DART_DELTA_MAX_RAD, DART_DELTA_MAX_RAD);
    }
}
```

重要：MATLAB 中的控制律是 `u = -K_d * x`，所以 C 代码里必须有负号。不要把 `K` 预先取负，除非你在变量名和注释里明确写成 `minus_K`。

## 6. 实时任务调用位置

建议 1 kHz 姿态控制任务中按以下顺序执行：

```c
void ControlTask_1kHz(void)
{
    float x[6];
    float delta_cmd[4];

    // 1. 从 IMU / AHRS 读取姿态和机体角速度。
    // 2. 计算相对期望姿态的误差，单位 rad。
    // 3. 填充状态 x，顺序必须严格一致。
    x[0] = roll_err_rad;
    x[1] = pitch_err_rad;
    x[2] = yaw_err_rad;
    x[3] = gyro_p_radps;
    x[4] = gyro_q_radps;
    x[5] = gyro_r_radps;

    // 4. LQR 计算四片翼面目标偏转角。
    DartLqr_Update(x, delta_cmd);

    // 5. 把 rad 转换成舵机角度闭环需要的单位。
    // 6. 做舵机零位、安装方向、PWM/CAN 输出映射。
}
```

## 7. 角度误差来源

第一版 LQR 只吃姿态误差，不生成期望姿态。

如果暂时没有落点外环，可以先令期望姿态为发射后某个配平姿态：

```c
roll_err  = roll_meas  - roll_ref;
pitch_err = pitch_meas - pitch_ref;
yaw_err   = yaw_meas   - yaw_ref;
```

推荐先用小角度调试。不要一开始让 yaw 跨越 `+-pi` 边界。

如果使用欧拉角，yaw 误差需要做 wrap：

```c
float wrap_pi(float a)
{
    while (a > 3.1415926535897932f) {
        a -= 6.2831853071795864f;
    }
    while (a < -3.1415926535897932f) {
        a += 6.2831853071795864f;
    }
    return a;
}
```

## 8. 舵机安装方向映射

LQR 输出的 `delta_i` 是理论翼面偏转角。实际舵机通常还需要以下映射：

```c
servo_angle[i] = servo_zero[i] + servo_dir[i] * delta_cmd[i];
```

其中：

- `servo_zero[i]` 是机械零位。
- `servo_dir[i]` 只能取 `+1` 或 `-1`，用于补偿舵机安装方向。
- `delta_cmd[i]` 单位是 rad，如果舵机接口用 degree，需要转换。

示例：

```c
servo_deg[i] = servo_zero_deg[i] + servo_dir[i] * delta_cmd[i] * 57.29577951308232f;
```

## 9. 上车前必须做的符号检查

这是最重要的一节。飞镖是一次性的，G 矩阵符号反了会形成正反馈。

建议先断开真实发射流程，只做地面小幅舵面检查：

1. 人为给 `roll_err > 0`，观察四个 `delta` 是否同向。
2. 人为给 `pitch_err > 0`，观察是否符合 pitch 两两差动预期。
3. 人为给 `yaw_err > 0`，观察是否符合 yaw 两两差动预期。
4. 人为给 `p > 0`、`q > 0`、`r > 0`，确认阻尼方向是反向抑制角速度。
5. 对照实物气动力方向确认舵面偏转产生的是“把误差打回 0”的力矩。

如果方向错，优先改 `servo_dir[i]` 或舵机映射层。只有确认 MATLAB 的舵机编号约定和实物编号约定不一致时，才改 `K` 的行顺序。

## 10. 饱和和安全策略

当前 MATLAB 仿真限幅为：

```c
DART_DELTA_MAX_RAD = 1.0471975511965976f; // 60 deg
```

即每个舵机：

```text
-60 deg <= delta_i <= +60 deg
```

实际飞行时建议保留限幅。若实物翼面 60 deg 已经失速或机械干涉，应按真实机械/气动极限重新设置。

可选安全策略：

- IMU 未初始化完成时，输出 0 舵偏。
- 姿态角异常、NaN、Inf 时，输出 0 舵偏或进入保护。
- 发射前不启用 LQR，发射检测后延时若干毫秒再启用。
- 舵机命令做最终机械限幅。

## 11. 与 MATLAB 对拍

移植完成后，必须做一次 C 和 MATLAB 对拍。

选一个固定状态，例如：

```c
x = [deg2rad(15), deg2rad(10), deg2rad(10), 0, 0, 0]
```

单片机或 PC 端 C 程序算出的 `delta_cmd` 应该和 MATLAB 脚本第一步算出的 `u = -K_d*x` 一致，限幅前后都要分别检查。

当前这组初始扰动在 MATLAB 中会触到 `+-60 deg` 限幅，因此对拍时要确认：

- 限幅前 `u_raw` 数值一致。
- 限幅后 `delta_cmd` 被限制在 `+-1.04719755 rad`。

## 12. 需要从 MATLAB 同步到 C 的参数

每次重新运行 MATLAB 调参后，需要同步：

- `dart_lqr_K[4][6]`
- `DART_DELTA_MAX_RAD`
- 状态顺序说明
- 舵机输出顺序说明
- 是否更改了舵机编号或 G 矩阵符号

不要只替换部分行，也不要手动四舍五入到太少位数。建议至少保留 `float` 有效精度。

## 13. 第二版预留接口

当前 C 函数可以保持不变，后续第二版可以在外层增加：

- `K_d(V)` 速度增益调度
- 气动恢复/阻尼辨识后的新 A 矩阵
- 落点外环生成 `roll_ref/pitch_ref/yaw_ref`

如果做增益调度，建议接口变成：

```c
void DartLqr_UpdateScheduled(float V, const float x[6], float delta_cmd[4]);
```

第一版暂时不要实现调度，避免把结构验证和参数标定搅在一起。

