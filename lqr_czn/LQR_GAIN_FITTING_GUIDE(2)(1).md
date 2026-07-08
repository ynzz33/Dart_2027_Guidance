# 飞镖 LQR 速度方程拟合说明

本文档说明 `dart_attitude_LQR_v1.m` 中 `K_d(V)`、`A(V)`、`B(V)`、`A_d(V)`、`B_d(V)` 的速度方程拟合与导出流程。

当前目标不是让单片机保存一整张速度查表，而是像 HerKules 一样导出可调用的 MATLAB 方程文件：

```matlab
K_d = LQR_K_Dart_d(V);
```

后续可以把这个方程文件手动移植到 C，或用 MATLAB Coder 生成 C。

气动恢复/阻尼参数本身的辨识入口脚本在：

```matlab
dart_aero_A_calibration_from_csv.m
```

所有 CSV 导入、数据清洗、求导、扣除舵面输入、最小二乘拟合和结果保存都集中在这个文件里。它用于从飞行日志拟合 `Dp_ref_ac / Cm_pitch_ref_ac / Dq_ref_ac / Cm_yaw_ref_ac / Dr_ref_ac`。

CSV 日志至少需要列：

```text
t, d_phi, d_theta, d_psi, p, q, r, delta1, delta2, delta3, delta4, V
```

单位必须是 `s / rad / rad/s / rad / m/s`。

## 1. 拟合对象

脚本会在速度范围内逐点计算以下矩阵：

```matlab
K_schedule      % 连续 LQR K(V)
K_d_schedule    % 离散 LQR K_d(V)
A_schedule      % 连续 A(V)
B_schedule      % 连续 B(V)
A_d_schedule    % 离散 A_d(V)
B_d_schedule    % 离散 B_d(V)
```

其中单片机姿态控制主要使用：

```matlab
K_d(V)
```

控制律保持：

```matlab
u = -K_d(V) * x
```

状态顺序必须保持：

```matlab
x = [d_phi, d_theta, d_psi, p, q, r]'
```

## 2. 速度采样

当前速度采样设置在 `dart_attitude_LQR_v1.m` 参数区：

```matlab
V_ac = 4.0;
V_schedule_ac = 3.0:0.1:5.0;
V_ref_ac = 4.0;
K_poly_order_ac = 5;
```

含义：

- `V_ac`：单点仿真和单点 `K_d` 输出使用的标称速度。
- `V_schedule_ac`：用于拟合的速度采样点，当前是 `3.0~5.0 m/s`，步长 `0.1 m/s`。
- `V_ref_ac`：多项式中心速度，当前为 `4.0 m/s`。
- `K_poly_order_ac`：拟合多项式阶数，当前为 5 阶。

## 3. 多项式形式

为了避免直接用 `V^5` 造成数值尺度偏大，拟合时使用中心化变量：

```matlab
s = V - V_ref_ac
```

每个矩阵元素单独拟合：

```matlab
M_ij(V) = c0 + c1*s + c2*s^2 + c3*s^3 + c4*s^4 + c5*s^5
```

其中 `M` 可以是：

```matlab
K, K_d, A, B, A_d, B_d
```

脚本中的拟合函数是：

```matlab
[poly_coeff, max_abs_err] = fit_matrix_schedule(matrix_schedule, V_fit_offset, poly_order);
```

`poly_coeff(row, col, :)` 保存对应矩阵元素的多项式系数，顺序是：

```matlab
[c0, c1, c2, c3, c4, c5]
```

## 4. 方程导出

脚本最后使用 `matlabFunction` 导出：

```matlab
matlabFunction(K_sym,   'File', 'LQR_K_Dart',   'Vars', {V});
matlabFunction(K_d_sym, 'File', 'LQR_K_Dart_d', 'Vars', {V});
matlabFunction(A_sym,   'File', 'LQR_A_Dart',   'Vars', {V});
matlabFunction(B_sym,   'File', 'LQR_B_Dart',   'Vars', {V});
matlabFunction(A_d_sym, 'File', 'LQR_A_Dart_d', 'Vars', {V});
matlabFunction(B_d_sym, 'File', 'LQR_B_Dart_d', 'Vars', {V});
```

生成文件：

```matlab
LQR_K_Dart.m
LQR_K_Dart_d.m
LQR_A_Dart.m
LQR_B_Dart.m
LQR_A_Dart_d.m
LQR_B_Dart_d.m
```

其中上车主要用：

```matlab
LQR_K_Dart_d.m
```

## 5. 拟合误差检查

运行脚本后会打印每类矩阵的最大拟合误差，例如：

```text
Polynomial fit order = 5
  K(V)   max abs error = ...
  K_d(V) max abs error = ...
  A(V)   max abs error = ...
  B(V)   max abs error = ...
  A_d(V) max abs error = ...
  B_d(V) max abs error = ...
```

当前一次验证结果中：

```text
K_d(V) max abs error ≈ 7.12e-6
LQR_K_Dart_d(4.0) vs 单点 K_d 最大误差 ≈ 4.78e-6
```

这个量级对单精度单片机控制是可以接受的。

## 6. 如何对拍

MATLAB 侧：

```matlab
V = 4.0;
x = [deg2rad(15); deg2rad(10); deg2rad(10); 0; 0; 0];
u = -LQR_K_Dart_d(V) * x;
u = min(max(u, -deg2rad(60)), deg2rad(60));
```

C 端用同样的 `V` 和 `x` 计算，检查：

- 限幅前 `u_raw` 是否一致。
- 限幅后 `u` 是否一致。
- 再用 `V = 4.25` 做一次非中心速度点对拍。

## 7. 修改速度范围或阶数

如果实测速度不是 `3~5 m/s`，只改参数区：

```matlab
V_schedule_ac = 2.5:0.1:5.5;
V_ref_ac = 4.0;
K_poly_order_ac = 5;
```

建议：

- 速度范围变宽后，先保持 `0.1 m/s` 采样。
- 如果 `K_d(V)` 拟合误差变大，再提高 `K_poly_order_ac`。
- 不要盲目把阶数调太高，避免边界振荡。

## 8. 气动 A 开关的影响

当前脚本有开关：

```matlab
DART_LQR_ENABLE_AERO_A = false;
```

关闭时：

```matlab
A = [0 I; 0 0]
```

开启后：

```matlab
A(V)
```

会加入辨识得到的气动恢复/阻尼项。此时必须重新运行脚本，因为 `K_d(V)`、`A(V)`、`A_d(V)` 的方程都会改变。

## 9. 上车使用边界

方程拟合只保证在采样范围内可靠：

```text
3.0 m/s <= V <= 5.0 m/s
```

单片机端建议先夹紧速度：

```c
V = clamp(V, 3.0f, 5.0f);
```

不要在范围外直接外推使用 `K_d(V)`。
