# 部署与使用说明书（DEPLOY_GUIDE）

> 面向**第一次拿到这套代码**的人。一步步教你：把环境跑起来 → 跑测试 → 看结果 → 改参数 → （阶段二）接 Simulink。
> 看完即可独立操作。算法原理与参数整定在 **[`README.md`](README.md)**。

---

## 0. 先看这张“最快上手”表

| 我想… | 在 MATLAB 命令窗口敲 | 看什么 |
|---|---|---|
| 跑综合演示 | `main_dart_sim` | 命令窗口指标 + `analysis/figs/MAIN_overview.png` |
| 验证核心指标(超调/调节时间) | `t3_attitude_step` | 打印 `M3 PASS=1` + `M3_attitude_step.png` |
| 验证鲁棒性(速度/风) | `t4_robustness` | 打印 `M4 PASS=1` + `M4_robustness.png` |
| 调完参数想确认稳定性 | `probe_gain` | wc×K 网格的超调/调节时间 |

> 前提：已按第 2 节 `cd` 到项目目录并 `addpath(genpath(pwd))`。

---

## 1. 运行前提

- **MATLAB**：R2024a（本项目开发/验证版本；R2020b 及以上应都可用）。
- **工具箱**：
  - **Control System Toolbox** —— 必需（外环用到 `lqr`）。
  - Simulink / Aerospace Toolbox —— 仅阶段二（Simulink 模型）需要；阶段一纯脚本不依赖。
- **本机 MATLAB 可执行文件位置**（本环境）：
  ```
  E:\DevelopLotteany1\Matlab\app\bin\matlab.exe
  ```
- **项目根目录**：
  ```
  E:\DevelopLotteany1\Simulink_Guidence_lqr_adrc
  ```

检查工具箱是否齐全：在命令窗口敲 `ver`，看列表里有没有 “Control System Toolbox”。

---

## 2. 启动 MATLAB 并加载项目（两种方式）

### 方式 A：图形界面（推荐日常使用，中文显示正常）

1. 双击打开 MATLAB。
2. 在**命令窗口（Command Window）**里把当前目录切到项目根，并把子目录加入搜索路径：
   ```matlab
   cd 'E:\DevelopLotteany1\Simulink_Guidence_lqr_adrc'
   addpath(genpath(pwd))      % 把 models/ control/ analysis/ tests/ 全部加入路径
   ```
   > `genpath(pwd)` 会递归加入所有子文件夹，这样才能跨目录调用 `dart_6dof`、`ladrc_inner` 等函数。
3. 之后直接敲脚本名运行（见第 3 节）。

### 方式 B：命令行无界面运行（适合批量/自动化）

在系统终端（PowerShell / Git Bash）里：
```bash
"E:/DevelopLotteany1/Matlab/app/bin/matlab.exe" -batch "cd('E:/DevelopLotteany1/Simulink_Guidence_lqr_adrc'); addpath(genpath(pwd)); main_dart_sim"
```
- `-batch "..."`：无界面执行引号里的命令，跑完自动退出，控制台打印 `fprintf` 的输出。
- 把末尾 `main_dart_sim` 换成 `t3_attitude_step`、`t4_robustness` 等即可跑对应脚本。
- **图仍会照常保存**到 `analysis/figs/`（脚本里用 `Visible','off'` + `saveas`，无界面也能出图）。

> ⚠️ **中文乱码提醒**：用 `-batch` 在某些终端里，命令窗口打印的**中文可能显示为乱码**（终端代码页问题），
> 但**数字、PASS 判定、生成的 PNG 图全部正常**。想看正常中文输出，用**方式 A 的图形界面**运行即可。

---

## 3. 跑各个脚本（精确命令 + 预期输出）

下面以**方式 A**（已在命令窗口、已 `addpath`）为准；方式 B 把脚本名套进 `-batch` 即可。

### 3.1 主场景综合演示
```matlab
main_dart_sim
```
预期命令窗口（数字为参考值）：
```
================= MAIN 6DOF 自由飞行标准场景 =================
  仿真: T=4.0s, 1000Hz, 初速 60 m/s, Dryden紊流σ=1.0 + 常值侧风 2.5 m/s(东)
  弹道: 空速 60.0->62.4 m/s, 弹道倾角 -1.0->-35.0 deg, 下坠 75.5 m
  滚转机动跟踪(2.5~3s保持10°): 误差 -0.029 deg
  俯仰顺气流: 攻角α RMS=1.59 deg
  偏航增稳: 全程最大 |psi|=2.10 deg (侧滑β RMS=0.87 deg)
  峰值舵偏 48.7 deg (上限90°)，触饱和步数 124/4000
完成。综合报告图已保存到 analysis/figs/MAIN_overview.png
```

### 3.2 分项测试（M1~M4）
```matlab
t1_openloop        % M1 开环动力学合理性 → 打印 M1 PASS=1，出 M1_openloop.png
t2_inner_rate      % M2 内环角速度闭环+抗扰 → 打印 M2 PASS=1，出 M2_inner_rate.png
t3_attitude_step   % M3 姿态阶跃核心指标 → 打印 M3 PASS=1，出 M3_attitude_step.png
t4_robustness      % M4 速度衰减+风扰 → 打印 M4 PASS=1，出 M4_robustness.png
```
判定标准：每个脚本末尾打印 `>>> Mx PASS = 1` 即通过（`0` 为不通过）。

### 3.3 整定/诊断工具
```matlab
probe_gain         % 扫 wc×K 网格，看超调/调节时间随增益怎么变（改完 Q 或 wc 跑它确认稳定性）
probe_main         % 诊断主场景舵饱和来源（扫风强度，定位是哪个轴/哪个舵饱和）
```

---

## 4. 怎么看结果

### 4.1 命令窗口指标怎么读

- `OS%` = 超调百分比（要 <20%）；`ts` = 调节时间秒（要 <0.5s @50m/s）；
- `ess` = 稳态误差（越接近 0 越好）；`maxDelta` = 峰值舵偏（接近 25° 说明接近饱和）；
- `PASS = 1` 表示该里程碑全部判据通过。

### 4.2 图在哪、每张图看什么

所有图自动存到 **`analysis/figs/`**：

| 文件 | 来自 | 重点看 |
|---|---|---|
| `MAIN_overview.png` | main | 九宫格：①空速 ②姿态跟踪 ③角速度 ④力矩 ⑤4舵偏(±90°红线) ⑥LESO扰动估计 ⑦风 ⑧攻角/侧滑 ⑨3D弹道 |
| `M1_openloop.png` | t1 | 速度先降后升、姿态阻尼收敛 |
| `M2_inner_rate.png` | t2 | 角速度跟上指令、z2 估出注入扰动 |
| `M3_attitude_step.png` | t3 | 阶跃响应无超调、快速进带、偏航几乎不动 |
| `M4_robustness.png` | t4 | 各速度响应一致(动压调度)、风扰下姿态稳 |

**怎么判断主场景“干净”**：看 `MAIN_overview.png` 的子图⑤——4 条舵偏曲线应在 ±25° 红线**以内**平滑变化，不贴着红线来回颤（颤=饱和/失稳）。

---

## 5. 改参数的标准工作流

所有参数在 **`dart_params.m`**。推荐流程（改完务必按顺序验证，别直接看 main）：

```
①改 dart_params.m  →  ②probe_gain 确认 ζ 在 0.7~1 甜区  →  ③t3_attitude_step 看指标
                    →  ④t4_robustness 看鲁棒性  →  ⑤main_dart_sim 看综合演示
```

常见调整示例：

- **想让姿态响应更快**：把 `params.lqr.Q` 整体乘 1.5（保持 36:36:3.6 比例）→ K 变大。
  **但必须**回头跑 `probe_gain` 确认阻尼比没掉出甜区（K 变大、wc 不变会让 ζ 变小、超调上升）。详见 README §4.2。
- **想换实物舵机**：改 `params.servo.wn/zeta/pos_max/rate_max`。
- **想换真实气动数据**：替换 `params.aero.*`（接口不变，LADRC 对模型误差不敏感）。
- **想做更猛的抗风演示**：改 `main_dart_sim.m` 里的 `windSig`、`windSteady`（会更接近饱和，见 README §4.6）。

> 黄金法则：**一次只改一个参数**，跑一遍看影响，再改下一个。同时改多个会分不清谁起的作用。

---

## 6. 常见问题排查

| 现象 | 原因 | 解决 |
|---|---|---|
| `Undefined function 'dart_6dof'...` | 没加路径 | 先 `cd` 到项目根再 `addpath(genpath(pwd))` |
| `Undefined function 'lqr'` | 缺 Control System Toolbox | 安装该工具箱（`ver` 检查） |
| 命令窗口中文是乱码 | `-batch` 终端代码页问题 | 改用 MATLAB **图形界面**运行（方式 A），数字/图不受影响 |
| 没看到图弹出来 | 脚本用 `Visible','off'` 存盘不弹窗 | 去 `analysis/figs/` 看 PNG；想弹窗可临时把 `'Visible','off'` 改 `'on'` |
| 结果出现 NaN/Inf 或发散 | 参数改过头（如 K 过大、wc/K 失衡、舵机带宽过低） | 跑 `probe_gain` 看是否离开甜区；回退到 `dart_params.m` 默认值对照 |
| 低速时舵偏顶到 25° | 正常的**幅度上限**（动压低、大机动） | 不是 bug；看不饱和区指标，或减小机动幅度（README §4.5） |
| 偏航对侧风有残留偏离 | **设计上的弱轴**（Cndr 极小） | 正常；偏航只增稳不消除（README §4.4 / §5.3） |

---

## 7. 阶段二：接通 Simulink（MCP）

阶段一（纯脚本）已验证全部算法与指标。要做 Simulink 模型 `dart_control.slx`：

1. 确认本机 MATLAB 已启用 **MATLAB 的 MCP / 外部工具接口**（提供 `model_edit` 等工具）。
2. **重启 Claude Code** 以接通 MCP（首次接通需要重启）。
3. 然后让我用 `model_edit` 搭建：Plant / Controller / Allocation / Servo / Disturbance 子系统 + Bus + 定步长 ode4 + Saturation/Rate Limiter。
4. 脚本版每个函数都能直接落成 Simulink 的 **MATLAB Function 块**（接口已对齐），逻辑无需重写。

---

## 8. 一键自检（确认环境 OK）

把下面整段粘进命令窗口，全绿(PASS=1)即部署成功：
```matlab
cd 'E:\DevelopLotteany1\Simulink_Guidence_lqr_adrc'; addpath(genpath(pwd));
t1_openloop; t2_inner_rate; t3_attitude_step; t4_robustness; main_dart_sim;
disp('==== 若上面四个 PASS 均为 1、且 figs/ 下生成 5 张 PNG，则环境与代码均正常 ====');
```
