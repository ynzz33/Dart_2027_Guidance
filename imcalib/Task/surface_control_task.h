 //
// Created by ynz on 2025/11/27.
//
#include <stdint.h>

#ifndef SURFACE_CONTROL_TSAK_H
#define SURFACE_CONTROL_TSAK_H

#define  Wing_left_Channel    TIM_CHANNEL_2
#define  Wing_right_Channel   TIM_CHANNEL_3
#define  Vertical_fin_Channel TIM_CHANNEL_4

#define  ZERO_POINT 1500

#define  Wing_left_ZERO_POINT      1400  //变大向上，变小向下
#define  Wing_right_ZERO_POINT     1480  //变小向上，变大向下
#define  Vertical_fin_ZERO_POINT   1520

#define  Wing_up_Change_Angle       50.0f  // 250.0f
#define  Wing_down_Change_Angle    -100.0f  //-400.0f
#define  Vertical_fin_Change_Angle  250.0f  // 800.0f
/* === X 翼 4 舵机布局(VECTOR_NOZZLE 模式使用),不影响原飞翼路径 === */
//硬件原因导致左上舵机接在了 TIM3 上，所以单独写函数控制
#define  Servo_UL_Channel   TIM_CHANNEL_2   /* htim3 CH2 → PB5 - UP_LEFT (硬件原因独占 TIM3) */
#define  Servo_UR_Channel   TIM_CHANNEL_2   /* htim4 CH2 → PB7 - UP_RIGHT   */
#define  Servo_DR_Channel   TIM_CHANNEL_3   /* htim4 CH3 → PB8 - DOWN_RIGHT  */
#define  Servo_DL_Channel   TIM_CHANNEL_4   /* htim4 CH4 → PB9 - DOWN_LEFT   */

//镖体1 红色
// #define  Servo_UL_ZERO      1420
// #define  Servo_UR_ZERO      1585
// #define  Servo_DR_ZERO      1535
// #define  Servo_DL_ZERO      1500


// //镖体2 蓝色
#define  Servo_UL_ZERO      1440
#define  Servo_UR_ZERO      1405
#define  Servo_DR_ZERO      1530
#define  Servo_DL_ZERO      1420

/* X 翼物理装配方向系数:实际舵令 = SIGN ⊙ (逻辑阵·指令)。[UL,UR,DR,DL]=[−,+,+,−],
 * 左侧两片(UL,DL)取 −1 因左右舵机镜像安装;台架单轴阶跃标定,某片整体反了翻它的号。
 * SIGN 每片三轴共享、只修整片装反;轴间配对结构由逻辑阵的列决定(见 surface_control_task.c C 阵/Alloc_B)。*/
#define  SIGN_UL  (-1.0f)
#define  SIGN_UR  (+1.0f)
#define  SIGN_DR  (+1.0f)
#define  SIGN_DL  (-1.0f)

/* X 翼舵机机械偏转角限幅(度),统一替代散落的 60.0f 字面量 */
#define  SERVO_ANGLE_LIMIT  60.0f

/* === 控制分配(混控)参数 ===
 * Alloc_Mode 运行时切三档分配器(见 .c):0=旧 Servo_Mix_PitchPriority 对照,
 * 1=可调三轴限幅 Servo_Mix_AxisLimit,2=最小能量分配 Servo_Mix_MinEnergy。*/
#define  AXIS_LIMIT_PITCH   40.0f   /* 交付A:三轴各自独立限幅(度),可调 */
#define  AXIS_LIMIT_ROLL    40.0f
#define  AXIS_LIMIT_YAW     80.0f
#define  ALLOC_U_MAX        SERVO_ANGLE_LIMIT   /* 交付B:单舵物理上限 */
#define  ALLOC_GAIN         4.0f   /* 交付B:伪逆解标称增益。理想阵(BBᵀ=4I)下令最小能量解 Bᵀv/4 还 原成与三轴限幅/旧版同幅度(Bᵀv),复用 PID 标定;辨识非理想 B 后可重调 */

/* === 世界系 pitch/yaw 解算(roll 反旋) ===
 * 喂 PID 的 ZYX 欧拉 pitch/yaw 是世界系参考,PID 输出即世界系 pitch/yaw 力矩需求;
 * 但 X 翼舵面产生机体系力矩。机身横滚 Δ 后把该需求反旋到机体系再送混控(见 .c)。
 * SIGN: 横滚正向/舵面朝向的总符号,台架单轴阶跃验证后可翻 ±1。*/
#define  ROLL_WORLD_COMP_SIGN  (+1.0f)

/* === 末制导视觉目标斜坡(方案3:setpoint 端速率限制) ===
 * 视觉~20Hz,锁存目标每 50ms 阶跃刷新一次;直接喂阶跃目标会周期性冲击外环 P(及对误差微分时的 D)。
 * 在目标端(非反馈环,不引入 P/I 相位滞后)把阶跃摊成斜坡:每 tick 目标朝锁存终点最多走 VISION_TARGET_SLEW_DPS·dT 度。
 * 调大→更跟手(接近阶跃)、调小→更平滑但跟踪滞后增大;台架按抖动/跟踪权衡。*/
#define  VISION_TARGET_SLEW_DPS   5000.0f

 enum
{
    Wing_left,
    Wing_right,
    Vertical_fin
};

enum
{
    FIXED_WING = 0,
    VECTOR_NOZZLE = 1
};

/* X 翼舵机索引,值与 Wing_left/Wing_right/Vertical_fin 重合但语义不同,
 * 二者通过 DART_TYPE 分支区别使用,数据底仓共享 Surface.output_angle_Servo[3][4] */
enum
{
    UP_LEFT     = 0,
    UP_RIGHT    = 1,
    DOWN_RIGHT  = 2,
    DOWN_LEFT   = 3,
    SERVO_COUNT_X = 4
};

enum
{
    Self_Text_State,
    Start,
    Stable,
    Terminal,
    End,
    PROCESS_OK
};

enum
{
    Angle,
    Gyro
};


enum
{
    Self_Text_Failure,
    Self_Text_Success,
};

enum
{
    Self_Text_Vision,
    Self_Text_Dart_Trigeer,
    Self_Text_OK,
    Self_Text_Start,
};
/* 控制状态总仓。多数为 [NOW,LAST,LLAST] 历史槽 × [PITCH,ROLL,YAW] 或 4 舵(列序 UL/UR/DR/DL) */
typedef struct
{
    float output_angle_Servo [3][4];  /* 混控输出的 4 舵机械角 °(含 SIGN, ±SERVO_ANGLE_LIMIT 内) */
    float current_angle_Euler[3][3];  /* 当前欧拉角 °(来自 IMU_Data.Euler) */
    float target_angle_Euler [3][3];  /* 目标欧拉角 °(状态机/视觉锁存写入) */
    float current_gyro_Euler [3][3];  /* 当前角速度 °/s(串级内环反馈,yaw 已取负) */
    float output_gyro_Euler  [3][3];  /* 串级 PID 内环输出 = 送混控的三轴力矩需求 */
    float Finally_Angle      [3][4];  /* 最终写定时器的 PWM 比较值 µs(各舵 ZERO + 角度映射) */
    float Stable_Euler_Angle[3];      /* 自稳基准角:自检后锁存,作 Start/Stable/Terminal 的 roll/yaw(及保持时 pitch)目标 */
    int16_t Guidance_cnt[4];          /* 制导状态机各跳变的去抖计数 */
    uint8_t pid_cale_flag;            /* 本拍是否跑了 PID(Vofa 观测) */
    uint8_t Text_Flag;                /* 自检标志(预留) */
}Surface_t;

/* 上电自检流程(视觉 + 镖头触发板)状态 */
typedef struct
{
    uint8_t Vision_Self_Text_flag;            /* 视觉自检:Self_Text_Failure/Success */
    uint8_t Dart_Trigger_Self_Text_flag;      /* 触发板自检:Failure/Success */
    uint16_t Vision_Self_Text_flag_cnt;       /* 视觉自检去抖计数 */
    uint16_t Dart_Trigger_Self_Text_flag_cnt; /* 触发板自检去抖计数 */
    uint8_t Self_Text_Process;                /* 自检流程进度(Self_Text_Vision/Dart_Trigeer/OK/Start) */
}Self_Text_t;

extern float Stable_Euler_Angle[3];
extern Surface_t Surface;
extern Self_Text_t Self_Text;
extern uint8_t Guidance_State;
extern uint8_t Wing_Servo_Control_Flag;
extern float servo_lat_scale;   /* Vofa 可观测:最低优先轴保留比(横侧 k 泛化),1=未饱和,<1=有轴被挤缩 */
extern uint8_t Alloc_Mode;                     /* 控制分配档:0=旧pitch优先对照 1=三轴限幅 2=最小能量 */
extern uint8_t Alloc_Prio[3];                  /* 交付A 逐级优先级:轴枚举[0]最高,默认{PITCH,YAW,ROLL};调试器在线改 */
extern float   Alloc_B[3][4];                  /* 舵效矩阵 τ=B·u,行[p,r,y]列[UL,UR,DR,DL];默认理想阵,可台架辨识替换 */
extern float   alloc_u0[4], alloc_u_out[4];    /* Vofa:分配解(投影/降级后,×SIGN前) / 最终写舵值(×SIGN限幅后) */
extern float   alloc_alpha, alloc_u0_span, alloc_v_scale, alloc_p_scale; /* Vofa:零空间投影α / u0极差 / 降级横侧缩放比 / pitch缩放比 */
extern uint8_t alloc_infeasible, alloc_singular_flag;     /* Vofa:不可达降级(0可达/1缩横侧/2连pitch也缩) / 求逆奇异退回 */
extern float vision_los_final[3];   /* Vofa:末制导锁存的世界系视线终点(目标斜坡逼近它),帧间不变、仅新帧阶跃更新 */
void surface_control_task(void);
void Roll_Derotate_PitchYaw(float Pw, float Yw, float *Pb, float *Yb);
void Servo_Mix_AxisLimit(float p, float r, float y);
void Servo_Mix_MinEnergy(float p, float r, float y);
void Wing_Control(void);
void Wing_Control_VECTOR_NOZZLE(void);


#endif //SURFACE_CONTROL_TSAK_H
