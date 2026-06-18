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
#define  Servo_UL_ZERO      1400
#define  Servo_UR_ZERO      1360
#define  Servo_DR_ZERO      1500
#define  Servo_DL_ZERO      1480 
#define Shot_Pitch 35
 
// //镖体2 蓝色
// #define  Servo_UL_ZERO      1650
// #define  Servo_UR_ZERO      1495
// #define  Servo_DR_ZERO      1510
// #define  Servo_DL_ZERO      1490 

//镖体3 红色
// #define  Servo_UL_ZERO      1460
// #define  Servo_UR_ZERO      1560
// #define  Servo_DR_ZERO      1645
// #define  Servo_DL_ZERO      1500
// #define Shot_Pitch 25



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
#define  AXIS_LIMIT_PITCH   30.0f   /* 交付A:三轴各自独立限幅(度),可调 */
#define  AXIS_LIMIT_ROLL    20.0f
#define  AXIS_LIMIT_YAW     30.0f
#define  ALLOC_U_MAX        SERVO_ANGLE_LIMIT   /* 交付B:单舵物理上限 */
#define  ALLOC_GAIN         4.0f   /* 交付B:伪逆解标称增益。理想阵(BBᵀ=4I)下令最小能量解 Bᵀv/4 还 原成与三轴限幅/旧版同幅度(Bᵀv),复用 PID 标定;辨识非理想 B 后可重调 */

/* === 世界系 pitch/yaw 解算(roll 反旋) ===
 * 喂 PID 的 ZYX 欧拉 pitch/yaw 是世界系参考,PID 输出即世界系 pitch/yaw 力矩需求;
 * 但 X 翼舵面产生机体系力矩。机身横滚 Δ 后把该需求反旋到机体系再送混控(见 .c)。
 * SIGN: 横滚正向/舵面朝向的总符号,台架单轴阶跃验证后可翻 ±1。*/
#define  ROLL_WORLD_COMP_SIGN  (+1.0f)

// 这里用不到了，直接用误差/多少去做，也就是将误差分为多少段去逐渐弥补
// /* === 末制导视觉目标斜坡(方案3:setpoint 端速率限制) ===
//  * 视觉~20Hz,锁存目标每 50ms 阶跃刷新一次;直接喂阶跃目标会周期性冲击外环 P(及对误差微分时的 D)。
//  * 在目标端(非反馈环,不引入 P/I 相位滞后)把阶跃摊成斜坡:每 tick 目标朝锁存终点最多走 VISION_TARGET_SLEW_DPS·dT 度。
//  * 调大→更跟手(接近阶跃)、调小→更平滑但跟踪滞后增大;台架按抖动/跟踪权衡。*/
// #define  VISION_TARGET_SLEW_DPS   10000.0f

/* === 末制导俯仰能量管理:随接近度放开的最陡俯冲下限 θ_floor(见 surface_control_task.c Pitch_Dive_Floor) ===
 * 远离目标只许浅俯冲(保能量/射程),接近才放开到期望入射俯冲角;入射角=速度方向(γ)=撞击姿态(正向撞击)。
 * 角度均为机体俯仰°(俯冲为负,更负=更陡);全部待台架实测微调。*/
#define  PITCH_INCIDENT_DEG        (-27.0f)  /* 期望入射俯冲角(=速度方向),待实测 -25~-30 */
#define  PITCH_DIVE_LIMIT_FAR_DEG  (-8.0f)   /* 远处(s=0)允许的最陡俯冲:越浅越保射程 */
#define  AOA_MARGIN_DEG            (10.0f)   /* 机体俯仰最多比速度方向γ再低这么多(防大负迎角掉升力损能) */
#define  GAMMA_FAR_DEG             (-5.0f)   /* 距离/面积都缺失时按γ自调度的起点(≈刚看到引导灯的俯仰) */
/* 接近度 s 分段合成(面积+距离一起用):远段用距离(标定准、连续),近段用面积(blob大、近场更可靠);
 * dist_cm 决定走哪段,两段在切换点 s=DIVE_SCHED_SWITCH 衔接。dist/area 来自视觉 0x5B 包。全部待台架实测。*/
#define  DIST_ACQUIRE_CM         (700.0f)   /* 远段起点:刚识别引导灯(s=0)的目标距离cm */
#define  DIST_NEAR_CM            (450.0f)   /* 远/近段切换距离(s=DIVE_SCHED_SWITCH);≤此距离改用面积调度 */
#define  AREA_NEAR               (800.0f)   /* 近段起点面积(≈DIST_NEAR_CM处的blob像素,s=DIVE_SCHED_SWITCH) */
#define  AREA_IMPACT            (4000.0f)   /* 近段终点:接近撞击(s=1)的blob像素 */
#define  DIVE_SCHED_SWITCH        (0.6f)    /* 远/近段衔接处的接近度s:距离管 0→此值,面积管 此值→1 */

/* === 末制导 YAW 距离面积增益:随接近度调整 yaw 控制增益(补偿视觉距离效应) ===
 * 视觉原理:同样像素误差,远端对应的实际角度误差更大(视场角+距离),近端更小。
 * 因此:远端需要更大增益补偿(更灵敏),近端用正常或略小增益(防过冲)。
 * 使用与 pitch 俯冲放开相同的接近度 s 计算方式(距离/面积分段合成):
 *   远段用距离 dist_cm(标定准、连续,s:0→DIVE_SCHED_SWITCH),
 *   近段用面积 area(blob 大、近场更可靠,s:DIVE_SCHED_SWITCH→1),
 *   dist_cm 决定走哪段;两包都无(dist_cm=0)退化为默认增益1.0。
 * 增益线性插值:YAW_GAIN_FAR(s=0,远处) → YAW_GAIN_NEAR(s=1,近处)。
 * 全部待台架实测微调。*/
#define  YAW_GAIN_FAR            (1.5f)    /* 远处(s=0)的 yaw 增益:补偿视觉距离效应,更灵敏 */
#define  YAW_GAIN_NEAR           (0.8f)    /* 近处(s=1)的 yaw 增益:近距离视觉误差大,适当减小防过冲 */
#define  YAW_GAIN_ENABLE_DIST    (800.0f)  /* 启用 yaw 增益调整的最大距离阈值(cm):超过此距离不调整(无数据) */

/* === 末制导混合导引:视线率PN超前 + 配平迎角前馈(均不依赖会漂的IMU积分速度) ===
 * PN:目标角按"世界系惯性视线率λ̇"超前——λ̇由锁存视线帧间差分得(纯视觉),驱动λ̇→0=碰撞航线,
 *    既提前瞄准命中、又给外环超前相位压制猎振;λ̇限幅防丢帧/视觉跳变爆冲。
 * 迎角前馈:理论应加迎角 θ−γ,但本工程 γ 取姿态前向≡Euler[PITCH]→θ−γ≡0 不含迎角信息,
 *    无气动配平数据,退化为常值 AOA_TRIM_DEG(机体俯仰比视线高这么多,使速度方向落在视线上)。
 * 全部待台架/试飞实测调:先 PN_LEAD_K 小增益验证方向与稳定性、再逐步加大;AOA_TRIM 有数据再给。*/
#define  PN_LEAD_K          (0.05f)    /* 视线率超前系数(s):target += K·λ̇;↑更超前/更抗滞后,过大易被视觉噪声激励 */
#define  LOS_RATE_LIMIT_DPS (30.0f)   /* 视线率λ̇限幅(°/s):丢帧/视觉跳变时防 PN 项爆冲 */
#define  AOA_TRIM_DEG       (0.0f)    /* 常值配平迎角前馈(°,默认0=关):正=机头比视线高,使速度方向对准灯;有试飞数据再标定 */

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
    float output_Body_Euler  [3][3];  
    float Finally_Angle      [3][4];  /* 最终写定时器的 PWM 比较值 µs(各舵 ZERO + 角度映射) */
    float Stable_Euler_Angle[3];      /* 自稳基准角:自检后锁存,作 Start/Stable/Terminal 的 roll/yaw(及保持时 pitch)目标 */
    int16_t Guidance_cnt[6];          /* 制导状态机各跳变的去抖计数 */
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
extern float pitch_control_limit_deg;   /* 放开 pitch 控制的俯冲角阈值°(<此值才主动制导 pitch);PNG_Apply_Lead 引用 */
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
extern float vision_los_final[2][3];   /* Vofa:末制导锁存的世界系视线终点(目标斜坡逼近它),帧间不变、仅新帧阶跃更新 */
extern float vision_los_rate[3];    /* Vofa:世界系惯性视线率λ̇(°/s,PN用),视觉帧间差分、丢帧保持/丢目标清0 */
extern float pitch_dive_floor;      /* Vofa:末制导俯仰俯冲下限θ_floor°(随接近度放开) */
extern float closeness_s;           /* Vofa:接近度s∈[0,1](像素面积调度,缺失时按γ) */
void surface_control_task(void);
void Roll_Derotate_PitchYaw(float Pw, float Yw, float *Pb, float *Yb);
void Servo_Mix_AxisLimit(float p, float r, float y);
void Servo_Mix_MinEnergy(float p, float r, float y);
void Wing_Control(void);
void Wing_Control_VECTOR_NOZZLE(void);


#endif //SURFACE_CONTROL_TSAK_H
