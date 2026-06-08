/******************************************************************************
*** @File           : surface_control_task.c
*** @Description    : None
*** @Attention      : None
*** @Author         : Lmy
*** @Date           : 2025/3/2
*** @版权归属:
***                  ██╗███╗   ███╗ ██████╗ █████╗
***                  ██║████╗ ████║██╔════╝██╔══██╗
***                  ██║██╔████╔██║██║     ███████║
***                  ██║██║╚██╔╝██║██║     ██╔══██║
***                  ██║██║ ╚═╝ ██║╚██████╗██║  ██║
***                  ╚═╝╚═╝     ╚═╝ ╚═════╝╚═╝  ╚═╝
******************************************************************************/
/* 头文件 include(s) BEGIN */
#include "surface_control_task.h"

#include "buzzer.h"
#include "CallBack_Task.h"
#include "pid.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "IMU.h"
#include "tim.h"
#include "filter.h"
#include "PNG_Task.h"

/* 头文件 include(s) END */
/*---------------------------------------------------------------------------*/
/* 静态变量定义 static variable(s) BEGIN */

/* 静态变量定义 static variable(s) END */
/*---------------------------------------------------------------------------*/
/* 全局变量定义 global variable(s) BEGIN */
uint8_t DART_TYPE = VECTOR_NOZZLE;
uint8_t Guidance_State;
Surface_t Surface;
Self_Text_t Self_Text;
uint8_t Wing_Servo_Control_Flag = 1,Stable_Flag = 0;//舵机控制标志位
/* === 控制分配(混控)全局 === */
uint8_t Alloc_Mode = 1;                        /* 默认=交付A三轴限幅(逐级优先级缩放,roll-only 下线性正确);0=旧PitchPriority对照 2=最小能量 */
uint8_t Alloc_Prio[3] = { PITCH, YAW, ROLL };  /* 交付A 逐级优先级:轴枚举[0]最高,默认 pitch>yaw>roll;调试器 Watch 在线改 */
float   servo_lat_scale = 1.0f;                /* Vofa:最低优先轴保留比(横侧 k 泛化),1=未饱和 <1=有轴被挤;此前 extern 无定义,补回 */
/* 舵效矩阵 B (3x4): τ = B·u, 行序[pitch,roll,yaw], 列序[UL,UR,DR,DL]. 默认理想 X 阵(BBᵀ=4I).
 * 台架辨识替换法:固定其余三舵=0,给第 j 舵单位阶跃 Δu_j(如+20°),读三轴力矩响应(可用 output_gyro_Euler
 * 内环输出或 IMU 角加速度作代理)Δτ_p/r/y,令 B[0..2][j]=Δτ_{p,r,y}/Δu_j;四舵各做一次填满12个元素.
 * 单位任意一致(算法只用方向与相对幅值);改这里即可换实测值,无需改算法,奇异自动退回理想阵闭式. */
float   Alloc_B[3][4] = {
    { +1.0f, +1.0f, +1.0f, +1.0f },            /* pitch */
    { +1.0f, -1.0f, -1.0f, +1.0f },            /* roll  */
    { -1.0f, +1.0f, -1.0f, +1.0f },            /* yaw   */
};
float   alloc_u0[4] = {0}, alloc_u_out[4] = {0};
float   alloc_alpha = 0.0f, alloc_u0_span = 0.0f, alloc_v_scale = 1.0f, alloc_p_scale = 1.0f;
uint8_t alloc_infeasible = 0, alloc_singular_flag = 0;
/* 全局变量定义 global variable(s) END */
/*---------------------------------------------------------------------------*/


void Euler_Updata(void)
{
    //Updata
    for (int i = 0;i<3;i++)
    {
        Surface.target_angle_Euler [LLAST][i] = Surface.target_angle_Euler [LAST][i];
        Surface.target_angle_Euler [LAST ][i] = Surface.target_angle_Euler [NOW ][i];
        Surface.current_angle_Euler[LLAST][i] = Surface.current_angle_Euler[LAST][i];
        Surface.current_angle_Euler[LAST ][i] = Surface.current_angle_Euler[NOW ][i];
    }
}
void Servo_Updata(void)
{
    /* X 翼跑 4 个舵机,十字翼跑 3 个;output_angle_Servo 共用 [3][4] 缓存 */
    int n = (DART_TYPE == VECTOR_NOZZLE) ? 4 : 3;
    for (int i = 0; i < n; i++)
    {
        Surface.output_angle_Servo[LLAST][i] = Surface.output_angle_Servo[LAST][i];
        Surface.output_angle_Servo[LAST ][i] = Surface.output_angle_Servo[NOW ][i];
    }
}                    
void Data_Updata(void)
{
    Euler_Updata();
    Servo_Updata();
}

void Guidance_Start(void)//自检后的判断
{
    static uint16_t cnt = 0 ;
    if(IMU_Data.calib_done==1)
    {
      if(cnt++>=1000)
      {
          cnt = 0 ;
          IMU_Data.calib_done = 2;
          Surface.Stable_Euler_Angle[PITCH] = IMU_Data.Euler[NOW][PITCH];
          Surface.Stable_Euler_Angle[ROLL]  = IMU_Data.Euler[NOW][ROLL];
          Surface.Stable_Euler_Angle[YAW]   = IMU_Data.Euler[NOW][YAW]; 
      }
    }
            Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
            Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
            Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
}
void Guidance_Stable(void)//自稳
{
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
        // Buzzer_play_song(song_ni);
        if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE)
        {
            Vision_Transmit(Vision_Cmd_Work);
        }
}
void Guidance_Terminal(void)//制导段
{
    /* ROLL 始终自稳(与视觉新数据无关),每 tick 刷新 */
    Surface.target_angle_Euler[NOW][ROLL] = Surface.Stable_Euler_Angle[ROLL];

    /* 视线目标锁存(方向A):视觉~20Hz、控制1kHz。只在"视觉新数据到达"(Vision_New_Data_flag==1)那一刻,用当时
     * 姿态把视线锁存到世界系 los_world=v+current 并写入目标;帧间(flag==0)不进此块,目标靠 target_angle_Euler[NOW]
     * 保持上次锁存值不变(Euler_Updata 只移位 LAST/LLAST、不动 NOW) → 外环误差=los_world−current,机体一转误差即减,
     * 无每 tick 重算导致的盲转/符号翻转。新数据处理结束后置回0,等下一帧视觉数据中断里再置1。*/
    if (Vision_Rx_Data.Vision_New_Data_flag == 1)
    {
        taskENTER_CRITICAL();
        Vision_Rx_Buf_t v = Vision_Rx_Data;
        taskEXIT_CRITICAL();
        if (v.Vision_recognize_flag == RECOGNIZE_SUCCESS) /* RECOGNIZE_SUCCESS:识别到目标,锁存世界系视线目标,更新目标角 */
        {    /* 全程用临界区快照 v,避免与 UART 视觉中断(~20Hz 写多字段)撕裂读;清 flag 仍打真 struct */
            /* YAW:始终视觉制导,用当时姿态锁存世界系视线 → 误差=锁存−当前 */
            Surface.target_angle_Euler[NOW][YAW] = v.Euler[NOW][YAW] + Surface.current_angle_Euler[NOW][YAW];

            /* PITCH:仅俯冲到位(<-10°)才视觉制导(同样锁存);否则不控,目标=当前 */
            if (Surface.current_angle_Euler[NOW][PITCH] < -10.0f)
            {
                Surface.target_angle_Euler[NOW][PITCH] = v.Euler[NOW][PITCH] + Surface.current_angle_Euler[NOW][PITCH];
            }
            else
            {
                Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
            }
        }
        Vision_Rx_Data.Vision_New_Data_flag = 0;   /* 新数据处理完毕,置回0,等下一帧视觉新数据再置1 */
    }
    if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE) /* RECOGNIZE_FAILURE:丢目标,保持当前姿态,请求视觉继续工作 */
    {
        Vision_Transmit(Vision_Cmd_Work);
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
    }
        Surface.target_angle_Euler[NOW][PITCH] = Surface.Stable_Euler_Angle[PITCH];
        Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
}
void Guidance_End(void)
{
    // Dart_Trigger_Power_Control( Power_OFF );
    if (Surface.Guidance_cnt[3]++>200)
    {
        Vision_Transmit(Vision_Cmd_Record_Stop);
        Guidance_State = PROCESS_OK;
    }
    Wing_Servo_Control_Flag = 0;
    // Total_Power_Control( Power_OFF )//不能太快掉电，不然openmv保存不了视频等等
    Buzzer_stop();
}


void get_current_Target(void)
{
        // Guidance_State = Stable;
        Stable_Flag = 0;
        switch(Guidance_State)
        {
            case Start:
            {
                Guidance_Start();
            }break;
            case Stable:
            {
                if(IMU_Data.Euler[NOW][PITCH]<=30.0f)
                {
                    Stable_Flag = 1;
                    Guidance_Stable();
                }
            }break;
            case Terminal:
            {
                Guidance_Terminal();
            }break;
            case End:
            {
                Guidance_End();
            }break;
        }
}
void get_current_State(void)
{
    if (Self_Text.Self_Text_Process==Self_Text_OK)
    {
        Guidance_State = Start;
        if (Self_Text.Self_Text_Process<5)
        {
            Self_Text.Self_Text_Process = 5; 
        }
    }
    else if (Guidance_State == Start && (IMU_Data.A_Normed[NOW][Y] >= 0.80f||IMU_Data.A[NOW][Y] <= -1.0f))
    {
        if (Surface.Guidance_cnt[0]++>5)
        {
            Vision_Transmit( Vision_Cmd_Record_Start );
            Guidance_State = Stable;
            Surface.Guidance_cnt[0] = 0;
        }
    }
    else if (Guidance_State == Stable && IMU_Data.Euler[NOW][PITCH]<=0.0f)
    {
        if(Surface.Guidance_cnt[1]++>5)
        {
            Guidance_State = Terminal;
            Surface.Guidance_cnt[1] = 0;
        }
    }
    else if (Guidance_State == Terminal &&IMU_Data.A_Normed[NOW][Y] >= 0.90f&& IMU_Data.A[NOW][Y]>= 1.50f)
    {
        if(Surface.Guidance_cnt[2]++>5)
        {
            Guidance_State = End;
            Vision_Transmit( Vision_Cmd_Record_Stop );
            Surface.Guidance_cnt[2] = 0;
        }
    }
}
#if 1 //DART_TYPE == VECTOR_NOZZLE
/* X 翼 4 个舵机 PWM 写入:接 TIM3 CH2 TIM4 CH2-CH4 (PB6/PB7/PB8/PB9)
 * 输入 data 单位是度(±90 内), 内部映射到 PWM 微秒并做 ZERO 偏置 + 限幅
 */
void Wing_UL_Control(float data)//硬件原因导致左上舵机接在了 TIM3 上，所以单独写函数控制
{
    Surface.Finally_Angle[NOW][UP_LEFT]  = Servo_UL_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Surface.Finally_Angle[NOW][UP_LEFT]);
    
    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_UL_ZERO);
}
void Wing_UR_Control(float data)
{ 
    Surface.Finally_Angle[NOW][UP_RIGHT] = Servo_UR_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_UR_Channel, Surface.Finally_Angle[NOW][UP_RIGHT]);
    
    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_UR_ZERO);
}
void Wing_DL_Control(float data)
{
    Surface.Finally_Angle[NOW][DOWN_LEFT]  = Servo_DL_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DL_Channel, Surface.Finally_Angle[NOW][DOWN_LEFT]);

    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_DR_ZERO);
}
void Wing_DR_Control(float data)
{
    Surface.Finally_Angle[NOW][DOWN_RIGHT] = Servo_DR_ZERO + (data / 90.0f * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim4, Servo_DR_Channel, Surface.Finally_Angle[NOW][DOWN_RIGHT]);

    // __HAL_TIM_SET_COMPARE(&htim3, Servo_UL_Channel, Servo_DL_ZERO);
}
/* === 世界系 pitch/yaw 解算:roll 反旋(输出端) ===
 * 喂 PID 的 ZYX 欧拉 pitch/yaw 是世界系参考(绕纵轴横滚不改 pitch 读数,已台架确认),
 * 故 PID 输出 = 世界系 pitch/yaw 力矩需求;但 X 翼舵面产生的是机体系力矩。
 * 机身相对参考姿态横滚 Δ=当前roll−Stable_roll 后,把 (Pw,Yw) 反旋 Δ 投到机体系:
 *     Pb =  cosΔ·Pw + sinΔ·Yw
 *     Yb = −sinΔ·Pw + cosΔ·Yw
 * Δ=0(标称 roll)→恒等。roll 通道绕纵轴,不在此处理。
 * Roll_World_Comp_Flag=0 时直通(=旧行为,便于 A/B);
 * ROLL_WORLD_COMP_SIGN 为横滚正向/舵面朝向符号,台架单轴阶跃验证后可翻 ±1。*/
void Roll_Derotate_PitchYaw(float Pw, float Yw, float *Pb, float *Yb)
{
    float delta_roll = Surface.current_angle_Euler[NOW][ROLL] - Surface.Stable_Euler_Angle[ROLL];
    float r = DEG2RAD(ROLL_WORLD_COMP_SIGN * delta_roll);
    float c = cosf(r), s = sinf(r);
    *Pb =  c * Pw + s * Yw;
    *Yb = -s * Pw + c * Yw;
}

/* Pitch 优先最小能量控制分配:
 * pitch 全额保留,roll/yaw 等比缩到舵机余量内 → 调 roll/yaw 不污染 pitch。
 * 写入 Surface.output_angle_Servo[NOW][0..3](已含 SIGN,已在 ±SERVO_ANGLE_LIMIT 内)。*/
void Servo_Mix_PitchPriority(float p, float r, float y)
{
    float pitch_limit = SERVO_ANGLE_LIMIT-45;                   //pitch优先，但也有限幅
    abs_limit(&p, pitch_limit);                           /* 1) pitch 分量优先*/

    float P[4], L[4];                                           /* 2) 拆 pitch 分量 / 横侧分量(逻辑符号阵) */
    P[UP_LEFT]    = +p;  L[UP_LEFT]    = +r - y;
    P[UP_RIGHT]   = +p;  L[UP_RIGHT]   = -r + y;
    P[DOWN_RIGHT] = +p;  L[DOWN_RIGHT] = -r - y;
    P[DOWN_LEFT]  = +p;  L[DOWN_LEFT]  = +r + y;

    float k = 1.0f;                                             /* 3) 求最大横侧比例 k */
    for (int i = 0; i < 4; i++)
    {
        float aL = (L[i] < 0.0f) ? -L[i] : L[i];                // 绝对值
        if (aL < 1e-6f) continue;                               /* 该片无横侧分量,不构成约束 */
        float sgnL = (L[i] < 0.0f) ? -1.0f : 1.0f;              //拿到原来的符号以便复原
        float ki = (SERVO_ANGLE_LIMIT - sgnL * P[i]) / aL;      //(限幅−pitch分量)/横侧分量 = 该片可容许的最大横侧比例(已修正:原为反写的 aL/(LIMIT−P))
        if (ki < k) k = ki;                                     //取最严约束:在比例内按原样,超出按比例缩放,保证在限幅内
    }

    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;                                     /* 上钳:未饱和时横侧全额保留、绝不放大(原缺此钳致 |r|>limit 时输出超限) */

    float SGN[4];                                               /* 4) 最终的合成 + 物理方向 SIGN 写入 */
    SGN[UP_LEFT]   = SIGN_UL;  SGN[UP_RIGHT]   = SIGN_UR;
    SGN[DOWN_LEFT] = SIGN_DL;  SGN[DOWN_RIGHT] = SIGN_DR;
    for (int i = 0; i < 4; i++)
        Surface.output_angle_Servo[NOW][i] = SGN[i] * (P[i] + k * L[i]);
}

/* ===== 控制分配(混控)重写:可调三轴限幅(交付A) + 最小能量分配(交付B) =====
 * 三者输出约定一致:写 Surface.output_angle_Servo[NOW][0..3](度,含SIGN,±SERVO_ANGLE_LIMIT 内),
 * 下标 0/1/2/3 = UP_LEFT/UP_RIGHT/DOWN_RIGHT/DOWN_LEFT。由调用点按 Alloc_Mode 分派。*/

/* 交付A(重写):完全可配置的逐级(字典序)优先级缩放分配。
 * Alloc_Prio[0..2] 指定三轴优先级(轴枚举,[0]最高)。最高优先轴在前置限幅内全额保障;
 * 次级/三级各在每片剩余舵机余量内最大化保留,绝不回削高优先轴 → 高优先轴不被低优先轴污染。
 * 写 Surface.output_angle_Servo[NOW][0..3](度,含SIGN,±SERVO_ANGLE_LIMIT 内) 与 alloc_u_out[]。*/
void Servo_Mix_AxisLimit(float p, float r, float y)
{
    /* 1) 前置轴级限幅(各轴独立),保证 |d[a]| ≤ AXIS_LIMIT(应≤SERVO_ANGLE_LIMIT) */
    abs_limit(&p, AXIS_LIMIT_PITCH);
    abs_limit(&r, AXIS_LIMIT_ROLL);
    abs_limit(&y, AXIS_LIMIT_YAW);
    float d[3] = { p, r, y };                       /* 按 PITCH/ROLL/YAW 下标 */

    static const float C[4][3] = {                  /* 逻辑符号阵(未含物理 SIGN),列序 P/R/Y */
        { +1.0f, +1.0f, -1.0f },                    /* UP_LEFT    */
        { +1.0f, -1.0f, +1.0f },                    /* UP_RIGHT   */
        { +1.0f, -1.0f, -1.0f },                    /* DOWN_RIGHT */
        { +1.0f, +1.0f, +1.0f },                    /* DOWN_LEFT  */
    };

    /* 2) Alloc_Prio 合法性校验:必须是 {0,1,2} 的排列,否则退回默认 {PITCH,YAW,ROLL} */
    uint8_t prio[3];
    {
        uint8_t seen = 0, ok = 1;
        for (int rank = 0; rank < 3; rank++)
        {
            uint8_t a = Alloc_Prio[rank];
            if (a > 2 || (seen & (1u << a))) { ok = 0; break; }
            seen |= (1u << a);
        }
        if (ok) { prio[0]=Alloc_Prio[0]; prio[1]=Alloc_Prio[1]; prio[2]=Alloc_Prio[2]; }
        else    { prio[0]=PITCH;         prio[1]=YAW;           prio[2]=ROLL;          }
    }

    /* 3) 逐级缩放:base[i]=各已定轴在片 i 的累计逻辑贡献 */
    float base[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float k_last  = 1.0f;
    for (int rank = 0; rank < 3; rank++)
    {
        int   a = prio[rank];
        float k = 1.0f;
        for (int i = 0; i < 4; i++)
        {
            float term = C[i][a] * d[a];
            float at   = (term < 0.0f) ? -term : term;
            if (at < 1e-6f) continue;               /* 无贡献 → 不构成约束 */
            float sgn  = (term < 0.0f) ? -1.0f : 1.0f;
            float ki   = (SERVO_ANGLE_LIMIT - sgn * base[i]) / at;
            if (ki < k) k = ki;
        }
        if (k < 0.0f) k = 0.0f;
        if (k > 1.0f) k = 1.0f;
        for (int i = 0; i < 4; i++)
            base[i] += C[i][a] * k * d[a];          /* 叠加本级,永不回削前级 */
        k_last = k;
    }
    servo_lat_scale = k_last;                       /* Vofa:最低优先轴保留比(横侧 k 的泛化) */

    /* 4) ×物理装配 SIGN + 每片兜底限幅(消 FP 误差) → 写出 */
    float SGN[4];
    SGN[UP_LEFT]    = SIGN_UL;  SGN[UP_RIGHT]   = SIGN_UR;
    SGN[DOWN_RIGHT] = SIGN_DR;  SGN[DOWN_LEFT]  = SIGN_DL;
    for (int i = 0; i < 4; i++)
    {
        float v = SGN[i] * base[i];
        abs_limit(&v, SERVO_ANGLE_LIMIT);
        Surface.output_angle_Servo[NOW][i] = v;
        alloc_u_out[i] = v;
    }
}
#if 1
/* 1 维零空间 n:n_j=(-1)^j·det(B 删去第 j 列的 3x3 子阵)。理想阵应得 n=[-4,-4,-4,-4]∝[1,1,1,1]。*/
static void alloc_compute_nullspace(const float B[3][4], float n[4])
{
    for (int j = 0; j < 4; j++)
    {
        int c[3], k = 0;
        for (int cc = 0; cc < 4; cc++) if (cc != j) c[k++] = cc;
        float a = B[0][c[0]], b = B[0][c[1]], d = B[0][c[2]];
        float e = B[1][c[0]], f = B[1][c[1]], g = B[1][c[2]];
        float h = B[2][c[0]], m = B[2][c[1]], q = B[2][c[2]];
        float det = a * (f * q - g * m) - b * (e * q - g * h) + d * (e * m - f * h);
        n[j] = ((j & 1) ? -1.0f : 1.0f) * det;
    }
}

/* 最小能量伪逆解 u0 = Bᵀ(BBᵀ)⁻¹ v;成功返回1,(BBᵀ)奇异返回0。CMSIS,数据缓冲静态。*/
static uint8_t alloc_minnorm_solve(float B[3][4], float v[3], float u0[4])
{
    static float Bt_d[12], M_d[9], Minv_d[9], t3_d[3];
    mat Bm, Btm, Mm, Minvm, vm, t3m, u0m;
    mat_init(&Bm,    3, 4, (float *)B);
    mat_init(&Btm,   4, 3, Bt_d);
    mat_trans(&Bm, &Btm);                      /* Bᵀ (4x3) */
    mat_init(&Mm,    3, 3, M_d);
    mat_mult(&Bm, &Btm, &Mm);                  /* M = B·Bᵀ (3x3) */
    mat_init(&Minvm, 3, 3, Minv_d);
    if (mat_inv(&Mm, &Minvm) == ARM_MATH_SINGULAR) return 0;
    mat_init(&vm,    3, 1, v);
    mat_init(&t3m,   3, 1, t3_d);
    mat_mult(&Minvm, &vm, &t3m);               /* t3 = M⁻¹·v */
    mat_init(&u0m,   4, 1, u0);
    mat_mult(&Btm, &t3m, &u0m);                /* u0 = Bᵀ·t3 (4x1) */
    return 1;
}

/* 沿零空间 n 把 u0 投影进约束盒 |u_i|≤u_max(Bn=0 故不改变 Bu=v)。
 * 可行区间非空→就地更新 u0、记录 α、返回1;为空(v 不可达)→返回0。*/
static uint8_t alloc_project_nullspace(float u0[4], const float n[4], float u_max)
{
    float a_lo = -1e30f, a_hi = 1e30f;
    for (int i = 0; i < 4; i++)
    {
        if (fabsf(n[i]) < 1e-6f)
        {
            if (fabsf(u0[i]) > u_max) return 0;        /* 该舵无零空间自由度且已越界 → 不可达 */
            continue;
        }
        float lo = (-u_max - u0[i]) / n[i];
        float hi = ( u_max - u0[i]) / n[i];
        if (lo > hi) { float t = lo; lo = hi; hi = t; }
        if (lo > a_lo) a_lo = lo;
        if (hi < a_hi) a_hi = hi;
    }
    if (a_lo > a_hi) return 0;                          /* 可行区间空 → 不可达 */

    float ndotn = n[0]*n[0] + n[1]*n[1] + n[2]*n[2] + n[3]*n[3];
    float u0dn  = u0[0]*n[0] + u0[1]*n[1] + u0[2]*n[2] + u0[3]*n[3];
    float a = (ndotn > 1e-9f) ? -(u0dn / ndotn) : 0.0f; /* 最小能量 α* = -(u0·n)/(n·n) */
    if (a < a_lo) a = a_lo;
    if (a > a_hi) a = a_hi;
    alloc_alpha = a;
    for (int i = 0; i < 4; i++) u0[i] += a * n[i];
    return 1;
}

/* 用 v=[p,r,y] 解 minnorm→×ALLOC_GAIN→零空间投影。可达返回1(u0填最终解),(BBᵀ)奇异返回-1,不可达返回0。*/
static int alloc_try(float p, float r, float y, const float n[4], float u0[4])
{
    float v[3] = { p, r, y };
    if (!alloc_minnorm_solve(Alloc_B, v, u0)) return -1;
    for (int i = 0; i < 4; i++) u0[i] *= ALLOC_GAIN;
    return alloc_project_nullspace(u0, n, ALLOC_U_MAX) ? 1 : 0;
}

/* 交付B:真正的最小能量控制分配。min‖u‖² s.t. B·u=v(×ALLOC_GAIN),|u_i|≤u_max(=ALLOC_U_MAX)。
 * 可达→精确实现该力矩的最小能量解;不可达→优先级 pitch>yaw>roll:先保 pitch 二分缩横侧,横侧缩尽仍
 * 不可达(pitch 力矩超单舵极限)再二分缩 pitch(等效 Mode1 饱和截断);(BBᵀ)奇异→退回三轴限幅。
 * B 默认理想阵,可台架辨识替换(见 Alloc_B 注释)。*/
void Servo_Mix_MinEnergy(float p, float r, float y)
{
    float n[4];
    alloc_compute_nullspace(Alloc_B, n);

    float u0[4] = {0};
    float s_lat = 1.0f, s_pit = 1.0f;
    int rc;

    /* 先判 pitch 单独(横侧全关)可达性:决定走"保 pitch 缩横侧"还是"缩 pitch" */
    rc = alloc_try(p, 0.0f, 0.0f, n, u0);
    if (rc < 0) { alloc_singular_flag = 1; Servo_Mix_AxisLimit(p, r, y); return; }

    if (rc == 1)
    {
        /* 阶段1:pitch 可达 → 保 pitch,二分缩横侧(yaw/roll) */
        for (int it = 0; it < 10; it++)
        {
            rc = alloc_try(p, r * s_lat, y * s_lat, n, u0);
            if (rc != 0) break;                /* 可达(1)或奇异(-1)都跳出 */
            s_lat *= 0.7f;
        }
        if (rc == 0) { rc = alloc_try(p, 0.0f, 0.0f, n, u0); s_lat = 0.0f; }  /* 兜底:退化到纯 pitch(已知可达) */
    }
    else
    {
        /* 阶段2:pitch 力矩超单舵极限 → 横侧置 0,二分缩 pitch(牺牲幅度,等效 Mode1 饱和) */
        s_lat = 0.0f;
        for (int it = 0; it < 12; it++)
        {
            s_pit *= 0.7f;
            rc = alloc_try(p * s_pit, 0.0f, 0.0f, n, u0);
            if (rc != 0) break;
        }
    }
    if (rc < 0) { alloc_singular_flag = 1; Servo_Mix_AxisLimit(p, r, y); return; }
    alloc_singular_flag = 0;
    alloc_v_scale = s_lat;
    alloc_p_scale = s_pit;
    alloc_infeasible = (s_pit < 0.999f) ? 2 : ((s_lat < 0.999f) ? 1 : 0);  /* 0可达 1缩横侧 2连pitch也缩 */

    /* 输出 ×SIGN + 兜底限幅,记录 Vofa */
    float SGN[4];
    SGN[UP_LEFT]    = SIGN_UL;  SGN[UP_RIGHT]   = SIGN_UR;
    SGN[DOWN_RIGHT] = SIGN_DR;  SGN[DOWN_LEFT]  = SIGN_DL;
    float lo = 1e30f, hi = -1e30f;
    for (int i = 0; i < 4; i++)
    {
        alloc_u0[i] = u0[i];
        if (u0[i] < lo) lo = u0[i];
        if (u0[i] > hi) hi = u0[i];
        float out = SGN[i] * u0[i];
        abs_limit(&out, SERVO_ANGLE_LIMIT);
        Surface.output_angle_Servo[NOW][i] = out;
        alloc_u_out[i] = out;
    }
    alloc_u0_span = hi - lo;
}
#endif
void Wing_Control_VECTOR_NOZZLE(void)
{
    if (Guidance_State == Terminal||Guidance_State == Self_Text_State||Stable_Flag ==1 )
    // if (Guidance_State == Self_Text_State)
    {
        Wing_UL_Control(Surface.output_angle_Servo[NOW][UP_LEFT]    );
        Wing_UR_Control(Surface.output_angle_Servo[NOW][UP_RIGHT]   );
        Wing_DL_Control(Surface.output_angle_Servo[NOW][DOWN_LEFT]  );
        Wing_DR_Control(Surface.output_angle_Servo[NOW][DOWN_RIGHT] );
    }
    else
    {
        // 末端失去目标或制导结束,舵面回中
        Wing_UL_Control(0.0f);
        Wing_UR_Control(0.0f);
        Wing_DL_Control(0.0f);
        Wing_DR_Control(0.0f);
        return;
    }
        // Wing_UL_Control(0.0f);
        // Wing_UR_Control(0.0f);
        // Wing_DL_Control(0.0f);
        // Wing_DR_Control(0.0f);
}
#endif
/*---- 线程区 ----*/
void surface_control_task(void)
{
  /* USER CODE BEGIN surface_control_task */
    static uint32_t prev_tick = 0;
    static uint8_t  first_run = 1;
    uint32_t current_tick = xTaskGetTickCount();
    float delta_time;
    if (first_run)
    {
        delta_time = (float)CTRL_PERIOD_MS * 0.001f;
        first_run = 0;
    }
    else
    {
        delta_time = (float)(current_tick - prev_tick) * 0.001f;
        if (delta_time < 1e-4f) delta_time = (float)CTRL_PERIOD_MS * 0.001f;
    }
    prev_tick = current_tick;
    /*状态机，拿目标值*/
    get_current_State();
    get_current_Target();
    /*pid，算输出值*/
    Surface.pid_cale_flag = 0;
    if (Guidance_State==Stable||Guidance_State==Terminal)
    {
        Surface.pid_cale_flag = 1;
        Euler_pid_Cale(delta_time);//这部分先根据当前欧拉角计算差值，坐标系为东北天
    }
    /*解算到舵面*/
    if(DART_TYPE == VECTOR_NOZZLE   )//x翼
    {
        if (Guidance_State==Stable||Guidance_State==Terminal)
        // if (Guidance_State==Terminal)
        {
            /* 先把世界系 pitch/yaw 输出按当前 roll 反旋到机体系,再做 Pitch 优先最小能量分配:
             * pitch 全保,roll/yaw 等比缩进限幅,不污染 pitch。SIGN_xx 仍在函数内逐片乘上,标定流程不变。 */
            float 
            p_body = Surface.output_gyro_Euler[NOW][PITCH], 
            y_body = Surface.output_gyro_Euler[NOW][YAW],
            r_body = Surface.output_gyro_Euler[NOW][ROLL];

            Roll_Derotate_PitchYaw(Surface.output_gyro_Euler[NOW][PITCH],
                                   Surface.output_gyro_Euler[NOW][YAW],
                                   &p_body, &y_body);

            // p_body = 0.0f; 
            // y_body = 0.0f;
            // r_body = 0.0f;

            // if(Guidance_State==Terminal||IMU_Data.Euler[NOW][PITCH]<-15.0f)
            // {
            //     p_body*=0.85f;
            //     y_body*=1.05f;
            // }
            // if(Guidance_State==Stable)
            // {
            //     p_body = 0;
            // }
            // if(Guidance_State==Terminal) 
            // {
            //     r_body = 0.0f;
            // }

            /* 三轴统一接 PID 内环输出(roll 绕纵轴不反旋,直接用);按 Alloc_Mode 分派分配器。
             * 注:上面 p_body*0.85/y_body*1.05 是旧分配的经验微调,Mode2 精确分配下会破坏最优,
             * B 台架辨识后应把这两个系数设 1,由 B 承担全部相对标定。*/
            switch (Alloc_Mode)
            {
                case 0:  Servo_Mix_PitchPriority(p_body, r_body, y_body); break;  /* 旧对照(roll 入参已改 PID 输出) */
                case 1:  Servo_Mix_AxisLimit    (p_body, r_body, y_body); break;
                case 2:  Servo_Mix_MinEnergy    (p_body, r_body, y_body); break;
            }
        }
        for (int i = 0; i < 4; i++)                    /* 安全网:分配已保证在限内,此处仅兜底 FP 误差 */
            abs_limit(&Surface.output_angle_Servo[NOW][i], SERVO_ANGLE_LIMIT);
    }
    if(DART_TYPE == FIXED_WING    )//飞翼
    {
        /*东北天坐标系前*/
        /*这部分，伯努利原理去想，流速快气压小，其他被舵面撞击所以流速减小，也就是舵面往哪里转，就会有一个反方向的力*/
        /*pitch的话，因为上面pid计算出来的是差值，也就是会向目标方向转，所以直接进行*/
        if (Guidance_State==Stable||Guidance_State==Terminal)
        {
                    Surface.output_angle_Servo[NOW][Vertical_fin] =  +Surface.output_gyro_Euler[NOW][YAW];
            // if (Surface.current_angle_Euler[NOW][PITCH]<=-5)
            //     {
                    Surface.output_angle_Servo[NOW][Wing_left]    =(-Surface.output_gyro_Euler[NOW][ROLL]*0.6f + Surface.output_gyro_Euler[NOW][PITCH]*0.0f);
                    Surface.output_angle_Servo[NOW][Wing_right]   =(+Surface.output_gyro_Euler[NOW][ROLL]*0.6f + Surface.output_gyro_Euler[NOW][PITCH]*0.0f);
                // }
        }
        Low_Pass_Filter(Surface.output_angle_Servo[NOW][Wing_left]    ,Surface.output_angle_Servo[LAST][Wing_left]    ,0.7);
        Low_Pass_Filter(Surface.output_angle_Servo[NOW][Wing_right]   ,Surface.output_angle_Servo[LAST][Wing_right]   ,0.7);
        Low_Pass_Filter(Surface.output_angle_Servo[NOW][Vertical_fin] ,Surface.output_angle_Servo[LAST][Vertical_fin] ,0.7);


        abs_limit(&Surface.output_angle_Servo[NOW][Wing_left]    ,60);
        abs_limit(&Surface.output_angle_Servo[NOW][Wing_right]   ,60);
        abs_limit(&Surface.output_angle_Servo[NOW][Vertical_fin] ,60);
        if (Guidance_State == Start)
        {
            Surface.output_angle_Servo[NOW][Wing_left]    = Surface.target_angle_Euler[NOW][PITCH];
            Surface.output_angle_Servo[NOW][Wing_right]   = Surface.target_angle_Euler[NOW][ROLL];
            Surface.output_angle_Servo[NOW][Vertical_fin] = Surface.target_angle_Euler[NOW][YAW];
        }
    }
    if (Guidance_State == End || Guidance_State == PROCESS_OK)
    {
        for (int i = 0; i < 4; i++) Surface.output_angle_Servo[NOW][i] = 0;
    }
    if (Wing_Servo_Control_Flag == 1)
    {      
        if(Guidance_State == Self_Text_State)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 30;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 30;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 30;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 30;
        }
        else if(Guidance_State == Start)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 0;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 0;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 0;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 0; 
        }
        if(DART_TYPE == VECTOR_NOZZLE   )//x翼
        {
            Wing_Control_VECTOR_NOZZLE();
        }
        else if(DART_TYPE == FIXED_WING    )//飞翼
        {
            // Wing_Control_FIXED_WING();
        }
    }
    Data_Updata();
    if (Self_Text.Self_Text_Process == Self_Text_Dart_Trigeer&&Self_Text.Dart_Trigger_Self_Text_flag == Self_Text_Failure && Self_Text.Vision_Self_Text_flag == Self_Text_Success)
    {
        osDelay( 300 );
    }
  /* USER CODE END surface_control_task */
}

//@Author:Liko                     .~---------~.
//                                /'  | === \  '\
//                                |'  | |_/ /  '|
//                                |'  | |\ /   '|
//                                |'__\_| \_\__'|
//                                |>----(+)----<|
//                             _~/|'###########'|\~_
//                          _~/   |'###########'|   \~_
//                       _~/      |'###########'|      \~_
//                    _~/         |'###########'|         \~_
//                 _~/            |'###########'|            \~_
//              _~/               |'###########'|               \~_
//           _~/     '          ' |'###########'| '          '     \~_
//        _~/           '     '   |'###########'|   '     '           \~_
//     _~/                 '      |'###########'|      '                 \~_
//   _/    '            '     '   ||___-|_|-___||   '     '            '    \_
//  //         '     '          ' ||  /_| |_\  || '          '     '         \\
// /'             '               || // | | \\ ||               '             '\
// |'          '     '            ||//  | |  \\||            '     '          '|
// |'       '           '         |' /~-| |-~\ '|         '           '       '|
// |'    '                 '     /''   |===|   ''\     '                 '    '|
// |'  '                      ' / ''           '' \ '                       ' '|
// |'   ____________________-__/ /''           ''\ \__-____________________   '|
// \'   |          ||_+_____| /  |==||==| |==||==|  \ |_____+_||          |   '/
// ^\.._|          '~\V/~~~/  |-----'_ _ _ _ _'-----|  \~~~\V/~'          |_../^
//      ^~--..__          /       |____|===|____|       \          __..--~^
//              ^~--...__/^                             ^\__...--~^

/*---- 函数区 ----*/

