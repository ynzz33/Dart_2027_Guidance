/******************************************************************************
*** @File           : surface_control_task.c
*** @Description    : None
*** @Attention      : None
*** @Author         : ynzz33
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
#include "adrc.h"           /* ADRC自抗扰控制器 */
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

float pitch_control_limit_deg = -20.0f;//开始放开pitch控制的角度限制 

/* ADRC控制选择：0=使用原PID, 1=使用ADRC(全部通道), 2=仅Yaw用ADRC(调试用) */
uint8_t adrc_mode = 0;

/* === 控制分配(混控)全局 === */
uint8_t Alloc_Mode = 1;                        /* 默认=交付A三轴限幅(逐级优先级缩放,roll-only 下线性正确);0=旧PitchPriority对照 2=最小能量 */
uint8_t Alloc_Prio[3] = { YAW,  ROLL ,  PITCH };  /* 交付A 逐级优先级:轴枚举[0]最高,默认 pitch>yaw>roll;调试器 Watch 在线改 */
float   servo_lat_scale = 1.0f;                /* Vofa:最低优先轴保留比(横侧 k 泛化),1=未饱和 <1=有轴被挤;此前 extern 无定义,补回 */
/* 舵效矩阵 B (3x4): τ = B·u, 行序[pitch,roll,yaw], 列序[UL,UR,DR,DL]. 默认理想 X 阵(BBᵀ=4I).
 * 台架辨识替换法:固定其余三舵=0,给第 j 舵单位阶跃 Δu_j(如+20°),读三轴力矩响应(可用 output_gyro_Euler
 * 内环输出或 IMU 角加速度作代理)Δτ_p/r/y,令 B[0..2][j]=Δτ_{p,r,y}/Δu_j;四舵各做一次填满12个元素.
 * 单位任意一致(算法只用方向与相对幅值);改这里即可换实测值,无需改算法,奇异自动退回理想阵闭式. */
float   Alloc_B[3][4] = {
    { -1.0f, -1.0f, -1.0f, -1.0f },            /* pitch (四片同号,X 翼解算见 Servo_Mix_AxisLimit 的 C 阵) */
    { +1.0f, -1.0f, -1.0f, +1.0f },            /* roll  */
    { -1.0f, +1.0f, -1.0f, +1.0f },            /* yaw   */
};
float   alloc_u0[4] = {0}, alloc_u_out[4] = {0};
float   alloc_alpha = 0.0f, alloc_u0_span = 0.0f, alloc_v_scale = 1.0f, alloc_p_scale = 1.0f;
uint8_t alloc_infeasible = 0, alloc_singular_flag = 0;


float   vision_los_final[2][3],vision_los_current[3];   /* 末制导世界系视线终点:视觉新帧锁存,target 每 tick 斜坡逼近(方案3) */
float   vision_los_rate[3] = {0};                    /* 末制导世界系惯性视线率λ̇(°/s):视觉帧间差分,PN超前用;丢帧保持/丢目标清0 */
float   pitch_dive_floor = 0.0f, closeness_s = 0.0f; /* Vofa:末制导俯仰俯冲下限θ_floor° / 接近度s∈[0,1] */
float   yaw_distance_gain = 1.0f;                   /* Vofa:末制导 yaw 距离面积增益 */
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

void Wing_Control(void)
{
    if (Wing_Servo_Control_Flag == 1)
    {      
        if(Guidance_State == Self_Text_State)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 0;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 0;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 0;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 0; 
        }
        else if(Guidance_State == Start&&IMU_Data.calib_done==1)
        {
            Surface.output_angle_Servo[NOW][UP_LEFT]    = 0;
            Surface.output_angle_Servo[NOW][UP_RIGHT]   = 0;
            Surface.output_angle_Servo[NOW][DOWN_LEFT]  = 0;
            Surface.output_angle_Servo[NOW][DOWN_RIGHT] = 0; 
        }
        Wing_Control_VECTOR_NOZZLE();
    }
}

void Guidance_Start(void)//自检后的判断
{
            // Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
            // Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
            // Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
}
void Guidance_Stable(void)//自稳
{
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
        // Buzzer_play_song(song_ni);
}
/* 目标斜坡逼近(速率限制):把 cur 朝 target 每拍最多移动 max_step,把视觉阶跃摊成斜坡(方案3)。
 * wrap=1:差值按角度环绕到最短弧(YAW 是 atan2 周期角);wrap=0:普通(PITCH 是 asin 不环绕)。*/
static float Target_Slew(float cur, float target, float max_step, uint8_t wrap)
{
    float err = target - cur;
    if (wrap) err = Angle_Wrap_180(err);
    if (err >  max_step) err =  max_step;
    if (err < -max_step) err = -max_step;
    return cur + err;
}

/* === 末制导俯仰能量管理:随接近度放开的最陡俯冲下限 θ_floor ===
 * 钳"机体俯仰目标 ≥ θ_floor"=不许比 θ_floor 更陡。两条"≥"下限取较不陡(较大)者:
 *  ① 调度限幅 L_sched:接近度 s∈[0,1] 从远(s=0,浅滑翔保射程)线性放开到近(s=1,期望入射角)。
 *     s 分段合成(面积+距离一起用,数据来自视觉 0x5B 包):远段用距离 dist_cm(标定准、连续,s:0→DIVE_SCHED_SWITCH),
 *     近段用面积 area(blob 大、近场更可靠,s:DIVE_SCHED_SWITCH→1),dist_cm 决定走哪段;两包都无(dist_cm=0)
 *     退化按弹道角 γ 自调度(γ 随重力变陡→逐步放开)。
 *  ② 迎角限幅:θ ≥ γ−AOA_MARGIN,即鼻子不许指到比速度方向(γ)再低 AOA_MARGIN 以上——大负迎角
 *     →掉升力→因重力掉高度损能,正是"过早陡俯冲损耗能量"的物理根因;此条防距离/面积被骗时仍守住。
 * 终端 γ≈θ≈入射角时迎角 θ−γ→0=正向撞击。Vofa 观测 closeness_s / pitch_dive_floor。*/
static float Pitch_Dive_Floor(uint16_t dist_cm, uint16_t area)
{
    float s_dist, s_area, s;
    if (dist_cm == 0)                              /* 无 0x5B 距离/面积包 → 退化按弹道角 γ 自调度 */
        // s = (gamma_pitch_deg - GAMMA_FAR_DEG) / (PITCH_INCIDENT_DEG - GAMMA_FAR_DEG);
        s = 0.0f;   /* 直接退化到最保守的远段限幅,不放开 γ 自调度了(实测 γ 不够准),待实测再改回按 γ 调度 */
    else if ((float)dist_cm > DIST_NEAR_CM)        /* 远段:距离调度,s 由远(s=0)线性升到切换点 */
        s = DIVE_SCHED_SWITCH * (DIST_ACQUIRE_CM - (float)dist_cm) / (DIST_ACQUIRE_CM - DIST_NEAR_CM);
    else                                           /* 近段:面积调度,s 由切换点升到近(s=1) */
        s = DIVE_SCHED_SWITCH + (1.0f - DIVE_SCHED_SWITCH) * ((float)area - AREA_NEAR) / (AREA_IMPACT - AREA_NEAR);
    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    closeness_s = s;

    float L_sched = PITCH_DIVE_LIMIT_FAR_DEG + s * (PITCH_INCIDENT_DEG - PITCH_DIVE_LIMIT_FAR_DEG);
    float L_aoa   = gamma_pitch_fwd_deg - AOA_MARGIN_DEG;
    float dive_floor = (L_sched > L_aoa) ? L_sched : L_aoa;   /* 两个"≥"下限取较不陡(较大)者 */
    pitch_dive_floor = dive_floor;
    return dive_floor;
}

/* === 末制导 YAW 距离面积增益:随接近度调整 yaw 控制增益 ===
 * 参考 Pitch_Dive_Floor 的接近度 s 计算方式(距离/面积分段合成):
 *   远段用距离 dist_cm(标定准、连续,s:0→DIVE_SCHED_SWITCH),
 *   近段用面积 area(blob 大、近场更可靠,s:DIVE_SCHED_SWITCH→1),
 *   dist_cm 决定走哪段;两包都无(dist_cm=0)退化为默认增益1.0。
 *
 * 增益线性插值:YAW_GAIN_FAR(s=0) → YAW_GAIN_NEAR(s=1)。
 * 超过 YAW_GAIN_ENABLE_DIST 距离时不调整(返回1.0),避免远处误触发。
 * Vofa 观测 yaw_distance_gain。*/
static float Yaw_Distance_Area_Gain(uint16_t dist_cm, uint16_t area)
{
    /* 超过启用距离阈值，不调整增益 */
    if ((float)dist_cm > YAW_GAIN_ENABLE_DIST)
    {
        yaw_distance_gain = 1.0f;
        return 1.0f;
    }

    float s;
    float gain;
    if (dist_cm == 0)                              /* 无 0x5B 距离/面积包 → 退化为默认增益 */
        // s = 0.0f;
        gain = 1.0;
    else if ((float)dist_cm > DIST_NEAR_CM)        /* 远段:距离调度,s 由远(s=0)线性升到切换点 */
        gain = 1.3;
        // s = DIVE_SCHED_SWITCH * (DIST_ACQUIRE_CM - (float)dist_cm) / (DIST_ACQUIRE_CM - DIST_NEAR_CM);
    else         
        gain = 0.85;                                  /* 近段:面积调度,s 由切换点升到近(s=1) */
        // s = DIVE_SCHED_SWITCH + (1.0f - DIVE_SCHED_SWITCH) * ((float)area - AREA_NEAR) / (AREA_IMPACT - AREA_NEAR);

    // if (s < 0.0f) s = 0.0f;
    // if (s > 1.0f) s = 1.0f;

    /* 增益线性插值:YAW_GAIN_FAR(s=0) → YAW_GAIN_NEAR(s=1) */
    // float gain = YAW_GAIN_FAR + s * (YAW_GAIN_NEAR - YAW_GAIN_FAR);

    yaw_distance_gain = gain;
    return gain;
}

void Guidance_Terminal(void)//制导段
{
    /* PN 视线率状态(函数级静态,帧间保持):上帧世界系视线终点 / 自上帧起累计的控制拍数 / 首帧标志 */
    static uint16_t vis_dt_cnt    = 0;
    static uint8_t  los_rate_init = 0;
    vis_dt_cnt++;
    /* ROLL 始终自稳(与视觉新数据无关),每 tick 刷新 */
    Surface.target_angle_Euler[NOW][ROLL] = Surface.Stable_Euler_Angle[ROLL];
    vision_los_current[PITCH] = Surface.current_angle_Euler[NOW][PITCH];
    vision_los_current[YAW]   = Surface.current_angle_Euler[NOW][YAW];
    /* 视线目标锁存(方向A):视觉~20Hz、控制1kHz。只在"视觉新数据到达"(Vision_New_Data_flag==1)那一刻,用当时
     * 姿态把视线锁存到世界系 los=v+current,写入斜坡终点 vision_los_final(不再直接阶跃写 target);帧间(flag==0)
     * 终点保持不变 → 终点−current 仍随机体转动实时变化(航位推算)。新数据处理结束后置回0,等下一帧再置1。*/
    if (Vision_Rx_Data.Vision_New_Data_flag == 1)
    {
        taskENTER_CRITICAL();
        Vision_Rx_Buf_t v = Vision_Rx_Data;
        taskEXIT_CRITICAL();
        if (v.Vision_recognize_flag == RECOGNIZE_SUCCESS) /* 识别到目标:锁存世界系视线终点(全程用快照 v,避免与视觉中断撕裂读) */
        {
            /* YAW:始终视觉制导,锁存世界系视线终点 */
            vision_los_final[NOW][YAW] = v.Euler[NOW][1] + Surface.current_angle_Euler[NOW][YAW];

            /* PITCH:仅俯冲到位(<-10°)才视觉制导;否则不控,终点=当前 */
            if (Surface.current_angle_Euler[NOW][PITCH] < pitch_control_limit_deg)
                vision_los_final[NOW][PITCH] = v.Euler[NOW][0] + Surface.current_angle_Euler[NOW][PITCH];
            else
                vision_los_final[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];

            /* 惯性视线率 λ̇(PN用):本帧世界系视线终点 − 上帧,÷帧间实际时长(自上帧累计控制拍×周期)。世界系锁存
             * 已含弹体转动补偿,纯视觉、不依赖会漂的IMU速度。首帧无前值则λ̇=0;丢帧时Δt随拍数增大→λ̇自然减小不爆冲;
             * 再 LOS_RATE_LIMIT 限幅兜底。YAW 周期角先环绕到最短弧。λ̇ 帧间保持,供下方每 tick 取用。*/
            if (los_rate_init)
            {
                float dt_vis = (float)vis_dt_cnt * (CTRL_PERIOD_MS * 0.001f);
                if (dt_vis < 1e-3f) dt_vis = 1e-3f;
                vision_los_rate[YAW]   =                (vision_los_final[LAST][YAW]   - vision_los_final[NOW][YAW])/9.5f   / dt_vis;
                vision_los_rate[PITCH] =                (vision_los_final[LAST][PITCH] - vision_los_final[NOW][PITCH])/9.5f / dt_vis;
                abs_limit(&vision_los_rate[YAW],   LOS_RATE_LIMIT_DPS);
                abs_limit(&vision_los_rate[PITCH], LOS_RATE_LIMIT_DPS);
            }
            vision_los_final[LAST][YAW]   = vision_los_final[NOW][YAW];
            vision_los_final[LAST][PITCH] = vision_los_final[NOW][PITCH];
            los_rate_init   = 1;
            vis_dt_cnt      = 0;
        }
        Vision_Rx_Data.Vision_New_Data_flag = 0;   /* 新数据处理完毕,置回0 */
    }
    if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE) /* 丢目标:就地保持,终点与目标都对齐当前(斜坡 d=0,即原零误差语义) */
    {
        // Vision_Transmit(Vision_Cmd_Work);
        vision_los_final[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        vision_los_final[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
        vision_los_current[PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        vision_los_current[YAW]   = Surface.current_angle_Euler[NOW][YAW];
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
        vision_los_rate[PITCH] = 0.0f;   /* 丢目标:清视线率、复位首帧标志,重捕后不吃陈旧λ̇ */
        vision_los_rate[YAW]   = 0.0f;
        los_rate_init = 0;
    }

        /* YAW 距离面积增益:根据接近度调整 yaw 控制响应
         * 远距离/小面积时增益小(减少抖动),近距离/大面积时增益大(提高跟踪精度)
         * 对 yaw 目标误差应用增益:target_adj = current + gain * (target - current) */
        float yaw_gain = Yaw_Distance_Area_Gain(Vision_Rx_Data.dist_cm, Vision_Rx_Data.area);
        /* 俯仰俯冲限幅(能量管理):仅识别成功(主动视觉制导)时钳俯仰目标 ≥ θ_floor——远处禁陡俯冲保射程*/
        float dive_floor = Pitch_Dive_Floor(Vision_Rx_Data.dist_cm, Vision_Rx_Data.area);
    /* 方案3:setpoint 端速率限制——每 tick 目标朝锁存终点斜坡逼近,把视觉 50ms 阶跃摊平,
     * 消除目标台阶对外环 P(及对误差微分时的 D)的周期性冲击;非反馈环低通,不引入 P/I 相位滞后。
     * 帧间终点不变、斜坡到达后 target≡终点,与原"锁存+航位推算"等价,仅消去切换瞬间的阶跃。*/
    // 先将目标置为当前值，然后将新的目标值存在vision_los_final中以斜坡方式逼近最终目标值，
    Surface.target_angle_Euler[NOW][YAW] =
        Target_Slew(vision_los_current[YAW], vision_los_final[NOW][YAW], yaw_gain*fabs(vision_los_final[NOW][YAW]-vision_los_current[YAW])/20, 0);
    Surface.target_angle_Euler[NOW][PITCH] =
        Target_Slew(vision_los_current[PITCH], vision_los_final[NOW][PITCH], fabs(vision_los_final[NOW][PITCH]-vision_los_current[PITCH])/20, 0);

    /* 混合导引超前:在斜坡目标之上叠加 PN 超前(必须在 Target_Slew 之后,否则被覆盖) */
    if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_SUCCESS)
        PNG_Apply_Lead(&Surface, &IMU_Data);

        if (Surface.target_angle_Euler[NOW][PITCH] < dive_floor)
            Surface.target_angle_Euler[NOW][PITCH] = dive_floor;
}
void Guidance_End(void)
{
    // Dart_Trigger_Power_Control( Power_OFF );
    if (Surface.Guidance_cnt[4]++>500)
    {
        Vision_Transmit(Vision_Cmd_Record_Stop);
        Guidance_State = PROCESS_OK;
        Buzzer_stop();
        Surface.Guidance_cnt[4] = 0;
    }
    Wing_Servo_Control_Flag = 0;
    // Total_Power_Control( Power_OFF )//不能太快掉电，不然openmv保存不了视频等等
}
void Guidance_Process_OK(void)
{
    if (Surface.Guidance_cnt[5]++>500)
    {
		Total_Power_Control(Power_OFF);
        Surface.Guidance_cnt[5] = 0;
    }
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
                if(IMU_Data.calib_done==1)
                {
                    static uint16_t cnt = 0 ;
                    if(cnt++>1)
                    {
                        IMU_Data.calib_done = 2;
                        Surface.Stable_Euler_Angle[PITCH] = IMU_Data.Euler[NOW][PITCH];
                        Surface.Stable_Euler_Angle[ROLL]  = IMU_Data.Euler[NOW][ROLL]; 
                        Surface.Stable_Euler_Angle[YAW]   = IMU_Data.Euler[NOW][YAW]; 
                        Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
                        // Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
                    }
                }   
                Stable_Flag = 1;
                Guidance_Stable();
            }break;
            case Terminal:
            {
                Guidance_Terminal();
            }break;
            case End:
            {
                Guidance_End();
            }break;
            case PROCESS_OK:
            {
                Guidance_Process_OK();
            }break;
        }
}
void get_current_State(void)
{ 
    if (Self_Text.Self_Text_Process==Self_Text_OK&&Guidance_State == Self_Text_State)
    {
        if (Surface.Guidance_cnt[0]++>2000)
        {
            Buzzer_Remind();
            Guidance_State = Start;
            Vision_Transmit( Vision_Cmd_Work );
            if (Self_Text.Self_Text_Process<5)
            {
                Self_Text.Self_Text_Process = 5; 
            } 
            Surface.Guidance_cnt[0] = 0;
            Surface.Stable_Euler_Angle[PITCH] = IMU_Data.Euler[NOW][PITCH];
        }
    }
    else if (Guidance_State == Start && (IMU_Data.Euler[NOW][PITCH]<=Shot_Pitch+8.0f&&IMU_Data.Euler[NOW][PITCH]>=Shot_Pitch- 8.0f)&&((IMU_Data.A_Normed[NOW][Y] >= 0.80f&&IMU_Data.A[NOW][Y] >= 1.0f)||(IMU_Data.Euler[NOW][PITCH]<=(Surface.Stable_Euler_Angle[PITCH]-3)&&IMU_Data.Euler[NOW][PITCH]>=(Surface.Stable_Euler_Angle[PITCH]-10))))
    {
        Vision_Transmit( Vision_Cmd_Work );
        if (Surface.Guidance_cnt[1]++>5)
        {
            Buzzer_Remind();
            Vision_Transmit( Vision_Cmd_Record_Start );
            Guidance_State = Stable;
            Surface.Guidance_cnt[1] = 0;
        }
    }
    else if (Guidance_State == Stable && (IMU_Data.Euler[NOW][PITCH]<=10.0f||Vision_Rx_Data.x[NOW]!=0.0f))
    {
        Vision_Transmit( Vision_Cmd_Work );
        if(Surface.Guidance_cnt[2]++>5)
        {
            Buzzer_Remind();
            Guidance_State = Terminal;
            Surface.Guidance_cnt[2] = 0;
            Vel_Reanchor_Flag = 1;   /* 俯冲入段:请求 IMU 下一拍用"姿态前向×V_NOM"锚定世界速度 → γ起始≈机体俯仰 */
        }
    }
    else if (Guidance_State == Terminal &&fabs(IMU_Data.A_Normed[NOW][Y])>= 0.80f&& IMU_Data.A[NOW][Y]<=  -1.5f)
    {
        Vision_Transmit( Vision_Cmd_Work ); 
        if(Surface.Guidance_cnt[3]++>5)
        {
            Buzzer_Remind();
            Guidance_State = End;
            Vision_Transmit( Vision_Cmd_Record_Stop );
            Surface.Guidance_cnt[3] = 0;
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

#if 1
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

    /* 逻辑符号阵 C(未含物理 SIGN),列序 P/R/Y。pitch 列四片同号 [+1,+1,+1,+1] 是 X 翼解算所需:
     * 4 片成 X(45°,从尾看 UL135/UR45/DR315/DL225),每片切向力对力矩贡献 pitch∝cosθ、yaw∝sinθ、roll=常数;
     * 配 SIGN=[−1,+1,+1,−1](左右镜像,见.h)后 pitch 落到舵令 u=[−,+,+,−]=真俯仰、与 roll/yaw 正交解耦。
     * 轴间配对结构由本阵的列决定(SIGN 只修整片装反);同步:Alloc_B pitch 行 / Servo_Mix_PitchPriority 的 P[]。*/
    static const float C[4][3] = {                  /* 列序 P/R/Y */
        { -1.0f, +1.0f, -1.0f },                    /* UP_LEFT    */
        { -1.0f, -1.0f, +1.0f },                    /* UP_RIGHT   */
        { -1.0f, -1.0f, -1.0f },                    /* DOWN_RIGHT */
        { -1.0f, +1.0f, +1.0f },                    /* DOWN_LEFT  */
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
/* 1 维零空间 n:n_j=(-1)^j·det(B 删去第 j 列的 3x3 子阵)。理想阵应得 n=[4,4,-4,-4]∝[1,1,-1,-1]。*/
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
    /*pid/adrc，算输出值*/
    Surface.pid_cale_flag = 0;
    if (Guidance_State==Stable||Guidance_State==Terminal)
    {
        Surface.pid_cale_flag = 1;

        /* ========== ADRC/PID 控制选择 ========== */
        // if (adrc_mode == 1)
        // {
        //     /* 全通道ADRC */
        //     // Euler_ADRC_Cale(delta_time);
        // }
        // else if (adrc_mode == 2)
        // {
        //     /* 仅Yaw用ADRC，其他用PID（调试用） */
        //     // Euler_pid_Cale(delta_time);  /* 先算PID */
        //     // /* 覆盖Yaw通道为ADRC输出 */ 
        //     // float yaw_gyro_cmd = ADRC_Calc(
        //     //     &adrc_ctrl[ADRC_YAW][ADRC_ANGLE_LOOP],
        //     //     Surface.target_angle_Euler[NOW][YAW],
        //     //     Surface.current_angle_Euler[NOW][YAW],
        //     //     delta_time);
        //     // Surface.output_gyro_Euler[NOW][YAW] = ADRC_Calc(
        //     //     &adrc_ctrl[ADRC_YAW][ADRC_GYRO_LOOP],
        //     //     yaw_gyro_cmd,
        //     //     Surface.current_gyro_Euler[NOW][YAW],
        //     //     delta_time);
        // }
        // else
        // {
            /* 原PID（默认） */
            Euler_pid_Cale(delta_time);
        // }
        /* ===================================== */
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
        // if (Guidance_State==Terminal&&IMU_Data.Euler[NOW][PITCH]>0.0F)
        // {
        //     // y_body *= 1.5f;
        //     p_body = 0.0f;  
        // }
            if (Guidance_State==Terminal)
            {
            // Roll_Derotate_PitchYaw(Surface.output_gyro_Euler[NOW][PITCH],
            //                        Surface.output_gyro_Euler[NOW][YAW],
            //                        &p_body, &y_body);
            }

            // p_body = 0; 
            // y_body = 0; 
            // r_body = 0;
            /* 三轴统一接 PID 内环输出(roll 绕纵轴不反旋,直接用);按 Alloc_Mode 分派分配器。
             * 注:上面 p_body*0.85/y_body*1.05 是旧分配的经验微调,Mode2 精确分配下会破坏最优,
             * B 台架辨识后应把这两个系数设 1,由 B 承担全部相对标定。*/
            Surface.output_Body_Euler[NOW][PITCH] = p_body;
            Surface.output_Body_Euler[NOW][YAW]   = y_body; 
            Surface.output_Body_Euler[NOW][ROLL]  = r_body;
            switch (Alloc_Mode)
            {
                // case 0:  Servo_Mix_PitchPriority(p_body, r_body, y_body); break;  /* 旧对照(roll 入参已改 PID 输出) */
                case 1:  Servo_Mix_AxisLimit    (p_body, r_body, y_body); break;
                case 2:  Servo_Mix_MinEnergy    (p_body, r_body, y_body); break;
            }
        }
        for (int i = 0; i < 4; i++)                    /* 安全网:分配已保证在限内,此处仅兜底 FP 误差 */
            abs_limit(&Surface.output_angle_Servo[NOW][i], SERVO_ANGLE_LIMIT);
    }
    if (Guidance_State == End || Guidance_State == PROCESS_OK)
    {
        for (int i = 0; i < 4; i++) Surface.output_angle_Servo[NOW][i] = 0;
    }
    Wing_Control();
    Data_Updata();
    if (Self_Text.Self_Text_Process == Self_Text_Dart_Trigeer&&Self_Text.Dart_Trigger_Self_Text_flag == Self_Text_Failure && Self_Text.Vision_Self_Text_flag == Self_Text_Success)
    {
        osDelay( 300 );
    }
  /* USER CODE END surface_control_task */
}
