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
#include "adrc.h"           /* LADRC 线性自抗扰控制器(文件名仍 adrc.*) */
#include "../lqr_tool/lqr.h"  /* LQR 姿态控制器(6态→4舵一步解算，含混控)；未编译需手动加入工程 */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "IMU.h"
#include "tim.h"
#include "filter.h"
#include "PNG_Task.h"
#include "vision_ins.h"

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
int8_t Stable_Cnt = 0;
uint16_t current_tick;


/* === 速度矢量追踪制导:三级串级 ===
 * 外环:速度方向追目标方向 → 输出 body 目标角
 * 中环:镖头追 body 目标角 → 输出 rate_cmd (复用角度环 PID)
 * 内环:角速度追 rate_cmd → 输出舵偏 (复用 gyro 环 PID)
 * vel_pursuit_mode=0 时退化为原两级 PID。*/
uint8_t vel_pursuit_mode = 0;
/* LADRC 控制选择：0=全 PID(安全默认), 1=三轴全 LADRC, 3=仅 Roll 用 LADRC(本次先试)
 * LADRC 旋钮在 ladrc_ctrl[轴].{wc,wo,b0}，可用调试器 Watch 在线改(见 adrc.c)。*/
uint8_t ladrc_mode = 0;   /* ★ 先在 roll 上试 LADRC，pitch/yaw 仍走 PID；要回全 PID 就改 0 */

/* LQR 控制选择：0=关(走 PID/LADRC，安全默认), 1=用 LQR 一步解算(6态→4舵，含混控，绕过 PID 与 Servo_Mix_*)
 * K 矩阵在 lqr.c 的 dart_lqr_K[4][6](MATLAB 粘贴区)；符号/换序见 lqr.c。优先级高于 ladrc_mode/vel_pursuit_mode。
 * ★ 未编译/待台架：上车前务必按指南 §9 逐轴阶跃验符号(飞镖一次性，G 符号反=正反馈)。*/
uint8_t lqr_mode = 1;

/* === 控制分配(混控)状态:整合为单一结构体 Alloc(原 Alloc_Mode/Prio/B、alloc_*、servo_lat_scale 散落全局) === */
Alloc_t Alloc = {
    .Mode = 2,                                 /* 默认最小能量;1=三轴限幅 0=旧PitchPriority对照 */
    .Prio = { ROLL, YAW, PITCH },              /* 交付A(Mode1)逐级优先级:[0]最高=roll>yaw>pitch;Mode2 不读、其饱和优先级硬编码 pitch>横侧 */
    /* 舵效矩阵 B (3x4): τ=B·u, 行[pitch,roll,yaw] 列[UL,UR,DR,DL]. 默认理想 X 阵(BBᵀ=4I).
     * 台架辨识替换法:固定其余三舵=0,给第 j 舵单位阶跃 Δu_j,读三轴力矩响应 Δτ_p/r/y,
     * 令 B[0..2][j]=Δτ_{p,r,y}/Δu_j;改这里即可换实测值,无需改算法,奇异自动退回理想阵闭式. */
    .B = {
        { -1.0f, -1.0f, -1.0f, -1.0f },        /* pitch (四片同号,逻辑列=−1, SIGN 翻转后=真俯仰) */
        { +1.0f, -1.0f, -1.0f, +1.0f },        /* roll  */
        { -1.0f, +1.0f, -1.0f, +1.0f },        /* yaw   */
    },
    .v_scale = 1.0f, .p_scale = 1.0f, .lat_scale = 1.0f,   /* 其余(u0/u_out/alpha/u0_span/infeasible/singular_flag)默认 0 */
};


float   vision_los_final[2][3],vision_los_current[3];   /* 末制导世界系视线终点:视觉新帧锁存,target 每 tick 斜坡逼近(方案3) */
float   vision_los_rate[3] = {0};                    /* 末制导世界系惯性视线率λ̇(°/s):视觉帧间差分,PN超前用;丢帧保持/丢目标清0 */
/* ★封存:下列 4 个 Vofa 观测全局随末制导增益调度/俯冲下限函数(本文件 #if 0 块)一并停用,确认无其它消费者 */
// float   pitch_dive_floor = 0.0f, closeness_s = 0.0f; /* Vofa:末制导俯仰俯冲下限θ_floor° / 接近度s∈[0,1] */
// float   yaw_distance_gain = 1.0f;                   /* Vofa:末制导 yaw 距离面积增益 */
// float   pitch_distance_gain = 1.0f;                  /* Vofa:末制导 pitch 距离面积增益(远端小、近端大) */
float pitch_control_limit_deg = -0.0f;//开始放开pitch控制的角度限制
uint8_t pitch_glide_mode = 1;   /* 0=旧 pitch_control_limit 门限;1=主动滑翔→扎(本次新增),可 Watch 在线切回 0 做 A/B */
float   pitch_glide_target = 0.0f, pitch_glide_blend = 0.0f;
/* 全局变量定义 global variable(s) END */
/*---------------------------------------------------------------------------*/
#if 1

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
    Surface.target_angle_Euler[LAST][YAW] = Surface.target_angle_Euler[NOW][YAW];
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

#endif
void Guidance_Start(void)//自检后的判断
{
            Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
            Surface.target_angle_Euler[NOW][YAW]   = Surface.Stable_Euler_Angle[YAW];
        Surface.target_angle_Euler[NOW][ROLL]  =  
        Surface.Stable_Euler_Angle[ROLL];
}
void Guidance_Stable(void)//自稳
{
            // Buzzer_Remind();
        Surface.target_angle_Euler[NOW][PITCH] = 
        Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][YAW]   = 
        Surface.Stable_Euler_Angle[YAW];
        Surface.target_angle_Euler[NOW][ROLL]  =  
        Surface.Stable_Euler_Angle[ROLL];
}
void Guidance_Terminal(void)//制导段
{
    /* PN 视线率状态(函数级静态,帧间保持):上帧世界系视线终点 / 自上帧起累计的控制拍数 / 首帧标志 */
    static uint16_t vis_dt_cnt    = 0;
    static uint8_t  los_rate_init = 0;
    vis_dt_cnt++;
        // Surface.current_angle_Euler[NOW][ROLL]+((Surface.Stable_Euler_Angle[ROLL]-Surface.current_angle_Euler[NOW][ROLL])/2.0f);

    vision_los_current[PITCH] = Surface.current_angle_Euler[NOW][PITCH];
    vision_los_current[YAW]   = Surface.current_angle_Euler[NOW][YAW];
    /* 视线目标锁存(方向A):视觉~20Hz、控制1kHz。只在"视觉新数据到达"(Vision_New_Data_flag==1)那一刻,用当时
     * 姿态把视线锁存到世界系 los=v+current,写入斜坡终点 vision_los_final(不再直接阶跃写 target);帧间(flag==0)
     * 终点保持不变 → 终点−current 仍随机体转动实时变化(航位推算)。新数据处理结束后置回0,等下一帧再置1。*/
    if (Vision_Rx_Data.Vision_New_Data_flag == 1)
    {
        taskENTER_CRITICAL();
        Vision_Rx_Buf_t v = Vision_Rx_Data;
        taskEXIT_CRITICAL() ;
        if (v.Vision_recognize_flag == RECOGNIZE_SUCCESS) /* 识别到目标:锁存世界系视线终点(全程用快照 v,避免与视觉中断撕裂读) */
        {
            /* YAW:始终视觉制导,锁存世界系视线终点 */
            vision_los_final[NOW][YAW]   = v.Euler[NOW][1] + Surface.current_angle_Euler[NOW][YAW];
            vision_los_final[NOW][PITCH] = v.Euler[NOW][0] + Surface.current_angle_Euler[NOW][PITCH];

                
            if (los_rate_init)
            {
                float dt_vis = (float)vis_dt_cnt * (CTRL_PERIOD_MS * 0.001f);
                if (dt_vis < 1e-3f) dt_vis = 1e-3f;
                /* 帧间差分算原始世界系视线率(PN 超前用) */
                vision_los_rate[YAW]   = (vision_los_final[LAST][YAW]   - vision_los_final[NOW][YAW])   / dt_vis;
                vision_los_rate[PITCH] = (vision_los_final[LAST][PITCH] - vision_los_final[NOW][PITCH]) / dt_vis;
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
    if (Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_FAILURE) 
    {
        vision_los_final[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        vision_los_final[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
        vision_los_current[PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        vision_los_current[YAW]   = Surface.current_angle_Euler[NOW][YAW];
        Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        Surface.target_angle_Euler[NOW][YAW]   = Surface.current_angle_Euler[NOW][YAW];
        vision_los_rate[PITCH] = 0.0f;   
        vision_los_rate[YAW]   = 0.0f;
        los_rate_init = 0;
    }

    /* 视觉目标 = 锁存的世界系视线终点(无额外平滑) */
    {
        /* 视觉目标 = 锁存的世界系视线终点 vision_los_final(新帧阶跃更新,帧间靠航位推算)。
         * 原 LPF / 距离增益缩放(Yaw/Pitch_Gain_ByDist)已停用,此处直接取用,不再有中间滤波量。
         * 丢目标(FAILURE)时 vision_los_final[NOW] 已在上方被置为当前姿态 → 自然就地保持。*/

        /* YAW:直接追锁存视线终点 */
        Surface.target_angle_Euler[NOW][YAW] = vision_los_final[NOW][YAW];

        if (Surface.current_angle_Euler[NOW][PITCH] <= pitch_control_limit_deg)
        {
            Surface.target_angle_Euler[NOW][PITCH] = vision_los_final[NOW][PITCH];
        }
        else
        {
            Surface.target_angle_Euler[NOW][PITCH] = Surface.current_angle_Euler[NOW][PITCH];
        }
    }

    /* PN 超前:暂时关闭,先用纯 PID 跟踪视觉目标验证基础跟踪性能。
     * 后续标定好 K_Dyn 后再打开 Mode1(速度²缩放)。*/
    #if 0
    if (vins_out.locked)
    {
        PNG_Apply_Lead_Yaw(&Surface, &IMU_Data);
        if (Surface.current_angle_Euler[NOW][PITCH] <= pitch_control_limit_deg)
            PNG_Apply_Lead_Pitch(&Surface, &IMU_Data);
    }
    #endif

}
void Guidance_End(void)
{
    // Dart_Trigger_Power_Control( Power_OFF );
    if (Surface.Guidance_cnt[4]++>100)
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
    if (Surface.Guidance_cnt[5]++>3000)
    {
		Total_Power_Control(Power_OFF);
        Surface.Guidance_cnt[5] = 0;
    }
}

void get_current_Target(void)
{
        // Guidance_State = Stable;
        Surface.target_angle_Euler[NOW][ROLL]  =  
        Surface.Stable_Euler_Angle[ROLL];
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
                        // Surface.Stable_Euler_Angle[PITCH] = IMU_Data.Euler[NOW][PITCH];
                        // Surface.Stable_Euler_Angle[ROLL]  = IMU_Data.Euler[NOW][ROLL]; 
                        // Surface.Stable_Euler_Angle[YAW]   = IMU_Data.Euler[NOW][YAW]; 
                        // Surface.target_angle_Euler[NOW][ROLL]  = Surface.Stable_Euler_Angle[ROLL];
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
        
        /* ROLL 始终自稳(与视觉新数据无关),每 tick 刷新 */
        // Surface.target_angle_Euler[NOW][YAW]  =  
        // Surface.Stable_Euler_Angle[YAW];
        // Surface.target_angle_Euler[NOW][PITCH]  =  
        // Surface.current_angle_Euler[NOW][PITCH];

}   
void get_current_State(void)
{
    static uint8_t yaw_zero_latched = 0;   /* yaw 归零一次性闩锁:窗口内只发一次请求,流程重置后重新武装 */
    if(
            Surface.Guidance_flag[1] == 1
    )
    {
            Buzzer_Remind();
    }
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
            // Shot_Pitch = IMU_Data.Euler[NOW][PITCH];
        }
    }
    else if (Guidance_State == Start )
    {
        Vision_Transmit( Vision_Cmd_Work );
        Vision_Transmit( Vision_Cmd_Record_Start );

        if(IMU_Data.Euler[NOW][PITCH]<=Shot_Pitch+2.0f&&IMU_Data.Euler[NOW][PITCH]>=Shot_Pitch- 2.0f&&
           IMU_Data.Euler[NOW][ROLL]<=Shot_Roll+3.0f&&IMU_Data.Euler[NOW][ROLL]>=Shot_Roll- 3.0f)
        {
            Surface.Guidance_cnt[1]++;
        }
        if(Surface.Guidance_cnt[1]>=20&&Surface.Guidance_flag[1]==0)
        {
            Surface.Stable_Euler_Angle[ROLL]  = IMU_Data.Euler[NOW][ROLL];
            Surface.Stable_Euler_Angle[YAW]   = IMU_Data.Euler[NOW][YAW];
            Surface.Guidance_flag[1] = 1;
        }
        if(Surface.Guidance_flag[1] == 1&&(IMU_Data.Velocity[Body][NOW][Z]<=-0.7f||(IMU_Data.Velocity[Body][NOW][Y]>=0.7f)))
        {
            Buzzer_Remind();
            Guidance_State = Stable;
            Surface.Guidance_cnt[1] = 0;
            Surface.Guidance_flag[1] = 2;
        }
    }
    else if (Guidance_State == Stable && (IMU_Data.Euler[NOW][PITCH]<=10.0f&&Vision_Rx_Data.Vision_recognize_flag==RECOGNIZE_SUCCESS))
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
    else if (Guidance_State == Terminal &&V_DART<4.0f)
    {
        if(Surface.Guidance_cnt[3]++>50)
        {
            Buzzer_Remind();
            Guidance_State = End;
            Surface.Guidance_cnt[3] = 0;
        }
        
    }
    else 
    {
        Surface.Guidance_cnt[0] = 0;
        Surface.Guidance_cnt[1] = 0;
        Surface.Guidance_cnt[2] = 0;
        Surface.Guidance_cnt[3] = 0;
        yaw_zero_latched = 0;   /* 流程退回:重新武装 yaw 归零,下次进窗口再触发一次 */
    }
    if(Guidance_State>=Stable&&V_DART==2.0f)
    {
        if(Surface.POWER_OFF_CNT++>500)
        {
            Guidance_State = End;
        }
        
    }
    if(Guidance_State == PROCESS_OK)
    {
		Total_Power_Control(Power_OFF);
    }
}

/*---- 线程区 ----*/
void surface_control_task(void) 
{
  /* USER CODE BEGIN surface_control_task */
    static uint32_t prev_tick = 0;
    static uint8_t  first_run = 1;
    current_tick = xTaskGetTickCount();
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
    if ((Guidance_State==Stable||Guidance_State==Terminal)&&(imu_is_static==0))
    {
        Surface.pid_cale_flag = 1;
        Euler_LQR_Cale(delta_time);
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
