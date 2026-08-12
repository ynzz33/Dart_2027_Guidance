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
#include "../lqi_tool/lqi_torque.h"        /* LQI 力矩控制器(9态→3轴力矩)；未编译需手动加入工程 */
#include "../lqi_tool/torque_allocator.h"  /* 力矩→舵面零空间分配器 */
#include "../lqi_tool/lqi_geometry_table.h"/* H_tau 表 + 零空间 N_ry */
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
int16_t target_Cnt = 0,cnt = 0;
uint16_t current_tick;

float   vision_los_final[2][3],vision_los_current[3];   /* 末制导世界系视线终点:视觉新帧锁存,target 每 tick 斜坡逼近(方案3) */
float   vision_los_rate[3] = {0};                    /* 末制导世界系惯性视线率λ̇(°/s):视觉帧间差分,PN超前用;丢帧保持/丢目标清0 */


/* [死代码留存,2026-08-11] 末端姿态锁定(锁姿态+pitch偏置打实际目标)——定义从未被任何代码使用,
 * 原 .h extern 已注释。功能未实现,保留定义仅供日后实现参考。 */
TerminalLock_t TermLock = {
    .enable        = 1,     /* 默认关,台架验证稳定后再开(Watch置1) */
    .active        = 0,
    .pitch_bias_deg = 5.0f, /* pitch抬高偏置°(+:灯在靶下方,抬高瞄准点),待台架标定 */
    .locked_pitch  = 0.0f,
    .locked_yaw    = 0.0f,
};

/* 全局变量定义 global variable(s) END */
/*---------------------------------------------------------------------------*/
#if 1 
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

        Surface.target_angle_Euler[NOW][YAW] = vision_los_final[NOW][YAW];
        Surface.target_angle_Euler[NOW][PITCH] = vision_los_final[NOW][PITCH];

    /* PNG Mode0 Yaw 超前：在 target 赋值为 vision_los_final 之后叠加超前角。
     * vision_los_rate 已在上面 New_Data_flag 块内由帧间差分更新（或保持上帧值），
     * 每 tick 执行：target 会被 line 250 重置为 vision_los_final → 不累积。
     * 运行时开关：PNG_Mode=0 启用 Mode0，PNG_Yaw_Flag=1 启用 Yaw 轴。
     * 台架 A/B：切 #if 0 即退回纯 PID/LQI 跟踪（不叠加 PNG 超前）。 */
    #if 0
    if (PNG_Mode == 0 && PNG_Yaw_Flag
        && Vision_Rx_Data.Vision_recognize_flag == RECOGNIZE_SUCCESS)
    {
        PNG_Apply_Lead_Yaw(&Surface, &IMU_Data);
    }
    #endif

    /* [历史] 旧 PNG 调用（Mode1 EKF 全量，基于 vins_out.locked）——保留作对照。
     * 注:pitch_control_limit_deg 为已弃用悬空符号(见 surface_control_task.h 清理注释),
     * 若日后重新启用此块需恢复该全局定义。当前 #if 0 不参与编译。 */
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
    Buzzer_stop();
    if (Surface.Guidance_cnt[4]++>2000)
    {
        Vision_Transmit(Vision_Cmd_Record_Stop);
        Guidance_State = PROCESS_OK;
        Surface.Guidance_cnt[4] = 0;
    }
    Wing_Servo_Control_Flag = 0;
    // Total_Power_Control( Power_OFF )//不能太快掉电，不然openmv保存不了视频等等
}
void Guidance_Process_OK(void)
{
    Vision_Transmit(Vision_Cmd_Record_Stop);
    Buzzer_Remind();
    if (Surface.Guidance_cnt[5]++>5000)
    {
		Total_Power_Control(Power_OFF);
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
                    IMU_Data.calib_done = 2;
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
        
        Surface.target_angle_Euler[NOW][ROLL]  =  
        Surface.Stable_Euler_Angle[ROLL];
        /* ROLL 始终自稳(与视觉新数据无关),每 tick 刷新 */
        // Surface.target_angle_Euler[NOW][YAW]  =  
        // Surface.current_angle_Euler[NOW][YAW];
        // Surface.target_angle_Euler[NOW][PITCH]  =  
        // Surface.current_angle_Euler[NOW][PITCH];
        // Surface.target_angle_Euler[NOW][ROLL]  =  
        // 0;

}   
void get_current_State(void)
{
    if(cnt<target_Cnt)
    {
        cnt++;
        Vision_Transmit( Vision_Cmd_Record_Start );
    }
    if(
            Surface.Guidance_flag[1] == 1
    )
    {
        static uint16_t Text_cnt = 0;
        if((Text_cnt++)%1000==1)
        {
          Buzzer_Remind();
        }
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
            // Shot_Pitch = IMU_Data.Euler[NOW][PITCH];
        }
    }
    else if (Guidance_State == Start )
    { 
        if(IMU_Data.Euler[NOW][PITCH]<=Shot_Pitch+5.0f&&IMU_Data.Euler[NOW][PITCH]>=Shot_Pitch- 5.0f&&
           IMU_Data.Euler[NOW][ROLL]<=Shot_Roll+6.0f&&IMU_Data.Euler[NOW][ROLL]>=Shot_Roll- 6.0f)
        {
            Surface.Guidance_cnt[1]++;
            Vision_Transmit( Vision_Cmd_Record_Start );
        }
        if(Surface.Guidance_cnt[1]>=50&&Surface.Guidance_flag[1]==0)
        {
            Buzzer_Remind();
            Surface.Stable_Euler_Angle[ROLL]  = Surface.current_angle_Euler[NOW][ROLL];
            Surface.Stable_Euler_Angle[YAW]   = Surface.current_angle_Euler[NOW][YAW];
            Surface.Stable_Euler_Angle[PITCH] = Surface.current_angle_Euler[NOW][PITCH];
            Surface.Guidance_flag[1] = 1;
        } 
        if(Surface.Guidance_flag[1] == 1&&V_DART_Lqi>=1.5f&&fabs(IMU_Data.Velocity[Body][NOW][Y])>0.3f&&fabs(IMU_Data.Velocity[Body][NOW][X])<0.5f)
        {
            Buzzer_Remind();
            Guidance_State = Stable;
            Surface.Stable_Euler_Angle[YAW]   = Surface.current_angle_Euler[NOW][YAW];
            Surface.Guidance_cnt[1] = 0;
            Surface.Guidance_flag[1] = 2;
        }
    }
    else if (Guidance_State == Stable && (IMU_Data.Euler[NOW][PITCH]<=Shot_Pitch-10.0&&Vision_Rx_Data.Vision_recognize_flag==RECOGNIZE_SUCCESS))
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
    else if (Guidance_State == Terminal &&(V_DART_Lqi<4.0f||IMU_Data.A[NOW][Y]<-0.8f))
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
    // if (((Surface.Guidance_flag[2] == 1&&Guidance_State == Stable)||Guidance_State==Terminal)&&imu_is_static==0)
    if ((Guidance_State == Stable||Guidance_State==Terminal)&&imu_is_static==0)
    {
        Surface.pid_cale_flag = 1;
        if (lqi_mode == 1)
        {
            /* LQI 力矩控制 + Pitch 保护零空间分配（3轴力矩→4舵）——当前唯一激活链路 */
            Euler_LQI_Cale(delta_time);
        }
        else
        {
            /* [弃用留存] LQR 一步 6态→4舵——LQR 已不再使用（2026-08-11），代码留存仅供对照，
             * 正常恒走 LQI 分支（lqi_mode=1）。 */
            Euler_LQR_Cale(delta_time);
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
