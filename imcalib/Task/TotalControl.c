#include "main.h"
#include "Vofa_send.h"
#include "IMU.h"
#include "TotalControl.h"
#include "ADC_Battery.h"
#include "Button.h"
#include "buzzer.h"
#include "CallBack_Task.h"
#include "surface_control_task.h"
// #include "user_lib.h"
#include "cmsis_os.h"
#include "PNG_Task.h"
#include "usart.h"

// void ALL_reset(uint8_t reset)
// {
// 		HAL_Delay(100);
// 		__set_FAULTMASK(1);
// 		HAL_NVIC_SystemReset();
// }

void TotalControl(void)
{
	if (huart2.gState == HAL_UART_STATE_READY)
	{
	Vofa();
	}
	surface_control_task();
	ADC_Voltage_Real = ADC_GET_REAL_VALUE();
	// PNG_Guidance(&Vision_Rx_Data,&PNG_Data,&Surface,&IMU_Data);

}

void Vofa(void)
{
	Vofa_Send_Data24(
		//====================Dart_Trigger============
		// Dart_Trigger_Data.Communicate_Flag,
		// Dart_Trigger_Data.Frame_Head,
		// Dart_Trigger_Data.Frame_Cmd,
		// Dart_Trigger_Data.Tx_Set_Team_Color,
		// Dart_Trigger_Data.Rx_Set_Team_Color,
		// Dart_Trigger_Data.Borad_Version,
		// Dart_Trigger_Data.Borad_Temp,
		// Dart_Trigger_Data.Borad_State_Team,
		// Dart_Trigger_Data.Borad_State_Light_ON,
		// Dart_Trigger_Data.Borad_State_Voltage,
		// Dart_Trigger_Data.Borad_State_Light_Error,
		// Dart_Trigger_Data.Frame_Tail,
		// 0,0,0,0
		//=====================Buzzer======================
		// Button_Data.TIM_INIT_FLAG,
		// Button_Data.Start_Time,
		// Button_Data.Short_Press_Start_Time,
		// Button_Data.Current_Time,
		// Button_Data.Continuous_Time,
		// Button_Data.Continuous_FLag,
		// Button_Data.Debounce_Time,
		// Button_Data.Press_Type,
		// Button_Data.Press_Long_Cnt[LAST],
		// Button_Data.Press_Long_Cnt[NOW],
		// Button_Data.Press_Short_Cnt,
		// Button_Data.Short_Press_Temp,
		// Buzzer_message.cmd,
		// Buzzer_message.song.notes[current_count].frequency,
		// Buzzer_message.song.tempo,
		// Buzzer_message.song.note_count
		//============IMU================
	// IMU_Data.A[NOW][X],
	// IMU_Data.A[NOW][Y],
	// IMU_Data.A[NOW][Z],
	// IMU_Data.G[NOW][PITCH],
	// IMU_Data.G[NOW][ROLL],
	// IMU_Data.G[NOW][YAW],
	// IMU_Data.M[NOW][X],							//当前欧拉角
	// IMU_Data.M[NOW][Y],
	// IMU_Data.M[NOW][Z],
	// ADC_Voltage_Real,
	// Vision_Rx_Data.Vision_Head,
	// Vision_Rx_Data.Vision_Tail,
	// Vision_Rx_Data.Vision_Self_Text_Data,
	// Vision_Rx_Data.Vision_recognize_flag,
	// Dart_Trigger_Data.Borad_State_Team,
	// Dart_Trigger_Data.Borad_State_Light_ON
	// 	//============IMU================
	// IMU_Data.A_Normed[NOW][X],
	// IMU_Data.A_Normed[NOW][Y],
	// IMU_Data.A_Normed[NOW][Z],
	// IMU_Data.G[NOW][PITCH],
	// IMU_Data.G[NOW][ROLL],
	// IMU_Data.G[NOW][YAW],
	// IMU_Data.A_theory[NOW][X],
	// IMU_Data.A_theory[NOW][Y],
	// IMU_Data.A_theory[NOW][Z],
	// IMU_Data.Euler[NOW][PITCH],							//当前欧拉角
	// IMU_Data.Euler[NOW][ROLL ],
	// IMU_Data.Euler[NOW][YAW  ],
	// IMU_Data.Q[NOW][0],
	// IMU_Data.Q[NOW][1],
	// IMU_Data.Q[NOW][2],
	// IMU_Data.Q[NOW][3]000

		//============逻辑================
	Surface.target_angle_Euler[NOW][PITCH],
	Surface.target_angle_Euler[NOW][ROLL ],
	Surface.target_angle_Euler[NOW][YAW  ],
	Surface.current_angle_Euler[NOW][PITCH],
	Surface.current_angle_Euler[NOW][ROLL ],
	Surface.current_angle_Euler[NOW][YAW  ],
	Vision_Rx_Data.x[NOW],
	Vision_Rx_Data.y[NOW],
	Surface.Finally_Angle[NOW][UP_LEFT   ],
	Surface.Finally_Angle[NOW][UP_RIGHT  ],
	Surface.Finally_Angle[NOW][DOWN_RIGHT],
	Surface.Finally_Angle[NOW][DOWN_LEFT   ],
	IMU_Data.G[NOW][PITCH],
	IMU_Data.G[NOW][ROLL ],
	IMU_Data.G[NOW][YAW  ],
	IMU_Data.A[NOW][X],
	IMU_Data.A[NOW][Y],
	IMU_Data.A[NOW][Z],
	IMU_Data.A_Normed[NOW][X],
	IMU_Data.A_Normed[NOW][Y],
	IMU_Data.A_Normed[NOW][Z],
	Self_Text.Self_Text_Process,
	ADC_Voltage_Real,
	Guidance_State
	//============逻辑================
		// Surface.current_angle_Euler[NOW][PITCH],
		// Surface.current_angle_Euler[NOW][ROLL ],			//转换后的欧拉角，即当前欧拉角
		// Surface.current_angle_Euler[NOW][YAW  ],
		// Surface.target_angle_Euler[NOW][PITCH],
		// Surface.target_angle_Euler[NOW][ROLL ],
		// Surface.target_angle_Euler[NOW][YAW  ],
		// IMU_Data.G[NOW][PITCH],
		// IMU_Data.G[NOW][ROLL ],
		// IMU_Data.G[NOW][YAW  ],
		// Surface.output_gyro_Euler[NOW][PITCH],
		// Surface.output_gyro_Euler[NOW][ROLL ],
		// Surface.output_gyro_Euler[NOW][YAW  ],
		// Surface.output_angle_Servo[NOW][Wing_left],
		// Surface.output_angle_Servo[NOW][Wing_right],
		// Surface.output_angle_Servo[NOW][Vertical_fin],
		// ADC_Voltage_Real
	)
	;
}

void IMU_Task(void)
{
	IMU_Data_Read();
	IMU_Attitude_Algorithm();
}
