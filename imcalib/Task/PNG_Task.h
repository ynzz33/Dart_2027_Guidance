//
// Created by ynz on 2026/1/10.
//

#ifndef PNG_TASK_H
#define PNG_TASK_H

#define SENSOR_FOV 70
#define SENSOR_TOTAL_PIXEL_WIDTH 320
#define SAMPLE_RATE 20
#define N_rate 3.0f
#define MAX_PNG_OUT 45
#define Vc_min 1.0
#define K_Dyn 8.0
#include <stdint.h>
#include "CallBack_Task.h"
#include "IMU.h"
#include "surface_control_task.h"

typedef struct
{
	float FOV;
	float los_GYRO[2][2];
	float los_ANGLE[2][2];
	float los_vector[3];
	float V_c;
	float N_R;
}PNG_Data_t;


extern PNG_Data_t PNG_Data;

void PNG_Init(PNG_Data_t* PNG_Data);
void PNG_Guidance(Vision_Rx_Buf_t* Vision_Data , PNG_Data_t* PNG_Data , Surface_t* Surface , IMU_DATA_t* IMU_Data);


#endif //PNG_TASK_H
