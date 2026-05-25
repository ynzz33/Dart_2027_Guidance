//
// Created by ynz on 2025/12/6.
//

#include "ADC_Battery.h"
#include "adc.h"
#include "filter.h"

uint16_t Battery_Buf[16] = {0};
void ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)Battery_Buf, 16);
    ADC_Battery_Kalman_Filter.p_last = 0,ADC_Battery_Kalman_Filter.x_last = 0;
}

float ADC_GET_REAL_VALUE(void)
{
    uint16_t ADC_Raw;
    static float ADC_Z = 0,Last_ADC_Z = 0;
    ADC_Raw = Battery_Buf[15];
    /* 4. 计算实际电压（核心公式） */
    ADC_Z = (ADC_Raw/(float)4095)*13.6f;
    ADC_Z = KalmanFilter( &ADC_Battery_Kalman_Filter,ADC_Z, 0.01,500);
    return ADC_Z;
}

