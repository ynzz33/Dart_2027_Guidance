/**
  **********************************2022 CKYF***********************************
  * @file    Vofa_send.c
  * @brief   ��Vofa+��λ��ͨѶ����
	* @author  �������� - ����
  ******************************************************************************
  * @attention
  *	
  * ��һ������λ������2/4/8��float���ݡ�
  *
  **********************************2022 CKYF***********************************
  */
	
#include "Vofa_send.h"
#include "string.h"
#include "usart.h"

Vofa_data_m_2 Vofa_data_2={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_4 Vofa_data_4={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_8 Vofa_data_8={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_16 Vofa_data_16={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_24 Vofa_data_24={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_32 Vofa_data_32={.tail={0x00,0x00,0x80,0x7f}};

void Vofa_Send_Data2(float data1, float data2)
{
	Vofa_data_2.ch_data[0] = data1;
	Vofa_data_2.ch_data[1] = data2;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Vofa_data_2, sizeof(Vofa_data_2));
}

void Vofa_Send_Data4(float data1, float data2,float data3, float data4)
{
	Vofa_data_4.ch_data[0] = data1;
	Vofa_data_4.ch_data[1] = data2;
	Vofa_data_4.ch_data[2] = data3;
	Vofa_data_4.ch_data[3] = data4;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Vofa_data_4, sizeof(Vofa_data_4));
}

void Vofa_Send_Data8(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8)
{
	Vofa_data_8.ch_data[0] = data1;
	Vofa_data_8.ch_data[1] = data2;
	Vofa_data_8.ch_data[2] = data3;
	Vofa_data_8.ch_data[3] = data4;
	Vofa_data_8.ch_data[4] = data5;
	Vofa_data_8.ch_data[5] = data6;
	Vofa_data_8.ch_data[6] = data7;
	Vofa_data_8.ch_data[7] = data8;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Vofa_data_8, sizeof(Vofa_data_8));
}



void Vofa_Send_Data16(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8,
	float data9, float data10,float data11, float data12,float data13, float data14,float data15, float data16)
{
	Vofa_data_16.ch_data[0] = data1;
	Vofa_data_16.ch_data[1] = data2;
	Vofa_data_16.ch_data[2] = data3;
	Vofa_data_16.ch_data[3] = data4;
	Vofa_data_16.ch_data[4] = data5;
	Vofa_data_16.ch_data[5] = data6;
	Vofa_data_16.ch_data[6] = data7;
	Vofa_data_16.ch_data[7] = data8;
	Vofa_data_16.ch_data[8] = data9;
	Vofa_data_16.ch_data[9] = data10;
	Vofa_data_16.ch_data[10] = data11;
	Vofa_data_16.ch_data[11] = data12;
	Vofa_data_16.ch_data[12] = data13;
	Vofa_data_16.ch_data[13] = data14;
	Vofa_data_16.ch_data[14] = data15;
	Vofa_data_16.ch_data[15] = data16;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Vofa_data_16, sizeof(Vofa_data_16));
}

void Vofa_Send_Data32(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8,
	float data9, float data10,float data11, float data12,float data13, float data14,float data15, float data16,
	float data17, float data18,float data19, float data20,float data21, float data22,float data23, float data24,
	float data25, float data26, float data27, float data28, float data29, float data30, float data31, float data32)
{
	Vofa_data_32.ch_data[0] = data1;
	Vofa_data_32.ch_data[1] = data2;
	Vofa_data_32.ch_data[2] = data3;
	Vofa_data_32.ch_data[3] = data4;
	Vofa_data_32.ch_data[4] = data5;
	Vofa_data_32.ch_data[5] = data6;
	Vofa_data_32.ch_data[6] = data7;
	Vofa_data_32.ch_data[7] = data8;
	Vofa_data_32.ch_data[8] = data9;
	Vofa_data_32.ch_data[9] = data10;
	Vofa_data_32.ch_data[10] = data11;
	Vofa_data_32.ch_data[11] = data12;
	Vofa_data_32.ch_data[12] = data13;
	Vofa_data_32.ch_data[13] = data14;
	Vofa_data_32.ch_data[14] = data15;
	Vofa_data_32.ch_data[15] = data16;
	Vofa_data_32.ch_data[16] = data17;
	Vofa_data_32.ch_data[17] = data18;
	Vofa_data_32.ch_data[18] = data19;
	Vofa_data_32.ch_data[19] = data20;
	Vofa_data_32.ch_data[20] = data21;
	Vofa_data_32.ch_data[21] = data22;
	Vofa_data_32.ch_data[22] = data23;
	Vofa_data_32.ch_data[23] = data24;
	Vofa_data_32.ch_data[24] = data25;
	Vofa_data_32.ch_data[25] = data26;
	Vofa_data_32.ch_data[26] = data27;
	Vofa_data_32.ch_data[27] = data28;
	Vofa_data_32.ch_data[28] = data29;
	Vofa_data_32.ch_data[29] = data30;
	Vofa_data_32.ch_data[30] = data31;
	Vofa_data_32.ch_data[31] = data32;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Vofa_data_32, sizeof(Vofa_data_32));
}
void Vofa_Send_Data24(float data1 ,float data2,float data3, float data4,float data5, float data6,float data7, float data8, float data9, float data10,float data11, float data12,float data13, float data14,float data15, float data16,
	float data17, float data18,float data19, float data20,float data21, float data22,float data23, float data24)
{
	Vofa_data_24.ch_data[0] = data1;
	Vofa_data_24.ch_data[1] = data2;
	Vofa_data_24.ch_data[2] = data3;
	Vofa_data_24.ch_data[3] = data4;
	Vofa_data_24.ch_data[4] = data5;
	Vofa_data_24.ch_data[5] = data6;
	Vofa_data_24.ch_data[6] = data7;
	Vofa_data_24.ch_data[7] = data8;
	Vofa_data_24.ch_data[8] = data9;
	Vofa_data_24.ch_data[9] = data10;
	Vofa_data_24.ch_data[10] = data11;
	Vofa_data_24.ch_data[11] = data12;
	Vofa_data_24.ch_data[12] = data13;
	Vofa_data_24.ch_data[13] = data14;
	Vofa_data_24.ch_data[14] = data15;
	Vofa_data_24.ch_data[15] = data16;
	Vofa_data_24.ch_data[16] = data17;
	Vofa_data_24.ch_data[17] = data18;
	Vofa_data_24.ch_data[18] = data19;
	Vofa_data_24.ch_data[19] = data20;
	Vofa_data_24.ch_data[20] = data21;
	Vofa_data_24.ch_data[21] = data22;
	Vofa_data_24.ch_data[22] = data23;
	Vofa_data_24.ch_data[23] = data24;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Vofa_data_24, sizeof(Vofa_data_24));

}

