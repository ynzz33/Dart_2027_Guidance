#ifndef VOFA_SEND_H
#define VOFA_SEND_H
#include "main.h"



typedef struct
{
  float ch_data[2];
	uint8_t  tail[4];
} Vofa_data_m_2;

typedef struct
{
  float ch_data[4];
	uint8_t  tail[4];
} Vofa_data_m_4;

typedef struct
{
  float ch_data[8];
	uint8_t  tail[4];
} Vofa_data_m_8;

typedef struct
{
  float ch_data[16];
	uint8_t  tail[4];
} Vofa_data_m_16;
typedef struct
{
	float ch_data[32];
	uint8_t  tail[4];
} Vofa_data_m_32;
typedef struct
{
	float ch_data[24];
	uint8_t  tail[4];

}Vofa_data_m_24;





extern Vofa_data_m_2 Vofa_data_2;
extern Vofa_data_m_4 Vofa_data_4;
extern Vofa_data_m_8 Vofa_data_8;
extern Vofa_data_m_16 Vofa_data_16;
extern Vofa_data_m_32 Vofa_data_32;

void Vofa_Send_Data2(float data1, float data2);
void Vofa_Send_Data4(float data1, float data2,float data3, float data4);
void Vofa_Send_Data8(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8);

void Vofa_Send_Data16(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8,
	float data9, float data10,float data11, float data12,float data13, float data14,float data15, float data16);
void Vofa_8(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8);
void Vofa_Send_Data32(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8,
	float data9, float data10,float data11, float data12,float data13, float data14,float data15, float data16,
	float data17, float data18,float data19, float data20,float data21, float data22,float data23, float data24,float data25, float data26, float data27, float data28, float data29, float data30, float data31, float data32);
void Vofa_Send_Data24(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8, float data9, float data10,float data11, float data12,float data13, float data14,float data15, float data16,
	float data17, float data18,float data19, float data20,float data21, float data22,float data23, float data24);

#endif

