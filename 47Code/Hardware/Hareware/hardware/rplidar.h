#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#include "usart.h"
#include "stdint.h"

extern uint8_t rplidar_rx_byte;

void RPLIDAR_Init(void);
void RPLIDAR_ProcessByte(uint8_t byte); // �������ֽڴ���
void RPLIDAR_RequestScan(void); // ����ɨ������

// 避障功能
void ObstacleAvoidance_Init(void);
void ObstacleAvoidance_Update(void);
uint8_t ObstacleAvoidance_IsBlocked(void);
float ObstacleAvoidance_GetFrontDistance(void);

#endif

