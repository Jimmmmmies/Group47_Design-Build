#include "bluetooth.h"
#include "main.h"
#include "usart.h"
#include "motor.h"
#include "imu.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "imu.h"
#include "data.h"
#include "rplidar.h"
 
static int16_t current_speed = 50;  
extern uint8_t rplidar_rx_byte;  // ????????????
uint8_t bluetooth_data;
extern float yaw;
extern int16_t gx, gy, gz;
extern float gyro_z_offset;

// 路径规划目标坐标
static float target_x = 0.0f;
static float target_y = 0.0f;
static float target_z = 0.0f;  // 目标z坐标

// 当前位置（简化版本，实际应该根据编码器等计算）
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;

// 接收缓冲区
#define RX_BUFFER_SIZE 32
static char rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;

void uart_init()
{
	HAL_UART_Receive_IT(&huart3, &bluetooth_data, 1);
}

// 解析坐标指令并执行
void ParseAndExecuteCoordinate(const char* command) {
    // 指令格式: "x,y,z" 例如: "10.5,20.3,0.0"
    if (sscanf(command, "%f,%f,%f", &target_x, &target_y, &target_z) == 3) {
        char msg[128];
        sprintf(msg, "TARGET: X=%.2f Y=%.2f Z=%.2f | CURRENT: X=%.2f Y=%.2f Z=%.2f\r\n",
                target_x, target_y, target_z, current_x, current_y, current_z);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        
        // 根据目标坐标执行动作
        ExecuteCoordinateCommand();
    } else {
        char msg[64];
        sprintf(msg, "ERROR: Invalid coordinate format\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

// 根据坐标执行动作
void ExecuteCoordinateCommand(void) {
    char msg[64];
    
    // 检查避障状态
    if (ObstacleAvoidance_IsBlocked()) {
        sprintf(msg, "MOVEMENT BLOCKED by obstacle!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }
    
    // 计算到目标的距离
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float dz = target_z - current_z;
    
    float distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    sprintf(msg, "DISTANCE: %.2f cm\r\n", distance);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    // 简单的移动逻辑：根据主要方向决定动作
    if (fabs(dx) > fabs(dy) && fabs(dx) > fabs(dz)) {
        // X方向移动最大
        if (dx > 0) {
            if (!ObstacleAvoidance_IsBlocked()) {  // 再次检查避障
                MotorA_SetSpeed(current_speed);
                MotorB_SetSpeed(-current_speed);
                sprintf(msg, "MOVING FORWARD (X+)\r\n");
            } else {
                sprintf(msg, "FORWARD BLOCKED by obstacle!\r\n");
            }
        } else {
            MotorA_SetSpeed(-current_speed);
            MotorB_SetSpeed(current_speed);
            sprintf(msg, "MOVING BACKWARD (X-)\r\n");
        }
    } else if (fabs(dy) > fabs(dz)) {
        // Y方向移动最大
        if (dy > 0) {
            MotorA_SetSpeed(current_speed);
            MotorB_SetSpeed(current_speed);
            sprintf(msg, "MOVING LEFT (Y+)\r\n");
        } else {
            MotorA_SetSpeed(-current_speed);
            MotorB_SetSpeed(-current_speed);
            sprintf(msg, "MOVING RIGHT (Y-)\r\n");
        }
    } else {
        // Z方向移动最大
        if (dz > 0) {
            MotorA_SetSpeed(current_speed);
            MotorB_SetSpeed(current_speed);
            sprintf(msg, "MOVING UP (Z+)\r\n");
        } else {
            MotorA_SetSpeed(-current_speed);
            MotorB_SetSpeed(-current_speed);
            sprintf(msg, "MOVING DOWN (Z-)\r\n");
        }
    }
    
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void HandleBluetoothData(uint8_t data) {
  char msg[64];  // ?????

    // ??????????
    MPU9250_ReadGyro(&gx, &gy, &gz);
    float gz_dps = (gz - gyro_z_offset) / 131.0f;
    yaw += gz_dps * 0.05f;  // ??????? 50ms

    // 检查是否是坐标指令
    if (data == '\r' || data == '\n') {
        // 指令结束，解析坐标
        if (rx_index > 0) {
            rx_buffer[rx_index] = '\0';
            ParseAndExecuteCoordinate(rx_buffer);
            rx_index = 0;
        }
        return;
    } else if (rx_index < RX_BUFFER_SIZE - 1) {
        // 接收坐标字符
        rx_buffer[rx_index++] = data;
        return;
    }

	switch (data) {
        case 'B':  // Forward
            if (!ObstacleAvoidance_IsBlocked()) {
                MotorA_SetSpeed(current_speed);
                MotorB_SetSpeed(-current_speed);
                sprintf(msg, "FWD | Yaw: %.2f Gx: %d Gy: %d Gz: %d\r\n", yaw, gx, gy, gz);
            } else {
                sprintf(msg, "FWD BLOCKED | Yaw: %.2f Gx: %d Gy: %d Gz: %d\r\n", yaw, gx, gy, gz);
            }
            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            break;
        case 'F':  // Backward
            MotorA_SetSpeed(-current_speed);
            MotorB_SetSpeed(current_speed);
            sprintf(msg, "FWD | Yaw: %.2f Gx: %d Gy: %d Gz: %d\r\n", yaw, gx, gy, gz);
            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            break;
        case 'L':  // Left
            MotorA_SetSpeed(current_speed);
            MotorB_SetSpeed(current_speed);
            sprintf(msg, "FWD | Yaw: %.2f Gx: %d Gy: %d Gz: %d\r\n", yaw, gx, gy, gz);
            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            break;
        case 'R':  // Right
            MotorA_SetSpeed(-current_speed);
            MotorB_SetSpeed(-current_speed);
            sprintf(msg, "FWD | Yaw: %.2f Gx: %d Gy: %d Gz: %d\r\n", yaw, gx, gy, gz);
            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            break;
        case 'S':  // Stop
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            sprintf(msg, "FWD | Yaw: %.2f Gx: %d Gy: %d Gz: %d\r\n", yaw, gx, gy, gz);
            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            break;
     
        case 'G':  // ??
            if (current_speed < MAX_SPEED) {
                current_speed += SPEED_STEP;
                // ?????????,????????
                // (??:????????????,??????)
                // ??:??????,???????????current_speed
            }
            sprintf(msg, "go,current_speed | MAX_SPEED %d\r\n", current_speed, MAX_SPEED);
            break;
        case 'D':  // ??
            if (current_speed > MIN_SPEED) {
                current_speed -= SPEED_STEP;
                // ??,?????,??????
            }
            sprintf(msg, "down,current_speed: %d | MIN_SPEED %d\r\n", current_speed, MIN_SPEED);
            break;

        default:
            sprintf(msg, "unknown| current_speed : %d\r\n", current_speed);
            break;
    }

    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        // ????????
        HandleBluetoothData(bluetooth_data);
        HAL_UART_Receive_IT(&huart3, &bluetooth_data, 1);  // ??????
    } 
    else if (huart == &huart6) {
        // ????????
        RPLIDAR_ProcessByte(rplidar_rx_byte);
        HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1);  // ??????
    }
}
