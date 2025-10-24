#include "rplidar.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>
#include "motor.h"

uint8_t rplidar_rx_byte;
static uint8_t rplidar_response_header[7];
static uint8_t scan_started = 0;

// 避障相关变量
#define OBSTACLE_DETECTION_DISTANCE 30.0f  // 障碍物检测距离(cm)
#define OBSTACLE_CONFIRM_COUNT 5          // 确认次数，避免误报
#define OBSTACLE_CLEAR_COUNT 10           // 清除障碍物需要的次数
#define MAX_VALID_DISTANCE 1000.0f        // 最大有效距离(mm)
#define MIN_VALID_DISTANCE 50.0f          // 最小有效距离(mm)

static uint8_t obstacle_detected = 0;     // 障碍物检测标志
static uint8_t obstacle_confirm_count = 0; // 确认计数器
static uint8_t obstacle_clear_count = 0;   // 清除计数器
static float front_distance = 100.0f;     // 前方距离
static float min_distance = 100.0f;       // 最小距离
static float distance_buffer[5] = {1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f}; // 距离滤波缓冲区(mm)
static uint8_t buffer_index = 0;          // 缓冲区索引
static float current_filtered_distance = 1000.0f; // 当前滤波后的距离(mm)

void RPLIDAR_Init(void)
{
	HAL_Delay(100); 
    // ??????,?????????(7??)
    HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1);
    RPLIDAR_RequestScan();
}

void RPLIDAR_RequestScan(void)
{
    uint8_t cmd[] = {0xA5, 0x20};
    HAL_UART_Transmit(&huart6, cmd, 2, HAL_MAX_DELAY);
}

void RPLIDAR_ProcessByte(uint8_t byte) {
    static uint8_t buffer[5];
    static int index = 0;
    static int header_index = 0;
    static int print_count = 0;

    if (!scan_started) {
        rplidar_response_header[header_index++] = byte;
        if (header_index == 7) {
            header_index = 0;
            if (rplidar_response_header[0] == 0xA5 && rplidar_response_header[1] == 0x5A) {
                scan_started = 1;
            }
        }
    } else {
        buffer[index++] = byte;
        if (index == 5) {
            index = 0;

            if ((buffer[0] & 0x01) != 0x01 || (buffer[0] & 0x02) != 0x00) return;

            uint16_t angle_q6 = ((buffer[1] >> 1) | ((uint16_t)(buffer[2] & 0x7F) << 7));
            uint16_t distance_q2 = (buffer[3] | ((uint16_t)buffer[4] << 8));

            float angle = angle_q6 / 64.0f;
            float distance = distance_q2 / 4.0f;

            // 避障检测：检查前方区域（-15度到+15度）
            if (angle >= 345.0f || angle <= 15.0f) {
                // 数据有效性检查
                if (distance >= MIN_VALID_DISTANCE && distance <= MAX_VALID_DISTANCE) {
                    if (distance < min_distance) {
                        min_distance = distance;
                    }
                    // 实时滤波更新
                    current_filtered_distance = FilterDistance(distance);
                }
            }

            print_count++;
            if (print_count >= 10) {  // ?10??????
                char msg[64];
               snprintf(msg, sizeof(msg), "Angle: %.1f deg  Dist: %.1f mm\r\n", angle, distance);
	
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                print_count = 0;
            }
        }
    }

    HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1);  // ????
}

// 距离滤波函数
float FilterDistance(float new_distance) {
    // 更新缓冲区
    distance_buffer[buffer_index] = new_distance;
    buffer_index = (buffer_index + 1) % 5;
    
    // 计算中位数（简单的排序）
    float temp[5];
    for (int i = 0; i < 5; i++) {
        temp[i] = distance_buffer[i];
    }
    
    // 冒泡排序
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4 - i; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    
    return temp[2]; // 返回中位数
}

// 避障功能实现
void ObstacleAvoidance_Init(void) {
    obstacle_detected = 0;
    obstacle_confirm_count = 0;
    obstacle_clear_count = 0;
    front_distance = 100.0f;
    min_distance = 100.0f;
    buffer_index = 0;
    
    // 初始化距离缓冲区
    for (int i = 0; i < 5; i++) {
        distance_buffer[i] = 1000.0f;  // 初始化为1000mm
    }
    current_filtered_distance = 1000.0f;
}

void ObstacleAvoidance_Update(void) {
    // 使用实时滤波后的距离（mm转cm）
    float filtered_distance_cm = current_filtered_distance / 10.0f;
    front_distance = filtered_distance_cm;
    
    // 重置最小距离
    min_distance = 1000.0f;
    
    // 改进的障碍物检测逻辑
    if (filtered_distance_cm < OBSTACLE_DETECTION_DISTANCE) {
        obstacle_confirm_count++;
        obstacle_clear_count = 0;  // 重置清除计数器
        
        if (obstacle_confirm_count >= OBSTACLE_CONFIRM_COUNT) {
            if (!obstacle_detected) {
                obstacle_detected = 1;
                // 立即停止
                MotorA_SetSpeed(0);
                MotorB_SetSpeed(0);
                
                char msg[64];
                sprintf(msg, "OBSTACLE DETECTED! Distance: %.2f cm (Filtered)\r\n", filtered_distance_cm);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
        }
    } else {
        obstacle_confirm_count = 0;
        
        if (obstacle_detected) {
            obstacle_clear_count++;
            if (obstacle_clear_count >= OBSTACLE_CLEAR_COUNT) {
                obstacle_detected = 0;
                obstacle_clear_count = 0;
                char msg[64];
                sprintf(msg, "OBSTACLE CLEARED! Distance: %.2f cm (Filtered)\r\n", filtered_distance_cm);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
        }
    }
}

uint8_t ObstacleAvoidance_IsBlocked(void) {
    return obstacle_detected;
}

float ObstacleAvoidance_GetFrontDistance(void) {
    return front_distance;
}
