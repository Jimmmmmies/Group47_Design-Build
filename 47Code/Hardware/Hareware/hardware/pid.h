#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// PID????????
typedef struct {
    float Kp, Ki, Kd;           // PID??
    float Setpoint;             // ???
    float CurrentValue;         // ???
    float Error;                // ????
    float LastError;            // ?????
    float Integral;             // ???
    float Derivative;           // ???
    float Output;               // ???
    float OutputMin, OutputMax; // ????
    float IntegralMin, IntegralMax; // ????
    uint32_t LastTime;          // ??????
    float dt;                   // ????(?)
    bool FirstRun;              // ??????
} PID_HandleTypeDef;

// ????
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, 
              float output_min, float output_max, 
              float integral_min, float integral_max);
              
float PID_Compute(PID_HandleTypeDef *pid, float current_value);
void PID_SetTarget(PID_HandleTypeDef *pid, float target);
float PID_GetOutput(PID_HandleTypeDef *pid);
void PID_Reset(PID_HandleTypeDef *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
// ?????