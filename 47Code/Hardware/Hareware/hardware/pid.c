#include "pid.h"
#include <stdint.h>
#include <stdbool.h>


void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, 
              float output_min, float output_max, 
              float integral_min, float integral_max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Setpoint = 0;
    pid->CurrentValue = 0;
    pid->Error = 0;
    pid->LastError = 0;
    pid->Integral = 0;
    pid->Derivative = 0;
    pid->Output = 0;
    pid->OutputMin = output_min;
    pid->OutputMax = output_max;
    pid->IntegralMin = integral_min;
    pid->IntegralMax = integral_max;
    pid->LastTime = HAL_GetTick();
    pid->dt = 0.01f;            // ??????10ms
    pid->FirstRun = true;
}

// ??PID??(?????)
float PID_Compute(PID_HandleTypeDef *pid, float current_value) {
    uint32_t current_time = HAL_GetTick();
    
    // ??????
    if (pid->FirstRun) {
        pid->FirstRun = false;
        pid->dt = 0.01f;        // ?????????
    } else {
        pid->dt = (current_time - pid->LastTime) / 1000.0f;
        // ????????,??????
        if (pid->dt > 0.1f) pid->dt = 0.1f;
    }
    pid->LastTime = current_time;
    
    // ????
    pid->CurrentValue = current_value;
    pid->Error = pid->Setpoint - pid->CurrentValue;
    
    // ?????(?????)
    pid->Integral += pid->Error * pid->dt;
    
    // ????,??????
    if (pid->Integral > pid->IntegralMax)
        pid->Integral = pid->IntegralMax;
    else if (pid->Integral < pid->IntegralMin)
        pid->Integral = pid->IntegralMin;
    
    // ?????(?????)
    if (pid->dt > 0)
        pid->Derivative = (pid->Error - pid->LastError) / pid->dt;
    else
        pid->Derivative = 0;
    
    // PID????
    pid->Output = pid->Kp * pid->Error 
                + pid->Ki * pid->Integral 
                + pid->Kd * pid->Derivative;
    
    // ????
    if (pid->Output > pid->OutputMax)
        pid->Output = pid->OutputMax;
    else if (pid->Output < pid->OutputMin)
        pid->Output = pid->OutputMin;
    
    // ??????
    pid->LastError = pid->Error;
    
    return pid->Output;
}

// ??PID???
void PID_SetTarget(PID_HandleTypeDef *pid, float target) {
    pid->Setpoint = target;
}

// ????PID??
float PID_GetOutput(PID_HandleTypeDef *pid) {
    return pid->Output;
}

// ??PID?????
void PID_Reset(PID_HandleTypeDef *pid) {
    pid->Error = 0;
    pid->LastError = 0;
    pid->Integral = 0;
    pid->Derivative = 0;
    pid->Output = 0;
    pid->FirstRun = true;
}