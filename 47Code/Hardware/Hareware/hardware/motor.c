#include "motor.h"
#include "tim.h"
#include "pid.h"
#include "motor.h"
#include "tim.h"
#include "pid.h"
#include "stm32f4xx_hal.h" 
// ??????PID??????
PID_HandleTypeDef pid_A, pid_B;




static int16_t target_speed_A = 0;
static int16_t target_speed_B = 0;









// ???abs??(???????)
int abs(int n)
{
    if(n > 0)
    {
        return n;
    }
    return -n;
}

// ??????PID???
void motor_init(void)
{
    // ???PID???(???????)
    PID_Init(&pid_A, 0.8f, 0.02f, 0.1f, -1000.0f, 1000.0f, -300.0f, 300.0f);
    PID_Init(&pid_B, 0.8f, 0.02f, 0.1f, -1000.0f, 1000.0f, -300.0f, 300.0f);
    
    // ??PWM??
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

// ??GPIO??(???????)
void motor_gpio_ctrl(GPIO_PinState a1, GPIO_PinState a2, GPIO_PinState b1, GPIO_PinState b2)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, a1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, a2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, b1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, b2);
}

// ???????(???????)
int16_t EncoderA_Get(void) {
    int16_t val = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return val;
}

int16_t EncoderB_Get(void) {
    int16_t val = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return val;
}

// ????????(???????)
// ??MotorA_SetSpeed??(MotorB_SetSpeed??)
void MotorA_SetSpeed(int16_t speed) {
    // ?????[-MAX_SPEED, MAX_SPEED]???(??????)
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;
    
    // ??PWM???(??????,?????????)
    uint32_t pwm = abs(speed) * (htim3.Init.Period + 1) / 100;
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}

void MotorB_SetSpeed(int16_t speed) {
	   if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;
    
    uint32_t pwm = abs(speed) * (htim3.Init.Period + 1) / 100;
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void MotorA_PID_Update(int set_speed) {
    uint32_t current_time = HAL_GetTick();  // ?????????
    PID_SetTarget(&pid_A, (float)set_speed);
    int current_speed = EncoderA_Get();
    float output = PID_Compute(&pid_A, (float)current_speed);
    int16_t pwm_output = (int16_t)(output / 10.0f);
    MotorA_SetSpeed(pwm_output);
}

void MotorB_PID_Update(int set_speed) {
    uint32_t current_time = HAL_GetTick();  // ?????????
    PID_SetTarget(&pid_B, (float)set_speed);
    int current_speed = EncoderB_Get();
    float output = PID_Compute(&pid_B, (float)current_speed);
    int16_t pwm_output = (int16_t)(output / 10.0f);
    MotorB_SetSpeed(pwm_output);
}

// ????????(???????)
void move_on(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void move_off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void move_toggle(void)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}