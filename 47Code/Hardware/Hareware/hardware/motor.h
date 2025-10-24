// motor.h ??
#ifndef __MOTOR_H
#define __MOTOR_H
#define MIN_SPEED 0        // ????(??)
#define MAX_SPEED 100      // ????(????????)
#define SPEED_STEP 10 

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// ????(??????)
void MotorA_PID_Update(int set_speed);
void MotorB_PID_Update(int set_speed);
void motor_init(void);
void motor_gpio_ctrl(GPIO_PinState a1, GPIO_PinState a2, 
                     GPIO_PinState b1, GPIO_PinState b2);
void MotorA_SetSpeed(int16_t speed);
void MotorB_SetSpeed(int16_t speed);
void move_on(void);
void move_off(void);
void move_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
