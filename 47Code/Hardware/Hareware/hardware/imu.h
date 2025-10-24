#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include "i2c.h"

#define MPU9250_ADDR (0x68 << 1)  // HAL¿âµØÖ·¸ñÊ½

uint8_t MPU9250_Init(void);
void MPU9250_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);

#endif

