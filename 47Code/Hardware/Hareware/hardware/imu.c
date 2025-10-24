#include "imu.h"

uint8_t MPU9250_Init(void)
{
    uint8_t check, data;

    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x75, 1, &check, 1, 100);
    if (check == 0x70) {
        data = 0x00; 
        HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x6B, 1, &data, 1, 100);
        return 1;
    }
    return 0;
}

void MPU9250_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x43, 1, buf, 6, 100);
    *gx = (int16_t)(buf[0] << 8 | buf[1]);
    *gy = (int16_t)(buf[2] << 8 | buf[3]);
    *gz = (int16_t)(buf[4] << 8 | buf[5]);
}

