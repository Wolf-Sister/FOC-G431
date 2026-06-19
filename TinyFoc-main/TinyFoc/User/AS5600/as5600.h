#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdbool.h>
#include <stdint.h>

#include "i2c.h"

#define AS5600_RAW_ADDR            0x36
#define AS5600_ADDR                (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR          (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR           ((AS5600_RAW_ADDR << 1) | 1)
#define AS5600_RAW_ANGLE_REGISTER  0x0C

#define AS5600_RESOLUTION 4096.0f // 12bit Resolution

typedef struct {
    I2C_HandleTypeDef *i2c_ins;

    // --- 状态变量 ---
    float prev_angle_raw;         // 上一次的原始角度值 (0-4095)
    unsigned long prev_update_ts; // 上一次调用的时间戳

    // --- 计算结果 ---
    float rotation_offset;        // 累计的圈数偏移量 (弧度)
    float total_angle_rad;        // 累计的总角度 (弧度)
    float velocity_rad_s;         // 角速度 (弧度/秒)

} AS5600_T;

extern AS5600_T AngleSensor;

int AS5600_Init(AS5600_T *a, I2C_HandleTypeDef *hi2c);
float AS5600_GetAccumulateAngle(AS5600_T *a);
uint16_t AS5600_GetRawAngle(AS5600_T *a);
float AS5600_GetOnceAngle(AS5600_T *a);
float AS5600_GetVelocity(AS5600_T *a);
float AS5600_GetAngle(AS5600_T *a);
void AS5600_Update(AS5600_T *a);

#endif