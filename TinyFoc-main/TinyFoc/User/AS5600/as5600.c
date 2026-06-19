#include "as5600.h"

#define abs(x) ((x) > 0 ? (x) : -(x))
#define _2PI 6.283185307179586f
#define _PI 3.141592653589793f

AS5600_T AngleSensor;

// 写辅助函数
HAL_StatusTypeDef _WriteData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef sta = HAL_ERROR;
  sta = HAL_I2C_Master_Transmit(hi2c, dev_addr, data, size, -1);
  return sta;
}

// 读辅助函数
HAL_StatusTypeDef _ReadData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef sta = HAL_ERROR;
  sta = HAL_I2C_Master_Receive(hi2c, (dev_addr | 1), data, size, -1);
  return sta;
}

// 初始化函数
int AS5600_Init(AS5600_T *a, I2C_HandleTypeDef *hi2c) {
    a->i2c_ins = hi2c;
    a->prev_angle_raw = AS5600_GetRawAngle(a); // 初始化基准角度
    a->prev_update_ts = HAL_GetTick();         // 初始化时间戳
    a->rotation_offset = 0.0f;
    a->total_angle_rad = 0.0f;
    a->velocity_rad_s  = 0.0f;
    return 0;
}

// 获得原始角度值
uint16_t AS5600_GetRawAngle(AS5600_T *a) {
  uint16_t raw_angle;
  uint8_t buffer[2];
  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;

  _WriteData(a->i2c_ins, AS5600_ADDR, &raw_angle_register, 1);
  _ReadData(a->i2c_ins, AS5600_ADDR, buffer, 2);
  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle;
}

/**
 * @brief 获得实时单圈弧度值 (单位: 弧度)
 * @note 执行I2C通信，返回单圈弧度值
 */
float AS5600_GetOnceAngle(AS5600_T *a) {
  return (float)(AS5600_GetRawAngle(a) / AS5600_RESOLUTION) * _2PI;
}

/**
* @brief 获得上次单圈弧度值 (单位: 弧度)
 * @note 执行I2C通信，返回单圈弧度值
 */
float AS5600_GetAngle(AS5600_T *a) {
  return (float)(a->prev_angle_raw / AS5600_RESOLUTION) * _2PI;
}

/**
 * @brief 获取角速度 (单位: 弧度/秒)
 * @note 不执行I2C通信，直接返回由 AS5600_Update 计算好的值。
 */
float AS5600_GetVelocity(AS5600_T *a) {
    return a->velocity_rad_s;
}

/**
 * @brief 获取累计角度 (单位: 弧度)
 * @note 不执行I2C通信，直接返回由 AS5600_Update 计算好的值。
 */
float AS5600_GetAccumulateAngle(AS5600_T *a) {
    return a->total_angle_rad;
}

/**
 * @brief 更新函数，周期性调用。
 * 该函数会执行I2C读取，并计算最新的累计角度和瞬时速度。
 */
void AS5600_Update(AS5600_T *a) {
    // 读取传感器和时间戳
    float current_angle_raw = AS5600_GetRawAngle(a);
    unsigned long current_ts = HAL_GetTick();

    // 计算时间差 (单位: 秒)
    float delta_t_s = (current_ts - a->prev_update_ts) / 1000.0f;
    
    // 如果时间间隔过小或为零，则不进行更新，防止除以零
    if (delta_t_s < 1e-4) {
        return;
    }

    // 使用旧状态变量, 计算出上一次的总弧度
    float prev_total_angle_rad = a->rotation_offset + (a->prev_angle_raw / AS5600_RESOLUTION) * _2PI;

    // 处理过零问题并累加圈数
    float delta_angle_raw = current_angle_raw - a->prev_angle_raw;
    if (abs(delta_angle_raw) > (0.8f * AS5600_RESOLUTION)) {
        // 发生过零
        a->rotation_offset += (delta_angle_raw > 0 ? -_2PI : _2PI);
    }

    // 计算当前的累计总角度
    a->total_angle_rad = a->rotation_offset + (current_angle_raw / AS5600_RESOLUTION) * _2PI;

    
    // 计算弧度变化量
    float delta_angle_rad = a->total_angle_rad - prev_total_angle_rad;

    // 速度计算
    a->velocity_rad_s = delta_angle_rad / delta_t_s;

    // 更新状态变量，为下一次计算做准备
    a->prev_angle_raw = current_angle_raw;
    a->prev_update_ts = current_ts;
}

