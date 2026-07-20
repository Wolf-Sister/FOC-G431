/**
  ******************************************************************************
  * @file    as5047p_ext.c
  * @brief   AS5047P 编码器扩展 — 速度估算、多圈累积
  *
  *          使用方法（与 TinyFoc AS5600 完全一致）：
  *            1. AS5047P_Sensor_Init(&AngleSensor);
  *            2. 以固定频率 (20kHz) 调用 AS5047P_Sensor_Update(&AngleSensor)
  *            3. 通过 AS5047P_GetAngle / GetVelocity / GetAccumulateAngle 读取
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "as5047p_ext.h"
#include "as5047p.h"
#include "utils.h"

/* 转换常数: AS5047P 14 位 (0–16383) → 0–2π 弧度 -----------*/
#define AS5047P_RAD_SCALE     (_2PI / AS5047P_ANGLE_STEPS)
#define AS5047P_MAX_SAMPLE_DT 0.1f

/* 全局传感器实例 ----------------------------------------------------*/
AS5047P_Sensor_T AngleSensor = {0};

/* 双缓冲编码器缓存：写入者填充非活跃槽，然后翻转索引 */
static volatile encoder_cache_t encoder_cache_buffers[2] = {0};
static volatile uint8_t encoder_cache_active = 0;

static void AS5047P_VelocityWindow_Reset(AS5047P_Sensor_T *s)
{
    s->velocity_rad_s      = 0.0f;
    s->velocity_count_sum  = 0;
    s->velocity_cycle_sum  = 0U;
    s->velocity_window_index = 0U;
    s->velocity_window_count = 0U;
}

static void AS5047P_VelocityWindow_Update(AS5047P_Sensor_T *s,
                                          int16_t delta_count,
                                          uint32_t delta_cycles)
{
    uint8_t index = s->velocity_window_index;

    if (s->velocity_window_count >= AS5047P_VELOCITY_WINDOW_SIZE) {
        s->velocity_count_sum -= s->velocity_count_window[index];
        s->velocity_cycle_sum -= s->velocity_cycle_window[index];
    } else {
        s->velocity_window_count++;
    }

    s->velocity_count_window[index] = delta_count;
    s->velocity_cycle_window[index] = delta_cycles;
    s->velocity_count_sum += delta_count;
    s->velocity_cycle_sum += delta_cycles;

    index++;
    if (index >= AS5047P_VELOCITY_WINDOW_SIZE) index = 0U;
    s->velocity_window_index = index;

    if (s->velocity_window_count >= AS5047P_VELOCITY_WINDOW_SIZE &&
        s->velocity_cycle_sum > 0U) {
        s->velocity_rad_s = (float)s->velocity_count_sum
                            * AS5047P_RAD_SCALE
                            * (float)SystemCoreClock
                            / (float)s->velocity_cycle_sum;
    }
}

/* --------------------------------------------------------------------------*/
/**
  * @brief  初始化传感器状态
  */
void AS5047P_Sensor_Init(AS5047P_Sensor_T *s)
{
    s->prev_angle        = 0.0f;
    s->turn_count        = 0;
    s->total_angle       = 0.0f;
    s->velocity_rad_s    = 0.0f;
    s->prev_raw          = 0U;
    s->prev_cycle        = 0U;
    s->sample_cycle      = 0U;
    s->velocity_count_sum = 0;
    s->velocity_cycle_sum = 0U;
    s->total_errors      = 0U;
    s->consecutive_errors = 0U;
    s->velocity_window_index = 0U;
    s->velocity_window_count = 0U;
    s->initialized       = 0U;

    for (uint8_t i = 0U; i < AS5047P_VELOCITY_WINDOW_SIZE; i++) {
        s->velocity_count_window[i] = 0;
        s->velocity_cycle_window[i] = 0U;
    }
}

/**
  * @brief  更新传感器读数 — 以固定频率 (20kHz TIM ISR) 调用
  *
  *         1. 读取最新的 DMA 捕获原始角度
  *         2. 检测圈数翻转，实现多圈累积
  *         3. 根据角度增量 / 时间增量计算速度
  */
bool AS5047P_Sensor_Update(AS5047P_Sensor_T *s)
{
    uint32_t angle_sample_cycle = 0U;
    uint16_t raw = AS5047P_DMA_GetAngleCallback(&angle_sample_cycle);
    if (raw == 0xFFFFU) {
        s->total_errors++;
        s->consecutive_errors++;
        (void)AS5047P_DMA_StartRequest();
        return false;
    }

    float angle = (float)raw * AS5047P_RAD_SCALE;
    uint32_t now_cycle = angle_sample_cycle;

    if (!s->initialized) {
        s->prev_angle         = angle;
        s->total_angle        = angle;
        s->turn_count         = 0;
        s->prev_raw           = raw;
        s->prev_cycle         = now_cycle;
        s->sample_cycle       = now_cycle;
        AS5047P_VelocityWindow_Reset(s);
        s->initialized        = 1U;
        s->consecutive_errors = 0;
        (void)AS5047P_DMA_StartRequest();
        return true;
    }

    int32_t delta_count = (int32_t)raw - (int32_t)s->prev_raw;

    if (delta_count > 8192) {
        delta_count -= 16384;
        s->turn_count--;
    }
    else if (delta_count < -8192) {
        delta_count += 16384;
        s->turn_count++;
    }

    s->total_angle = (float)s->turn_count * _2PI + angle;

    uint32_t delta_cycles = now_cycle - s->prev_cycle;
    float dt = dwt_cycles_to_seconds(delta_cycles);
    if (dt > 0.0f && dt < AS5047P_MAX_SAMPLE_DT) {
        AS5047P_VelocityWindow_Update(s, (int16_t)delta_count, delta_cycles);
    } else {
        AS5047P_VelocityWindow_Reset(s);
    }

    s->prev_angle         = angle;
    s->prev_raw           = raw;
    s->prev_cycle         = now_cycle;
    s->sample_cycle       = now_cycle;
    s->consecutive_errors = 0;

    (void)AS5047P_DMA_StartRequest();
    return true;
}

void AS5047P_EncoderCache_Publish(const AS5047P_Sensor_T *s)
{
    uint8_t current = encoder_cache_active;
    uint8_t next = current ^ 1U;
    uint32_t next_count = encoder_cache_buffers[current].update_count + 1U;

    encoder_cache_buffers[next].angle_raw       = s->prev_angle;
    encoder_cache_buffers[next].velocity_rad_s  = s->velocity_rad_s;
    encoder_cache_buffers[next].total_angle_rad = s->total_angle;
    encoder_cache_buffers[next].sample_cycle    = s->sample_cycle;
    encoder_cache_buffers[next].update_count    = next_count;
    encoder_cache_buffers[next].data_valid      = 1U;

    __DMB();
    encoder_cache_active = next;
}

bool AS5047P_EncoderCache_Read(encoder_cache_t *snapshot)
{
    if (snapshot == NULL) return false;

    uint8_t before;
    uint8_t after;
    do {
        before = encoder_cache_active;
        __DMB();
        snapshot->angle_raw       = encoder_cache_buffers[before].angle_raw;
        snapshot->velocity_rad_s  = encoder_cache_buffers[before].velocity_rad_s;
        snapshot->total_angle_rad = encoder_cache_buffers[before].total_angle_rad;
        snapshot->sample_cycle    = encoder_cache_buffers[before].sample_cycle;
        snapshot->update_count    = encoder_cache_buffers[before].update_count;
        snapshot->data_valid      = encoder_cache_buffers[before].data_valid;
        __DMB();
        after = encoder_cache_active;
    } while (before != after);

    return snapshot->data_valid != 0U;
}

/**
  * @brief  获取当前机械角度 [0, 2π)
  */
float AS5047P_GetAngle(const AS5047P_Sensor_T *s)
{
    return s->prev_angle;
}

/**
  * @brief  获取瞬时角速度 [rad/s]
  */
float AS5047P_GetVelocity(const AS5047P_Sensor_T *s)
{
    return s->velocity_rad_s;
}

/**
  * @brief  获取累积多圈角度 [rad]
  */
float AS5047P_GetAccumulateAngle(const AS5047P_Sensor_T *s)
{
    return s->total_angle;
}

/**
  * @brief  获取原始角度，单次转换（阻塞式，仅用于校准）
  */
float AS5047P_GetOnceAngle(const AS5047P_Sensor_T *s)
{
    uint16_t raw = AS5047P_DMA_GetAngleCallback(NULL);
    if (raw == 0xFFFFU) return s->prev_angle;
    return (float)raw * AS5047P_RAD_SCALE;
}
