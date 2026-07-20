/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ JustFloat 发送 + 文本命令接收（通过 UART2 DMA）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "vofa.h"
#include "foc.h"        /* UART2_SendString(), motor_control, motor_config    */
#include "usart.h"      /* huart2, hdma_usart2_rx                             */
#include "pid.h"        /* foc_set_current_pid(), foc_set_id_current_pid()    */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ── RX state ──────────────────────────────────────────────────────────── */
static uint8_t  rx_buf[VOFA_RX_BUF_SIZE];        /* DMA NORMAL 模式目标缓冲区  */
static char     rx_line[VOFA_RX_BUF_SIZE + 1U];        /* 组装后的命令字符串         */
static volatile uint8_t rx_ready = 0;             /* 标志：完整命令帧已就绪     */

/* ========================================================================== */
/*  TX: Send telemetry                                                        */
/* ========================================================================== */

void VOFA_SendData(const float *data, uint8_t count)
{
    static char buf[VOFA_TX_BUF_SIZE];  /* 静态缓冲区：DMA 后台发送安全 */
    int pos = 0;

    if (count == 0 || data == NULL) {
        return;
    }
    if (count > VOFA_MAX_CHANNELS) {
        count = VOFA_MAX_CHANNELS;
    }

    pos = snprintf(buf, sizeof(buf), "channels: ");
    for (uint8_t i = 0; i < count; i++) {
        if (i > 0) pos += snprintf(buf + pos, sizeof(buf) - pos, ",");
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%.6f", (double)data[i]);
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
    UART2_SendString(buf);
}

/* ========================================================================== */
/*  RX: Start DMA reception with IDLE detection                               */
/* ========================================================================== */

/**
  * @brief  初始化 UART2 DMA 接收（NORMAL 模式 + IDLE 检测）
  *         CubeMX 默认配置 DMA 为 CIRCULAR 模式；此处覆盖为 NORMAL，
  *         因为 HAL_UARTEx_ReceiveToIdle_DMA 需要 DMA 在帧尾停止，
  *         才能正确触发 IDLE 回调
  */
void VOFA_InitRx(void)
{
    /* 覆盖 CubeMX 的 CIRCULAR → NORMAL，实现帧级接收 */
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_usart2_rx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, VOFA_RX_BUF_SIZE);
}

/* ========================================================================== */
/*  RX: Weak callback override — called on IDLE / frame received              */
/* ========================================================================== */

/**
  * @brief  覆盖默认的弱定义 HAL_UARTEx_RxEventCallback
  *         当 UART2 接收数据后进入空闲状态时，由 HAL 在中断上下文中调用。
  *         将帧数据复制到行缓冲区、置位标志后快速返回。
  *
  *         DMA 重启延迟到 VOFA_ProcessCmd()（主循环上下文）中执行。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2 && Size > 0 && Size <= VOFA_RX_BUF_SIZE) {
        /* 将 DMA 数据复制到行缓冲区（在 ISR 中快速完成） */
        memcpy(rx_line, rx_buf, Size);
        rx_line[Size] = '\0';
        rx_ready = 1;
        /* DMA 重启延迟到主循环中 VOFA_ProcessCmd() 执行 */
    }
}

/* ========================================================================== */
/*  RX: Text command parser                                                    */
/* ========================================================================== */

/**
  * @brief  解析基于文本的电机控制命令，逗号分隔
  *
  * 格式（任意组合、任意顺序）：
  *   T=V      设置转矩 / Iq 电流指令 (A)
  *   D=V      设置 D 轴电流目标 (A)，SPM 电机默认为 0
  *   P=V      设置 Iq 环 P 增益
  *   I=V      设置 Iq 环 I 增益
  *   DP=V     设置 Id 环 P 增益
  *   DI=V     设置 Id 环 I 增益
  *   S=V      设置速度目标值 (机械角速度 rad/s)
  *   SP=V     设置速度环 P 增益
  *   SI=V     设置速度环 I 增益
  *   SG=V     speed gain schedule enable (0=manual SP, 1=scheduled)
  *   SPL=V    scheduled low-speed P gain
  *   SPH=V    scheduled high-speed P gain
  *   PS=V     设置位置目标值 (多圈弧度 rad)
  *   PR=V     set a relative position step from the measured position (rad)
  *   PP=V     设置位置环 P 增益 (rad/s per rad)
  *   PL=V     设置位置环速度限幅 (rad/s，旧版别名)
  *   CVL=V    设置电流环 dq 电压矢量限幅 (V)
  *   SIL=V    设置速度环 Iq 电流限幅 (A)
  *   PSL=V    设置位置环速度限幅 (rad/s)
  *   PAL=V    position trajectory acceleration limit (rad/s^2)
  *   M=V      设置控制模式 (0=转矩, 1=速度, 2=位置)
  *
  * 任何接收到的命令都会将 status_flag 置为 1，供 Python 端步进同步
  *
  * 示例:
  *   "T=0.5
"
  *   "P=3.0,I=200
"
  *   "M=2,PP=5,PL=100,PS=6.28
"
  */
static void vofa_parse_cmd(const char *line)
{
    const char *p = line;
    uint8_t iq_dirty = 0, id_dirty = 0, spd_dirty = 0, pos_dirty = 0;
    uint8_t torque_dirty = 0, id_cmd_dirty = 0, speed_cmd_dirty = 0;
    uint8_t position_cmd_dirty = 0, position_relative_dirty = 0, mode_dirty = 0;
    uint8_t spd_schedule_dirty = 0, spd_schedule_explicit = 0;
    uint8_t spd_manual_dirty = 0;
    uint8_t pos_accel_dirty = 0;
    float set_torque = motor_control.set_torque;
    float id_target = motor_control.id_target;
    float set_speed = motor_control.set_speed;
    float set_position = motor_control.set_position;
    float position_delta = 0.0f;
    uint8_t new_mode = motor_control.mode;
    float iq_p = motor_config.iq_p_gain;
    float iq_i = motor_config.iq_i_gain;
    float id_p = motor_config.id_p_gain;
    float id_i = motor_config.id_i_gain;
    float spd_p = motor_config.spd_p_gain;
    float spd_i = motor_config.spd_i_gain;
    float pos_p = motor_config.pos_p_gain;
    float spd_p_low = motor_config.spd_p_low_speed;
    float spd_p_high = motor_config.spd_p_high_speed;
    uint8_t spd_schedule = motor_config.spd_gain_schedule;
    float current_voltage_limit = motor_config.current_voltage_limit;
    float speed_current_limit = motor_config.speed_current_limit;
    float pos_limit = motor_config.pos_speed_limit;
    float pos_accel = motor_config.pos_accel_limit;

    while (*p) {
        while (*p == ' ' || *p == ',') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r') break;

        char key[4] = {0};
        int ki = 0;
        while (*p && *p != '=' && *p != ',' && *p != ' ' &&
               *p != '\n' && *p != '\r' && ki < 3) {
            key[ki++] = *p++;
        }
        if (*p != '=') { p++; continue; }
        p++;

        char *end;
        float val = strtof(p, &end);
        if (end == p) break;
        p = end;

        /* Stage every value locally; commit only after the full frame parses. */
        if      (strcmp(key, "T")  == 0) { set_torque = val; torque_dirty = 1; }
        else if (strcmp(key, "D")  == 0) { id_target = val; id_cmd_dirty = 1; }
        else if (strcmp(key, "P")  == 0) { iq_p = val; iq_dirty = 1; }
        else if (strcmp(key, "I")  == 0) { iq_i = val; iq_dirty = 1; }
        else if (strcmp(key, "DP") == 0) { id_p = val; id_dirty = 1; }
        else if (strcmp(key, "DI") == 0) { id_i = val; id_dirty = 1; }
        else if (strcmp(key, "S")  == 0) { set_speed = val; speed_cmd_dirty = 1; }
        else if (strcmp(key, "SP") == 0) { spd_p = val; spd_dirty = 1; spd_manual_dirty = 1; }
        else if (strcmp(key, "SI") == 0) { spd_i = val; spd_dirty = 1; }
        else if (strcmp(key, "SPL") == 0) { spd_p_low = val; spd_schedule_dirty = 1; }
        else if (strcmp(key, "SPH") == 0) { spd_p_high = val; spd_schedule_dirty = 1; }
        else if (strcmp(key, "SG") == 0) { spd_schedule = (val != 0.0f); spd_schedule_dirty = 1; spd_schedule_explicit = 1; }
        else if (strcmp(key, "PS") == 0) { set_position = val; position_cmd_dirty = 1; }
        else if (strcmp(key, "PR") == 0) { position_delta = val; position_relative_dirty = 1; }
        else if (strcmp(key, "PP") == 0) { pos_p = val; pos_dirty = 1; }
        else if (strcmp(key, "PAL") == 0) { pos_accel = val; pos_accel_dirty = 1; }
        else if (strcmp(key, "CVL") == 0) { current_voltage_limit = val; }
        else if (strcmp(key, "SIL") == 0) { speed_current_limit = val; }
        else if (strcmp(key, "PSL") == 0 || strcmp(key, "PL") == 0) {
            pos_limit = val;
        }
        else if (strcmp(key, "M") == 0) {
            new_mode = (uint8_t)val;
            mode_dirty = 1;
        }
    }

    if (spd_manual_dirty && !spd_schedule_explicit) {
        spd_schedule = 0U;
        spd_schedule_dirty = 1U;
    }

    if (position_delta > POS_RELATIVE_STEP_MAX_RAD) {
        position_delta = POS_RELATIVE_STEP_MAX_RAD;
    }
    if (position_delta < -POS_RELATIVE_STEP_MAX_RAD) {
        position_delta = -POS_RELATIVE_STEP_MAX_RAD;
    }

    if (!(pos_accel > 0.0f)) {
        pos_accel = motor_config.pos_accel_limit;
    }
    if (pos_accel > POS_ACCEL_LIMIT_MAX) {
        pos_accel = POS_ACCEL_LIMIT_MAX;
    }

    encoder_cache_t encoder = {0};
    if (motor_ready && ((mode_dirty && new_mode == MOTOR_POSITION) ||
                        position_relative_dirty)) {
        (void)AS5047P_EncoderCache_Read(&encoder);
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    motor_control.status_flag = 1;

    if (iq_dirty) {
        motor_config.iq_p_gain = iq_p;
        motor_config.iq_i_gain = iq_i;
        foc_set_current_pid(iq_p, iq_i,
                            current_loop.D, current_loop.output_ramp);
    }
    if (id_dirty) {
        motor_config.id_p_gain = id_p;
        motor_config.id_i_gain = id_i;
        foc_set_id_current_pid(id_p, id_i,
                               id_current_loop.D, id_current_loop.output_ramp);
    }
    if (spd_dirty) {
        motor_config.spd_p_gain = spd_p;
        motor_config.spd_i_gain = spd_i;
    }
    if (spd_schedule_dirty) {
        motor_config.spd_p_low_speed = spd_p_low;
        motor_config.spd_p_high_speed = spd_p_high;
        motor_config.spd_gain_schedule = spd_schedule;
    }
    if (pos_dirty) {
        motor_config.pos_p_gain = pos_p;
    }
    if (pos_accel_dirty) {
        motor_config.pos_accel_limit = pos_accel;
    }

    /* Commit all motor commands together so token order cannot change meaning. */
    if (motor_ready) {
        if (mode_dirty && new_mode <= MOTOR_POSITION) {
            motor_control.mode = new_mode;
            if (new_mode == MOTOR_SPEED) {
                motor_control.set_speed = speed_cmd_dirty ? set_speed : 0.0f;
                motor_control.set_torque = 0.0f;
                motor_control.vel_filter_state = 0.0f;
                motor_control.vel_raw = 0.0f;
                motor_control.vel_meas = 0.0f;
                motor_control.spd_needs_init = 1;
            } else if (new_mode == MOTOR_POSITION) {
                if (position_cmd_dirty) {
                    motor_control.set_position = set_position;
                } else if (position_relative_dirty) {
                    motor_control.set_position = encoder.total_angle_rad + position_delta;
                } else {
                    motor_control.set_position = encoder.total_angle_rad;
                }
                motor_control.pos_meas = encoder.total_angle_rad;
                motor_control.set_speed = 0.0f;
                motor_control.set_torque = 0.0f;
                motor_control.vel_filter_state = 0.0f;
                motor_control.vel_raw = 0.0f;
                motor_control.vel_meas = 0.0f;
                motor_control.spd_needs_init = 1;
            } else {
                motor_control.set_speed = 0.0f;
                motor_control.set_torque = torque_dirty ? set_torque : 0.0f;
            }
        } else {
            if (torque_dirty) {
                motor_control.set_torque = set_torque;
            }
            if (speed_cmd_dirty) {
                motor_control.set_speed = set_speed;
            }
            if (position_cmd_dirty) {
                motor_control.set_position = set_position;
            } else if (position_relative_dirty && motor_control.mode == MOTOR_POSITION) {
                motor_control.set_position = encoder.total_angle_rad + position_delta;
            }
        }
        if (id_cmd_dirty) {
            motor_control.id_target = id_target;
        }
    }

    foc_set_loop_limits(current_voltage_limit, speed_current_limit, pos_limit);

    __set_PRIMASK(primask);
}
/* ========================================================================== */
/*  RX: 公共 API — 在主循环中轮询                                              */
/* ========================================================================== */

/**
  * @brief  检查是否有接收到的命令帧并处理
  *         在 main() 的 while 循环中调用（~10-100 Hz）
  */
void VOFA_ProcessCmd(void)
{
    /* ── 如果有完整命令帧就绪，则解析 ── */
    if (rx_ready) {
        rx_ready = 0;
        vofa_parse_cmd(rx_line);
    }

    /* ── 每次循环迭代保持 DMA 接收活跃 ──
     *     正常路径: RxState == READY → 重启成功
     *     如果上次重启失败（瞬时 BUSY），下一次循环迭代会自动重试 */
    if (huart2.RxState == HAL_UART_STATE_READY) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, VOFA_RX_BUF_SIZE);
    }
}
