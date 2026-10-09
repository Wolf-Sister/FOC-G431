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
#include <math.h>
#include <errno.h>
#include "uart_tx.h"

volatile uint32_t vofa_rejected_frames = 0U;
volatile uint32_t vofa_rx_dropped = 0U;
volatile uint32_t vofa_tx_dropped = 0U;

/* ── RX state ──────────────────────────────────────────────────────────── */
#if APP_UART_ENABLE
static uint8_t  rx_buf[VOFA_RX_BUF_SIZE];        /* DMA NORMAL 模式目标缓冲区  */
static char     rx_line[VOFA_RX_BUF_SIZE + 1U];        /* 组装后的命令字符串         */
#define VOFA_RX_QUEUE_SIZE 4U
static char rx_queue[VOFA_RX_QUEUE_SIZE][VOFA_RX_BUF_SIZE + 1U];
static uint16_t rx_length[VOFA_RX_QUEUE_SIZE];
static volatile uint8_t rx_write = 0U, rx_read = 0U;
static uint16_t rx_line_length = 0U;
static uint8_t rx_discard_line = 0U;
static volatile uint8_t rx_error_pending = 0U;
#if APP_COMMAND_TIMEOUT_MS
static uint32_t command_last_ms;
static uint8_t command_lease_started = 0U;
#endif

/* ========================================================================== */
/*  TX: Send telemetry                                                        */
/* ========================================================================== */

void VOFA_SendData(const float *data, uint8_t count)
{
    static char buf[UART_TX_CHUNK_MAX + 1U];
    size_t pos = 0U;

    if (count == 0 || data == NULL) {
        return;
    }
    if (count > VOFA_MAX_CHANNELS) {
        count = VOFA_MAX_CHANNELS;
    }

    memcpy(buf, "channels: ", 10U);
    pos = 10U;
    for (uint8_t i = 0; i < count; i++) {
        int written = snprintf(buf + pos, sizeof(buf) - pos, "%s%.6f",
                               i ? "," : "", (double)data[i]);
        if (written < 0 || (size_t)written >= sizeof(buf) - pos) {
            vofa_tx_dropped++;
            return;
        }
        pos += (size_t)written;
    }
    if (pos >= UART_TX_CHUNK_MAX) {
        vofa_tx_dropped++;
        return;
    }
    buf[pos++] = '\n';
    if (!UART_TX_Put(buf, (uint16_t)pos)) vofa_tx_dropped++;
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
static void vofa_start_rx(void)
{
    if (huart2.RxState == HAL_UART_STATE_READY &&
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, VOFA_RX_BUF_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

void VOFA_InitRx(void)
{
    /* 覆盖 CubeMX 的 CIRCULAR → NORMAL，实现帧级接收 */
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
        Error_Handler();
        return;
    }
    vofa_start_rx();
}

/* ========================================================================== */
/*  RX: Weak callback override — called on IDLE / frame received              */
/* ========================================================================== */

/**
  * @brief  覆盖默认的弱定义 HAL_UARTEx_RxEventCallback
  *         当 UART2 接收数据后进入空闲状态时，由 HAL 在中断上下文中调用。
  *         按 CR/LF 组帧并发布稳定副本，复制完成后立即重启接收。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2 || Size == 0U || Size > VOFA_RX_BUF_SIZE ||
        HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;
    if (rx_error_pending) {
        rx_error_pending = 0U;
        rx_line_length = 0U;
        rx_discard_line = 1U;
    }
    for (uint16_t i = 0U; i < Size; i++) {
        char ch = (char)rx_buf[i];
        /* Treat CRLF as one terminator: LF sees an empty line after CR. */
        if (ch == '\r' || ch == '\n') {
            if (!rx_discard_line && rx_line_length != 0U) {
                if ((uint8_t)(rx_write - rx_read) < VOFA_RX_QUEUE_SIZE) {
                    uint8_t slot = rx_write % VOFA_RX_QUEUE_SIZE;
                    memcpy(rx_queue[slot], rx_line, rx_line_length);
                    rx_queue[slot][rx_line_length] = '\0';
                    rx_length[slot] = rx_line_length;
                    __DMB();
                    rx_write++;
                } else {
                    vofa_rx_dropped++;
                }
            }
            rx_line_length = 0U;
            rx_discard_line = 0U;
        } else if (!rx_discard_line) {
            if (ch == '\0' || rx_line_length >= VOFA_RX_BUF_SIZE) {
                rx_discard_line = 1U;
                vofa_rx_dropped++;
            } else {
                rx_line[rx_line_length++] = ch;
            }
        }
    }
    /* NORMAL DMA has completed/stopped; restart after the copy to reduce gaps. */
    vofa_start_rx();
}

void VOFA_NotifyRxError(void)
{
    rx_error_pending = 1U;
    vofa_rx_dropped++;
}

static void vofa_reject(void)
{
    vofa_rejected_frames++;
    UART2_SendString("[CMD] rejected: invalid frame/range/state\r\n");
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
static void vofa_parse_cmd(const char *line, uint16_t length)
{
    if (!line || !length || length > VOFA_RX_BUF_SIZE ||
        line[length] != '\0' || strlen(line) != length) {
        vofa_reject();
        return;
    }
    const char *p = line;
    uint8_t iq_dirty = 0, id_dirty = 0, spd_dirty = 0, pos_dirty = 0;
    uint8_t torque_dirty = 0, id_cmd_dirty = 0, speed_cmd_dirty = 0;
    uint8_t position_cmd_dirty = 0, position_relative_dirty = 0, mode_dirty = 0;
    uint8_t spd_schedule_dirty = 0, spd_schedule_explicit = 0;
    uint8_t spd_manual_dirty = 0;
    uint8_t pos_accel_dirty = 0;
    uint8_t heartbeat = 0U, parsed_tokens = 0U;
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
        while (*p == ' ' || *p == '\t' || *p == ',') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r') break;

        char key[4] = {0};
        int ki = 0;
        while (*p && *p != '=' && *p != ',' && *p != ' ' &&
               *p != '\n' && *p != '\r' && ki < 3) {
            key[ki++] = *p++;
        }
        if (ki == 0 || *p != '=') { vofa_reject(); return; }
        p++;

        char *end;
        errno = 0;
        float val = strtof(p, &end);
        if (end == p || errno == ERANGE || !isfinite(val) ||
            (*end && *end != ',' && *end != ' ' && *end != '\t')) {
            vofa_reject(); return;
        }
        p = end;
        parsed_tokens = 1U;

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
        else if (strcmp(key, "SG") == 0) {
            if (val != 0.0f && val != 1.0f) { vofa_reject(); return; }
            spd_schedule = (val != 0.0f); spd_schedule_dirty = 1; spd_schedule_explicit = 1;
        }
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
            if (val < MOTOR_TORQUE || val > MOTOR_POSITION || floorf(val) != val) {
                vofa_reject(); return;
            }
            new_mode = (uint8_t)val;
            mode_dirty = 1;
        }
        else if (strcmp(key, "HB") == 0) {
            if (val != 1.0f) { vofa_reject(); return; }
            heartbeat = 1U;
        }
        else { vofa_reject(); return; }
    }

    if (!parsed_tokens) { vofa_reject(); return; }
    if (spd_manual_dirty && !spd_schedule_explicit) {
        spd_schedule = 0U;
        spd_schedule_dirty = 1U;
    }

    /* Explicit command bounds, not a promise of stable tuning at these gains. */
    if (iq_p < 0.0f || iq_p > 100.0f || id_p < 0.0f || id_p > 100.0f ||
        iq_i < 0.0f || iq_i > 10000.0f || id_i < 0.0f || id_i > 10000.0f ||
        spd_p < 0.0f || spd_p > 100.0f || spd_i < 0.0f || spd_i > 10000.0f ||
        spd_p_low < 0.0f || spd_p_low > 100.0f || spd_p_high < 0.0f || spd_p_high > 100.0f ||
        pos_p < 0.0f || pos_p > 1000.0f || fabsf(set_speed) > POS_SPEED_LIMIT_MAX ||
        fabsf(set_position) > 1000000.0f || fabsf(position_delta) > POS_RELATIVE_STEP_MAX_RAD ||
        pos_accel <= 0.0f || pos_accel > POS_ACCEL_LIMIT_MAX ||
        current_voltage_limit <= 0.0f || current_voltage_limit > CURRENT_VOLTAGE_LIMIT_DEFAULT ||
        speed_current_limit <= 0.0f || speed_current_limit > CURRENT_REF_HARD_MAX_A ||
        pos_limit <= 0.0f || pos_limit > POS_SPEED_LIMIT_MAX ||
        fabsf(set_torque) > CURRENT_REF_HARD_MAX_A || fabsf(id_target) > CURRENT_REF_HARD_MAX_A ||
        ((torque_dirty || id_cmd_dirty) &&
         set_torque * set_torque + id_target * id_target > speed_current_limit * speed_current_limit)) {
        vofa_reject(); return;
    }

    encoder_cache_t encoder = {0};
    if (motor_ready && ((mode_dirty && new_mode != motor_control.mode && new_mode == MOTOR_POSITION) ||
                        position_relative_dirty)) {
        if (!AS5047P_EncoderCache_Read(&encoder) || encoder.stale || !encoder.data_valid) {
            vofa_reject(); return;
        }
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!motor_ready || motor_fault_reason != MOTOR_FAULT_NONE ||
        ((torque_dirty || id_cmd_dirty) &&
         set_torque * set_torque + id_target * id_target > speed_current_limit * speed_current_limit)) {
        __set_PRIMASK(primask);
        vofa_reject(); return;
    }

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
        if (mode_dirty && new_mode != motor_control.mode) {
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
#if APP_COMMAND_TIMEOUT_MS
    if (heartbeat || torque_dirty || id_cmd_dirty || speed_cmd_dirty ||
        position_cmd_dirty || position_relative_dirty || mode_dirty) {
        command_last_ms = HAL_GetTick();
        command_lease_started = 1U;
    }
#else
    (void)heartbeat;
#endif

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
    /* The producer cannot reuse a queued slot until parsing has finished. */
    for (uint8_t handled = 0U; handled < VOFA_RX_QUEUE_SIZE && rx_read != rx_write; handled++) {
        uint8_t slot = rx_read % VOFA_RX_QUEUE_SIZE;
        __DMB();
        vofa_parse_cmd(rx_queue[slot], rx_length[slot]);
        __DMB();
        rx_read++;
    }

    /* ── 每次循环迭代保持 DMA 接收活跃 ──
     *     正常路径: RxState == READY → 重启成功
     *     如果上次重启失败（瞬时 BUSY），下一次循环迭代会自动重试 */
    vofa_start_rx();
}

void VOFA_CheckCommandTimeout(void)
{
#if APP_COMMAND_TIMEOUT_MS
    if (!motor_ready || motor_fault_reason != MOTOR_FAULT_NONE) return;
    uint32_t now = HAL_GetTick();
    if (!command_lease_started) {
        command_last_ms = now;
        command_lease_started = 1U;
    }
    if ((uint32_t)(now - command_last_ms) >= APP_COMMAND_TIMEOUT_MS) {
        motor_fault_trip(MOTOR_FAULT_COMMAND_TIMEOUT);
    }
#endif
}
#else
void VOFA_SendData(const float *data, uint8_t count)
{
    (void)data;
    (void)count;
}
void VOFA_InitRx(void) {}
void VOFA_ProcessCmd(void) {}
void VOFA_NotifyRxError(void) {}
void VOFA_CheckCommandTimeout(void) {}
#endif
