/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ JustFloat TX + text-command RX via UART2 DMA
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
static uint8_t  rx_buf[VOFA_RX_BUF_SIZE];        /* DMA NORMAL target       */
static char     rx_line[VOFA_RX_BUF_SIZE];        /* assembled command line   */
static volatile uint8_t rx_ready = 0;             /* flag: complete cmd ready */

/* ========================================================================== */
/*  TX: Send telemetry                                                        */
/* ========================================================================== */

void VOFA_SendData(const float *data, uint8_t count)
{
    static char buf[VOFA_TX_BUF_SIZE];  /* static: safe for background DMA */
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
  * @brief  Initialize UART2 DMA reception (NORMAL mode + IDLE detection).
  *         CubeMX configured the DMA as CIRCULAR; override to NORMAL
  *         because HAL_UARTEx_ReceiveToIdle_DMA relies on DMA stopping
  *         at end-of-frame so the IDLE callback fires cleanly.
  */
void VOFA_InitRx(void)
{
    /* Override CubeMX CIRCULAR → NORMAL for frame-based reception */
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_usart2_rx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, VOFA_RX_BUF_SIZE);
}

/* ========================================================================== */
/*  RX: Weak callback override — called on IDLE / frame received              */
/* ========================================================================== */

/**
  * @brief  Override the default weak HAL_UARTEx_RxEventCallback.
  *         Called by HAL (ISR context) when UART2 goes idle after receiving
  *         data.  Copies frame to line buffer, sets flag, returns quickly.
  *
  *         DMA restart is deferred to VOFA_ProcessCmd() (main-loop context).
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2 && Size > 0 && Size <= VOFA_RX_BUF_SIZE) {
        /* Copy DMA data to line buffer (fast, from ISR) */
        memcpy(rx_line, rx_buf, Size);
        rx_line[Size] = '\0';
        rx_ready = 1;
        /* DMA restart is deferred to VOFA_ProcessCmd() in main loop */
    }
}

/* ========================================================================== */
/*  RX: Text command parser                                                    */
/* ========================================================================== */

/**
  * @brief  Parse text-based motor commands, comma-separated.
  *
  * Format (any combination, any order):
  *   T=V      set torque / Iq current command (A)
  *   D=V      set D-axis current target (A), default 0 for SPM
  *   P=V      set Iq-loop P gain
  *   I=V      set Iq-loop I gain
  *   DP=V     set Id-loop P gain
  *   DI=V     set Id-loop I gain
  *   S=V      set speed setpoint (mechanical rad/s)
  *   SP=V     set speed-loop P gain
  *   SI=V     set speed-loop I gain
  *   PS=V     set position setpoint (multi-turn rad)
  *   PP=V     set position-loop P gain (rad/s per rad)
  *   PL=V     set position-loop speed limit (rad/s)
  *   M=V      set control mode (0=torque, 1=speed, 2=position)
  *
  * Any received command sets status_flag=1 for step-sync in Python.
  *
  * Examples:
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
    float iq_p = motor_config.iq_p_gain;  /* start from current value, */
    float iq_i = motor_config.iq_i_gain;  /* only overwrite what's sent */
    float id_p = motor_config.id_p_gain;
    float id_i = motor_config.id_i_gain;
    float spd_p = motor_config.spd_p_gain;
    float spd_i = motor_config.spd_i_gain;
    float pos_p = motor_config.pos_p_gain;
    float pos_limit = motor_config.pos_speed_limit;

    while (*p) {
        /* Skip whitespace / commas */
        while (*p == ' ' || *p == ',') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r') break;

        /* Read key: T, D, P, I, DP, DI, S, SP, SI, PS, PP, PL, M */
        char key[4] = {0};
        int ki = 0;
        while (*p && *p != '=' && *p != ',' && *p != ' ' && *p != '\n' && *p != '\r' && ki < 3) {
            key[ki++] = *p++;
        }
        if (*p != '=') { p++; continue; }  /* skip malformed token */
        p++;  /* skip '=' */

        /* Read value */
        char *end;
        float val = strtof(p, &end);
        if (end == p) break;  /* no conversion */
        p = end;

        /* Dispatch */
        if      (strcmp(key, "T")  == 0) { motor_control.set_torque = val; }
        else if (strcmp(key, "D")  == 0) { motor_control.id_target  = val; }
        else if (strcmp(key, "P")  == 0) { iq_p = val; iq_dirty = 1; }
        else if (strcmp(key, "I")  == 0) { iq_i = val; iq_dirty = 1; }
        else if (strcmp(key, "DP") == 0) { id_p = val; id_dirty = 1; }
        else if (strcmp(key, "DI") == 0) { id_i = val; id_dirty = 1; }
        else if (strcmp(key, "S")  == 0) { motor_control.set_speed = val; }
        else if (strcmp(key, "SP") == 0) { spd_p = val; spd_dirty = 1; }
        else if (strcmp(key, "SI") == 0) { spd_i = val; spd_dirty = 1; }
        else if (strcmp(key, "PS") == 0) { motor_control.set_position = val; }
        else if (strcmp(key, "PP") == 0) { pos_p = val; pos_dirty = 1; }
        else if (strcmp(key, "PL") == 0) { pos_limit = val; pos_dirty = 1; }
        else if (strcmp(key, "M")  == 0) {
            uint8_t new_mode = (uint8_t)val;
            if (new_mode <= 2) {
                motor_control.mode = new_mode;
                if (new_mode == MOTOR_SPEED) {
                    motor_control.set_speed         = 0.0f;
                    motor_control.vel_filter_state  = 0.0f;
                    motor_control.vel_meas          = 0.0f;
                    motor_control.spd_needs_init    = 1;
                } else if (new_mode == MOTOR_POSITION) {
                    motor_control.set_position   = encoder_cache.total_angle_rad;
                    motor_control.pos_meas       = encoder_cache.total_angle_rad;
                    motor_control.set_speed       = 0.0f;
                    motor_control.vel_filter_state = 0.0f;
                    motor_control.vel_meas         = 0.0f;
                    motor_control.spd_needs_init   = 1;
                } else { /* MOTOR_TORQUE */
                    motor_control.set_speed  = 0.0f;
                    motor_control.set_torque = 0.0f;
                }
            }
        }

        /* unknown key → silently ignored */
    }

    /* Any received command triggers step-sync flag for Python analysis */
    motor_control.status_flag = 1;

    /* Apply PID changes only if at least one gain was set */
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
        speed_loop.P = spd_p;
        speed_loop.I = spd_i;
    }
    if (pos_dirty) {
        motor_config.pos_p_gain      = pos_p;
        motor_config.pos_speed_limit = pos_limit;
    }
}

/* ========================================================================== */
/*  RX: Public API — poll in main loop                                        */
/* ========================================================================== */

/**
  * @brief  Check for a received command frame and process it.
  *         Call from main() while loop at ~10–100 Hz.
  */
void VOFA_ProcessCmd(void)
{
    /* ── Parse command if a complete frame is ready ── */
    if (rx_ready) {
        rx_ready = 0;
        vofa_parse_cmd(rx_line);
    }

    /* ── Keep DMA reception alive every loop iteration ──
     *     Normal path: RxState == READY → restart succeeds.
     *     If a prior restart failed (transient BUSY), the next
     *     loop iteration retries automatically. */
    if (huart2.RxState == HAL_UART_STATE_READY) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, VOFA_RX_BUF_SIZE);
    }
}
