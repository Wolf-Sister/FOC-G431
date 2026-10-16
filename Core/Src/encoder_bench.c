#include "encoder_bench.h"
#if APP_UART_ENABLE
#include "as5047p.h"
#include "as5047p_ext.h"
#include "foc.h"
#include "motor_supervisor.h"
#include "tim.h"
#include "uart_tx.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BENCH_RECORDS 256U
#define BENCH_BINS 129U
#define BENCH_BIN_WIDTH 0.05f
#define BENCH_SECONDS 5U
enum { ACTION_NONE, ACTION_BEGIN, ACTION_REGS, ACTION_A, ACTION_B, ACTION_RESTORE, ACTION_END };
enum { CAPTURE_IDLE, CAPTURE_WAIT, CAPTURE_RUNNING, CAPTURE_DONE };
typedef struct { uint32_t cycle; uint16_t raw; int16_t delta; float velocity; } bench_record_t;
static struct {
    volatile uint8_t state;
    uint8_t aborted, agcMinimum, agcMaximum;
    uint32_t diagnosticSamples;
    uint16_t previousRaw, settings1, write, retained;
    uint32_t n, firstCycle, lastCycle;
    int32_t position, minimum, maximum;
    float angleMean, angleM2, velocityMean, velocityM2, velocityPeak;
    uint32_t histogram[BENCH_BINS];
    as5047p_transfer_counts_t firstCounts, lastCounts;
    bench_record_t records[BENCH_RECORDS];
} capture;
static volatile uint8_t bench_active;
static uint8_t action, saved_valid, ending, dump_stage, dump_records, sampling;
static uint16_t registers[4], saved_settings1, dump_index;
static uint32_t wait_started, capture_started_ms;
static char reply_line[240];

bool EncoderBench_IsActive(void) { return bench_active != 0U; }

static void bench_reply(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    (void)vsnprintf(reply_line, sizeof(reply_line), format, args);
    va_end(args);
}

static bool bench_disabled(void)
{
    return !current_loop_enable && !alignment_in_progress && !foc_stage2_driver_enabled() &&
           HAL_GPIO_ReadPin(DRV_EN_GPIO_Port, DRV_EN_Pin) == GPIO_PIN_RESET;
}

static void bench_finish_capture(uint8_t aborted)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    capture.aborted = aborted;
    AS5047P_GetTransferCounts(&capture.lastCounts);
    __DMB();
    capture.state = CAPTURE_DONE;
    __set_PRIMASK(key);
}

static bool bench_read_registers(void)
{
    static const uint16_t addresses[4] = { AS5047P_REG_SETTINGS1, AS5047P_REG_SETTINGS2,
                                         AS5047P_REG_DIAAGC, AS5047P_REG_MAG };
    for (uint8_t i = 0U; i < 4U; i++) {
        uint16_t frame = AS5047P_ReadRegister(addresses[i]);
        if (frame == 0xFFFFU) return false;
        registers[i] = frame & 0x3FFFU;
        if (i == 0U && action == ACTION_BEGIN && !saved_valid) {
            saved_settings1 = registers[0]; saved_valid = 1U;
        }
    }
    return (registers[2] & (AS5047P_DIAAGC_LF | AS5047P_DIAAGC_COF |
                           AS5047P_DIAAGC_MAGL | AS5047P_DIAAGC_MAGH)) == AS5047P_DIAAGC_LF &&
           registers[3] != 0U;
}

static void bench_status(const char *label)
{
    bench_reply("[ENC] %s fw=%08lX can=%u auto=%u active=%u action=%u capture=%u fault=%u current=%u s1=%04X s2=%04X dia=%04X mag=%04X saved=%04X/%u n=%lu\r\n",
                label, (unsigned long)APP_FIRMWARE_VERSION, APP_CAN_ENABLE, APP_AUTO_CALIBRATE,
                bench_active, action, capture.state, motor_fault_reason, motor_current.Calibrated, registers[0], registers[1],
                registers[2], registers[3], saved_settings1, saved_valid, (unsigned long)capture.n);
}

bool EncoderBench_Command(const char *line)
{
    uint8_t requested = ACTION_NONE;
    if (strncmp(line, "ENC=", 4U) != 0) return false;
    if (reply_line[0]) {
        /* Commands are stop-and-wait; never mutate state without reply capacity. */
        (void)UART_TX_PutString("[ENC] ERR busy_reply\r\n");
        return true;
    }
    if (strcmp(line, "ENC=STATUS") == 0) { bench_status("STATUS cached"); return true; }
    if (strcmp(line, "ENC=HELP") == 0) {
        bench_reply("[ENC] HELP BEGIN REGS A B RESTORE START STOP RESULT DUMP STATUS END; use ENC=<command>, CR/LF; START=5s disabled capture\r\n");
        return true;
    }
    if (action || ending || dump_stage) { bench_reply("[ENC] ERR busy\r\n"); return true; }
    if (strcmp(line, "ENC=STOP") == 0) {
        if (capture.state != CAPTURE_WAIT && capture.state != CAPTURE_RUNNING)
            bench_reply("[ENC] ERR no_capture\r\n");
        else { bench_finish_capture(1U); bench_reply("[ENC] OK STOP aborted=1\r\n"); }
        return true;
    }
    if (strcmp(line, "ENC=BEGIN") == 0) {
        uint32_t key = __get_PRIMASK();
        __disable_irq();
        if (bench_active || !bench_disabled() || !motor_current.Calibrated ||
            motor_fault_reason != MOTOR_FAULT_NONE) {
            __set_PRIMASK(key);
            bench_reply("[ENC] ERR BEGIN requires calibrated disabled fault-free motor\r\n");
            return true;
        }
        /* Latch first, then invalidate pending CAN/ADC enable commands atomically. */
        bench_active = 1U;
        MotorSupervisor_StopISR();
        motor_electrical_calibrated = 0U;
        __set_PRIMASK(key);
        saved_valid = 0U; requested = ACTION_BEGIN;
    } else {
        if (!bench_active || !bench_disabled()) {
            bench_reply("[ENC] ERR requires BEGIN and disabled motor\r\n"); return true;
        }
        if (capture.state == CAPTURE_WAIT || capture.state == CAPTURE_RUNNING) {
            bench_reply("[ENC] ERR capture_busy; use STOP\r\n"); return true;
        }
        if (strcmp(line, "ENC=START") == 0) {
            if (!saved_valid || !sampling || (registers[0] & AS5047P_SETTINGS1_NOISESET) ||
                !AS5047P_DiagnosticsHealthy() || motor_fault_reason != MOTOR_FAULT_NONE) {
                bench_reply("[ENC] ERR configuration\r\n"); return true;
            }
            /* IDLE/DONE has no ISR writer. Publish WAIT only after clearing RAM. */
            capture.state = CAPTURE_IDLE; __DMB();
            memset(&capture, 0, sizeof(capture));
            capture.agcMinimum = 255U;
            capture.settings1 = registers[0];
            capture_started_ms = HAL_GetTick();
            __DMB(); capture.state = CAPTURE_WAIT;
            bench_reply("[ENC] OK START seconds=5 s1=%04X\r\n", capture.settings1);
            return true;
        }
        if (strcmp(line, "ENC=RESULT") == 0 || strcmp(line, "ENC=DUMP") == 0) {
            if (capture.state != CAPTURE_DONE || !capture.n) {
                bench_reply("[ENC] ERR no_result\r\n"); return true;
            }
            dump_stage = 1U; dump_index = 0U;
            dump_records = strcmp(line, "ENC=DUMP") == 0;
            return true;
        }
        if (strcmp(line, "ENC=REGS") == 0) requested = ACTION_REGS;
        else if (strcmp(line, "ENC=A") == 0) requested = ACTION_A;
        else if (strcmp(line, "ENC=B") == 0) requested = ACTION_B;
        else if (strcmp(line, "ENC=RESTORE") == 0) requested = ACTION_RESTORE;
        else if (strcmp(line, "ENC=END") == 0) requested = ACTION_END;
        else { bench_reply("[ENC] ERR unknown_command\r\n"); return true; }
    }
    if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK) {
        bench_reply("[ENC] ERR timer_stop; bench remains active\r\n"); return true;
    }
    sampling = 0U;
    AS5047P_Sensor_Resync(&AngleSensor);
    capture.state = CAPTURE_IDLE;
    action = requested; wait_started = HAL_GetTick();
    return true;
}

void EncoderBench_OnDiagnostic(uint16_t frame)
{
    if (capture.state == CAPTURE_RUNNING && AS5047P_CheckParity(frame) && !(frame & 0x4000U)) {
        uint8_t agc = (uint8_t)frame;
        capture.diagnosticSamples++;
        if (agc < capture.agcMinimum) capture.agcMinimum = agc;
        if (agc > capture.agcMaximum) capture.agcMaximum = agc;
    }
}

void EncoderBench_OnSample(uint16_t raw, uint32_t cycle, float velocity, uint8_t windowCount)
{
    int32_t delta;
    float difference;
    uint32_t bin;
    if (!bench_active || (capture.state != CAPTURE_WAIT && capture.state != CAPTURE_RUNNING)) return;
    if (capture.state == CAPTURE_WAIT) {
        if (windowCount < AS5047P_VELOCITY_WINDOW_SIZE) return;
        capture.firstCycle = cycle; capture.previousRaw = raw;
        AS5047P_GetTransferCounts(&capture.firstCounts);
        capture.state = CAPTURE_RUNNING;
    }
    if ((uint32_t)(cycle - capture.firstCycle) >= SystemCoreClock * BENCH_SECONDS) {
        capture.lastCycle = cycle;
        bench_finish_capture(0U); return;
    }
    if (!isfinite(velocity)) { bench_finish_capture(1U); return; }
    delta = (int32_t)raw - capture.previousRaw;
    if (delta > 8192) delta -= 16384;
    else if (delta < -8192) delta += 16384;
    capture.position += delta; capture.previousRaw = raw; capture.lastCycle = cycle;
    if (capture.position < capture.minimum) capture.minimum = capture.position;
    if (capture.position > capture.maximum) capture.maximum = capture.position;
    capture.n++;
    difference = (float)capture.position - capture.angleMean;
    capture.angleMean += difference / (float)capture.n;
    capture.angleM2 += difference * ((float)capture.position - capture.angleMean);
    difference = velocity - capture.velocityMean;
    capture.velocityMean += difference / (float)capture.n;
    capture.velocityM2 += difference * (velocity - capture.velocityMean);
    if (fabsf(velocity) > capture.velocityPeak) capture.velocityPeak = fabsf(velocity);
    bin = fabsf(velocity) >= 6.4f ? BENCH_BINS - 1U : (uint32_t)(fabsf(velocity) / BENCH_BIN_WIDTH);
    capture.histogram[bin]++;
    capture.records[capture.write].cycle = cycle;
    capture.records[capture.write].raw = raw;
    capture.records[capture.write].delta = (int16_t)delta;
    capture.records[capture.write].velocity = velocity;
    capture.write = (uint16_t)((capture.write + 1U) % BENCH_RECORDS);
    if (capture.retained < BENCH_RECORDS) capture.retained++;
}

static void bench_export(void)
{
    char line[240];
    int length = 0;
    if (UART_TX_Pending() > UART_TX_RING_SIZE - sizeof(line)) return;
    if (dump_stage == 1U) {
        length = snprintf(line, sizeof(line), "[ENC] RESULT s1=%04X n=%lu seconds=%.6f aborted=%u retained=%u clock=%lu agc_n=%lu agc_min=%u agc_max=%u\r\n",
            capture.settings1, (unsigned long)capture.n,
            (double)(capture.lastCycle - capture.firstCycle) / SystemCoreClock,
            capture.aborted, capture.retained, (unsigned long)SystemCoreClock,
            (unsigned long)capture.diagnosticSamples, capture.agcMinimum, capture.agcMaximum);
    } else if (dump_stage == 2U) {
        uint32_t cumulative = 0U, percentile = (capture.n * 99U + 99U) / 100U, bin;
        for (bin = 0U; bin < BENCH_BINS; bin++) {
            cumulative += capture.histogram[bin];
            if (cumulative >= percentile) break;
        }
        length = snprintf(line, sizeof(line), "[ENC] STATS angle_std_counts=%.4f angle_pp_counts=%ld vmean=%.5f vrms=%.5f vpeak=%.5f vp99_bound=%.3f p99_overflow=%u\r\n",
            (double)sqrtf(fmaxf(0.0f, capture.angleM2 / capture.n)),
            (long)(capture.maximum - capture.minimum), (double)capture.velocityMean,
            (double)sqrtf(fmaxf(0.0f, capture.velocityM2 / capture.n + capture.velocityMean * capture.velocityMean)),
            (double)capture.velocityPeak, (double)(bin < BENCH_BINS - 1U ? (bin + 1U) * BENCH_BIN_WIDTH : 6.4f),
            bin >= BENCH_BINS - 1U);
    } else if (dump_stage == 3U) {
        const as5047p_transfer_counts_t *a = &capture.firstCounts, *b = &capture.lastCounts;
        length = snprintf(line, sizeof(line), "[ENC] COUNTS angle=%lu diagnostic=%lu priming=%lu no_sample=%lu parity=%lu ef=%lu spi=%lu unexpected=%lu\r\n",
            (unsigned long)(b->angle-a->angle), (unsigned long)(b->diagnostic-a->diagnostic),
            (unsigned long)(b->priming-a->priming), (unsigned long)(b->no_sample-a->no_sample),
            (unsigned long)(b->parity-a->parity), (unsigned long)(b->ef-a->ef),
            (unsigned long)(b->spi-a->spi), (unsigned long)(b->unexpected-a->unexpected));
    } else if (dump_stage == 4U) {
        length = snprintf(line, sizeof(line), "[ENC] CSV index,cycle,raw,delta,velocity_rad_s\r\n");
    } else if (dump_stage == 5U && dump_index < capture.retained) {
        uint16_t index = (uint16_t)((capture.write + BENCH_RECORDS - capture.retained + dump_index) % BENCH_RECORDS);
        const bench_record_t *r = &capture.records[index];
        length = snprintf(line, sizeof(line), "[ENC] ROW %u,%lu,%u,%d,%.6f\r\n", dump_index,
            (unsigned long)r->cycle, r->raw, (int)r->delta, (double)r->velocity);
    } else {
        length = snprintf(line, sizeof(line), "[ENC] END_RESULT rows=%u tx_dropped=%lu\r\n",
                          dump_records ? capture.retained : 0U, (unsigned long)UART_TX_Dropped());
    }
    if (length <= 0 || (size_t)length >= sizeof(line)) {
        dump_stage = 0U; bench_reply("[ENC] ERR export_format\r\n"); return;
    }
    if (!UART_TX_Put(line, (uint16_t)length)) return;
    if (dump_stage == 5U && dump_index < capture.retained) dump_index++;
    else if (dump_stage >= 5U) dump_stage = 0U;
    else if (dump_stage == 3U && !dump_records) dump_stage = 6U;
    else dump_stage++;
}

void EncoderBench_Poll(void)
{
    if (reply_line[0]) {
        size_t length = strlen(reply_line);
        if (UART_TX_Pending() + length <= UART_TX_RING_SIZE && UART_TX_Put(reply_line, (uint16_t)length))
            reply_line[0] = '\0';
        return;
    }
    if (action) {
        uint8_t completed = action;
        if (AS5047P_DMA_IsBusy()) {
            if ((uint32_t)(HAL_GetTick() - wait_started) >= 5U) {
                action = ACTION_NONE;
                bench_reply("[ENC] ERR dma_timeout; timer stopped, bench remains active\r\n");
            }
            return;
        }
        if (!saved_valid && action != ACTION_BEGIN && action != ACTION_END) {
            action = ACTION_NONE; bench_reply("[ENC] ERR missing_original; use END\r\n"); return;
        }
        if (action == ACTION_A || action == ACTION_B || action == ACTION_RESTORE || action == ACTION_END) {
            uint16_t value = saved_settings1;
            if (action == ACTION_A || action == ACTION_B) {
                value &= (uint16_t)~(AS5047P_SETTINGS1_NOISESET | AS5047P_SETTINGS1_DAECDIS | AS5047P_SETTINGS1_DATASELECT);
                if (action == ACTION_B) value |= AS5047P_SETTINGS1_DATASELECT;
            }
            if (saved_valid && AS5047P_WriteSettings1Verified(value) != HAL_OK) {
                action = ACTION_NONE; bench_reply("[ENC] ERR write_verify; timer stopped, bench remains active\r\n"); return;
            }
        }
        if (!bench_read_registers() || (registers[0] & AS5047P_SETTINGS1_NOISESET)) {
            action = ACTION_NONE; bench_status("ERR registers_or_field_or_NOISESET"); return;
        }
        action = ACTION_NONE;
        if (AS5047P_DMA_ResetPipeline() != HAL_OK) { bench_reply("[ENC] ERR pipeline\r\n"); return; }
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { bench_reply("[ENC] ERR timer_start\r\n"); return; }
        sampling = 1U;
        if (completed == ACTION_END) { ending = 1U; wait_started = HAL_GetTick(); }
        else bench_status("OK configured");
        return;
    }
    if (ending) {
        encoder_cache_t snapshot = {0};
        uint32_t key = __get_PRIMASK();
        bool ready;
        __disable_irq();
        ready = AngleSensor.velocity_window_count >= AS5047P_VELOCITY_WINDOW_SIZE &&
                AS5047P_EncoderCache_Read(&snapshot) && !snapshot.stale &&
                AS5047P_DiagnosticsHealthy() && motor_fault_reason == MOTOR_FAULT_NONE;
        if (ready) {
            MotorSupervisor_StopISR();
            motor_electrical_calibrated = 0U;
            bench_active = 0U; ending = saved_valid = 0U;
        }
        __set_PRIMASK(key);
        if (ready) bench_reply("[ENC] OK END restored; disabled, electrical recalibration required\r\n");
        else if ((uint32_t)(HAL_GetTick() - wait_started) >= 50U) {
            ending = 0U; bench_reply("[ENC] ERR warmup; bench remains active\r\n");
        }
        return;
    }
    if ((capture.state == CAPTURE_WAIT || capture.state == CAPTURE_RUNNING) &&
        ((uint32_t)(HAL_GetTick() - capture_started_ms) >= 5200U || !bench_disabled() ||
         motor_fault_reason != MOTOR_FAULT_NONE || !AS5047P_DiagnosticsHealthy())) {
        bench_finish_capture(1U);
        bench_reply("[ENC] ERR capture_aborted; query RESULT\r\n");
    }
    if (dump_stage) bench_export();
}
#endif
