#include "motor_alignment.h"
#include "foc.h"
#include <math.h>
#include <string.h>

void MotorAlignment_Start(motor_alignment_t *a, uint32_t now)
{
    memset(a, 0, sizeof(*a));
    a->phase = ALIGN_RAMP_UP;
    a->startedMs = a->phaseMs = now;
}

void MotorAlignment_Abort(motor_alignment_t *a)
{
    a->phase = ALIGN_IDLE;
    a->voltage = 0.0f;
}

static void alignment_fail(motor_alignment_t *a, uint8_t reason)
{
    a->failure = reason;
    a->phase = ALIGN_FAILED;
    a->voltage = 0.0f;
}

void MotorAlignment_Step(motor_alignment_t *a, const motor_snapshot_t *sample,
                         uint32_t now, int direction, int polePairs)
{
    uint32_t elapsed = now - a->phaseMs;
    if (a->phase == ALIGN_IDLE || a->phase >= ALIGN_DONE) return;
    if ((uint32_t)(now - a->startedMs) >= FOC_ALIGN_TOTAL_TIMEOUT_MS) {
        alignment_fail(a, MOTOR_FAULT_ALIGN_TIMEOUT); return;
    }
    if (!sample->valid || (uint32_t)(now - sample->capturedMs) >= 2U) {
        alignment_fail(a, MOTOR_FAULT_ENCODER); return;
    }
    if (sample->maxPhaseCurrent > FOC_ALIGN_CURRENT_LIMIT_A) {
        if (++a->overcurrentTicks >= FOC_ALIGN_OVERCURRENT_MS) {
            alignment_fail(a, MOTOR_FAULT_OVERCURRENT); return;
        }
    } else a->overcurrentTicks = 0U;
    switch (a->phase) {
    case ALIGN_RAMP_UP:
        a->voltage = FOC_ALIGN_VOLTAGE_V * fminf((float)elapsed / FOC_ALIGN_RAMP_UP_MS, 1.0f);
        if (elapsed >= FOC_ALIGN_RAMP_UP_MS) { a->phase = ALIGN_HOLD; a->phaseMs = now; }
        break;
    case ALIGN_HOLD:
        a->voltage = FOC_ALIGN_VOLTAGE_V;
        if (elapsed >= FOC_ALIGN_HOLD_MS) { a->phase = ALIGN_RAMP_TO_SAMPLE; a->phaseMs = now; }
        break;
    case ALIGN_RAMP_TO_SAMPLE:
        a->voltage = FOC_ALIGN_VOLTAGE_V + (FOC_ALIGN_HOLD_VOLTAGE_V - FOC_ALIGN_VOLTAGE_V) *
                     fminf((float)elapsed / FOC_ALIGN_RAMP_DOWN_MS, 1.0f);
        if (elapsed >= FOC_ALIGN_RAMP_DOWN_MS) { a->phase = ALIGN_SETTLE; a->phaseMs = now; }
        break;
    case ALIGN_SETTLE:
        a->voltage = FOC_ALIGN_HOLD_VOLTAGE_V;
        if (elapsed >= 120U) { a->phase = ALIGN_SAMPLE; a->phaseMs = now; }
        break;
    case ALIGN_SAMPLE:
        if (elapsed >= FOC_ALIGN_SAMPLE_TIMEOUT_MS) { alignment_fail(a, MOTOR_FAULT_ALIGN_TIMEOUT); break; }
        if (sample->encoderSequence != a->lastEncoderSequence &&
            (a->samples == 0U || (uint32_t)(now - a->lastSampleMs) >= 1U)) {
            float angle = (float)sample->rawAngle * (_2PI / 16384.0f);
            a->sumSin += sinf(angle); a->sumCos += cosf(angle);
            a->lastEncoderSequence = sample->encoderSequence; a->lastSampleMs = now;
            if (++a->samples >= 10U) {
                if (a->sumSin * a->sumSin + a->sumCos * a->sumCos < 100.0f *
                    FOC_ALIGN_MIN_RESULTANT * FOC_ALIGN_MIN_RESULTANT) {
                    alignment_fail(a, MOTOR_FAULT_ALIGN_UNSTABLE); break;
                }
                a->zero = fmodf((float)(direction * polePairs) * atan2f(a->sumSin, a->sumCos), _2PI);
                if (a->zero < 0.0f) a->zero += _2PI;
                a->phase = ALIGN_RAMP_DOWN; a->phaseMs = now;
            }
        }
        break;
    case ALIGN_RAMP_DOWN:
        a->voltage = FOC_ALIGN_HOLD_VOLTAGE_V * (1.0f - fminf((float)elapsed / FOC_ALIGN_RAMP_DOWN_MS, 1.0f));
        if (elapsed >= FOC_ALIGN_RAMP_DOWN_MS) { a->voltage = 0.0f; a->phase = ALIGN_DONE; }
        break;
    default: alignment_fail(a, MOTOR_FAULT_ALIGN_TIMEOUT); break;
    }
}
