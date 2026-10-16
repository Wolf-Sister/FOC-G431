#ifndef MOTOR_ALIGNMENT_H
#define MOTOR_ALIGNMENT_H
#include "motor_types.h"

typedef enum { ALIGN_IDLE, ALIGN_RAMP_UP, ALIGN_HOLD, ALIGN_RAMP_TO_SAMPLE,
               ALIGN_SETTLE, ALIGN_SAMPLE, ALIGN_RAMP_DOWN, ALIGN_DONE, ALIGN_FAILED } motor_align_phase_t;
typedef struct {
    uint8_t phase, samples, overcurrentTicks, failure;
    uint32_t startedMs, phaseMs, lastSampleMs, lastEncoderSequence;
    float sumSin, sumCos, zero, voltage;
} motor_alignment_t;
void MotorAlignment_Start(motor_alignment_t *alignment, uint32_t now);
void MotorAlignment_Step(motor_alignment_t *alignment, const motor_snapshot_t *sample,
                         uint32_t now, int direction, int polePairs);
void MotorAlignment_Abort(motor_alignment_t *alignment);
#endif
