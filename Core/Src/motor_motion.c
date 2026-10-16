#include "motor_motion.h"
#include <math.h>
#define MOTION_RAD_PER_COUNT (6.283185307179586476925286766559 / 16384.0)
int64_t MotorMotion_Counts(int32_t turns, uint16_t raw) { return (int64_t)turns * 16384 + raw; }
bool MotorMotion_ConfigValid(const motor_motion_config_t *c, uint8_t mode)
{
    if (mode == 0U) return true;
    if (mode > 2U || !c->speedLimit || !c->acceleration) return false;
    return mode == 1U || (c->positionMin < c->positionMax && c->positionSpeed && c->positionSpeed <= c->speedLimit);
}
bool MotorMotion_TargetValid(const motor_motion_config_t *c, uint8_t mode, int32_t v, uint32_t iqLimit)
{
    if (!MotorMotion_ConfigValid(c, mode)) return false;
    if (!mode) return v <= (int32_t)iqLimit && v >= -(int32_t)iqLimit;
    if (mode == 1U) return v <= (int32_t)c->speedLimit && v >= -(int32_t)c->speedLimit;
    return v >= c->positionMin && v <= c->positionMax;
}
bool MotorMotion_InRange(const motor_motion_config_t *c, int64_t counts)
{
    double p = (double)counts * MOTION_RAD_PER_COUNT;
    return p >= (double)c->positionMin * 0.0001 && p <= (double)c->positionMax * 0.0001;
}
void MotorMotion_SetTarget(motor_motion_t *s, int32_t target)
{
    s->target = target;
    if (s->mode == 2U) {
        double radians = (double)target * 0.0001;
        s->targetCounts = (int64_t)(radians / MOTION_RAD_PER_COUNT);
        s->targetRemainder = (float)(radians - (double)s->targetCounts * MOTION_RAD_PER_COUNT);
    }
}
float MotorMotion_Error(int64_t counts, int32_t position)
{
    motor_motion_t s = {0}; s.mode = 2U; MotorMotion_SetTarget(&s, position);
    return (float)(s.targetCounts - counts) * (float)MOTION_RAD_PER_COUNT + s.targetRemainder;
}
bool MotorMotion_Step(motor_motion_t *s, int64_t counts, float velocity, float kp, float *speed)
{
    s->failure = MOTION_FAILURE_NONE;
    if (!MotorMotion_ConfigValid(&s->config, s->mode) || !s->mode) {
        s->failure = MOTION_FAILURE_CONFIG; return false;
    }
    if (!isfinite(velocity) || !isfinite(s->speed)) {
        s->failure = MOTION_FAILURE_NONFINITE; return false;
    }
    if (fabsf(velocity) > (float)s->config.speedLimit * 0.001f) {
        s->failure = MOTION_FAILURE_OVERSPEED; return false;
    }
    float desired = (float)s->target * 0.001f;
    float accel = (float)s->config.acceleration * 0.001f;
    if (s->mode == 2U) {
        if (!MotorMotion_InRange(&s->config, counts)) {
            s->failure = MOTION_FAILURE_POSITION; return false;
        }
        if (!isfinite(kp)) { s->failure = MOTION_FAILURE_NONFINITE; return false; }
        if (kp <= 0.0f) { s->failure = MOTION_FAILURE_CONFIG; return false; }
        float error = (float)(s->targetCounts - counts) * (float)MOTION_RAD_PER_COUNT + s->targetRemainder;
        float ceiling = fminf((float)s->config.positionSpeed * 0.001f, sqrtf(2.0f * accel * fabsf(error)));
        desired = fmaxf(-ceiling, fminf(ceiling, error * kp));
    }
    float delta = accel * MOTOR_MOTION_LOOP_DT_S;
    s->speed += fmaxf(-delta, fminf(delta, desired - s->speed));
    *speed = s->speed;
    if (!isfinite(*speed)) { s->failure = MOTION_FAILURE_NONFINITE; return false; }
    return true;
}
