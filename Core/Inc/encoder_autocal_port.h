#ifndef ENCODER_AUTOCAL_PORT_H
#define ENCODER_AUTOCAL_PORT_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    bool healthy, enabled, electrical, saved;
    uint8_t role, mode;
    float velocity;
    int64_t counts;
    uint32_t healthFaults, sampleAgeMs;
} encoder_autocal_input_t;
enum {
    EAC_HEALTH_NO_SAMPLE=1U, EAC_HEALTH_SAMPLE_INVALID=2U,
    EAC_HEALTH_SAMPLE_STALE=4U, EAC_HEALTH_ENCODER=8U,
    EAC_HEALTH_CURRENT=16U, EAC_HEALTH_BUS=32U,
    EAC_HEALTH_DRIVER=64U, EAC_HEALTH_FAULT=128U,
    EAC_HEALTH_SAVED=256U, EAC_HEALTH_ROLE=512U,
    EAC_HEALTH_MODE=1024U, EAC_HEALTH_VELOCITY=2048U,
    EAC_HEALTH_OUTPUT_OFF=4096U, EAC_HEALTH_ELECTRICAL=8192U,
    EAC_HEALTH_OUTPUT_ON=16384U, EAC_HEALTH_TRIGGER_SPEED=32768U
};
bool EncoderAutoCalPort_Init(void);
bool EncoderAutoCalPort_Cookie(void);
bool EncoderAutoCalPort_Mark(void);
void EncoderAutoCalPort_Clear(void);
void EncoderAutoCalPort_Pin(bool high);
/* 0=success, 1=DMA not idle yet, 2=failed; caller owns disabled maintenance. */
uint8_t EncoderAutoCalPort_Configure(void);
void EncoderAutoCalPort_Input(encoder_autocal_input_t *input);
#endif
