#ifndef ENCODER_H
#define ENCODER_H
#include "main.h"
#include "as5047p_ext.h"

/* Preserve the existing estimator/cache ABI names for AS5047P clients. Both
 * devices feed this same estimator in native counts and publish wire raw14. */
#define Encoder_Sensor_Init AS5047P_Sensor_Init
#define Encoder_Sensor_Resync AS5047P_Sensor_Resync
#define Encoder_Sensor_Update AS5047P_Sensor_Update
#define Encoder_Cache_Publish AS5047P_EncoderCache_Publish
#define Encoder_Cache_Read AS5047P_EncoderCache_Read
#define Encoder_GetAngle AS5047P_GetAngle
#define ENCODER_STALE_LIMIT_S AS5047P_STALE_LIMIT_S

#if APP_ENCODER_TYPE == APP_ENCODER_MT6835
#include "mt6835.h"
#define Encoder_Init MT6835_Init
#define Encoder_CheckDiagnostics MT6835_CheckDiagnostics
#define Encoder_DiagnosticsHealthy MT6835_DiagnosticsHealthy
#define Encoder_DMA_StartRequest MT6835_DMA_StartRequest
/* This installation's MT6835 angle runs opposite to the original AS5047P.
 * Normalize once, before the shared velocity/turn estimator and all consumers.
 * Negation modulo one revolution preserves zero and the invalid sentinel. */
static inline uint32_t Encoder_DMA_Read(uint32_t *cycle)
{
    uint32_t raw = MT6835_DMA_GetAngleCallback(cycle);
    return raw == MT6835_INVALID_ANGLE ? raw :
           ((ENCODER_NATIVE_COUNTS - raw) & (ENCODER_NATIVE_COUNTS - 1UL));
}
#define Encoder_DMA_IsBusy MT6835_DMA_IsBusy
#define Encoder_DMA_ExpectedGap MT6835_DMA_ExpectedGap
#else
#include "as5047p.h"
#define Encoder_Init AS5047P_Init
#define Encoder_CheckDiagnostics AS5047P_CheckDiagnostics
#define Encoder_DiagnosticsHealthy AS5047P_DiagnosticsHealthy
#define Encoder_DMA_StartRequest AS5047P_DMA_StartRequest
#define Encoder_DMA_IsBusy AS5047P_DMA_IsBusy
#define Encoder_DMA_ExpectedGap AS5047P_DMA_ExpectedGap
static inline uint32_t Encoder_DMA_Read(uint32_t *cycle)
{
    uint16_t raw = AS5047P_DMA_GetAngleCallback(cycle);
    return raw == 0xFFFFU ? UINT32_MAX : (uint32_t)raw;
}
#endif
#endif
