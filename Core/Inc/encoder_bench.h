#ifndef ENCODER_BENCH_H
#define ENCODER_BENCH_H
#include "main.h"
#include <stdbool.h>

#if APP_UART_ENABLE
/* UART2 commands and SPI maintenance run in main context only. */
bool EncoderBench_Command(const char *line);
void EncoderBench_Poll(void);
bool EncoderBench_IsActive(void);
/* Called by the sole TIM2 producer after accepting a fresh angle. */
void EncoderBench_OnSample(uint16_t raw, uint32_t cycle, float velocity,
                           uint8_t windowCount);
void EncoderBench_OnDiagnostic(uint16_t frame);
#else
static inline bool EncoderBench_Command(const char *line) { (void)line; return false; }
static inline void EncoderBench_Poll(void) {}
static inline bool EncoderBench_IsActive(void) { return false; }
static inline void EncoderBench_OnSample(uint16_t raw, uint32_t cycle, float velocity,
                                        uint8_t windowCount)
{ (void)raw; (void)cycle; (void)velocity; (void)windowCount; }
static inline void EncoderBench_OnDiagnostic(uint16_t frame) { (void)frame; }
#endif
#endif
