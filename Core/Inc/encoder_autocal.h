#ifndef ENCODER_AUTOCAL_H
#define ENCODER_AUTOCAL_H
#include "app_config.h"
#include <stdbool.h>
#include <stdint.h>

#define CAN_CAP_ENCODER_AUTOCAL (1UL << 10)
#define CAN_OP_ENCODER_AUTOCAL 0x1EU
#define ENCODER_AUTOCAL_PREPARE 0xCA10U
#define ENCODER_AUTOCAL_START 0xCA11U
#define ENCODER_AUTOCAL_TARGET_MRAD_S 4190U /* 40 RPM, AUTOCAL_FREQ=7 */
enum {
    EAC_IDLE, EAC_PREPARING, EAC_PREPARED, EAC_WAIT_SPEED,
    EAC_RUNNING, EAC_SAVING, EAC_POWER_CYCLE, EAC_FAILED
};
enum {
    EAC_ERR_NONE, EAC_ERR_PREPARE, EAC_ERR_HEALTH, EAC_ERR_SPEED,
    EAC_ERR_TIMEOUT, EAC_ERR_CHIP, EAC_ERR_CANCELLED, EAC_ERR_BACKUP
};

#if APP_ENCODER_TYPE == APP_ENCODER_MT6835
void EncoderAutoCal_Init(bool coldBoot);
bool EncoderAutoCal_Supported(void);
bool EncoderAutoCal_OutputBlocked(void);
bool EncoderAutoCal_BlocksActions(void);
bool EncoderAutoCal_BlocksReset(void);
bool EncoderAutoCal_BootHeld(void);
bool EncoderAutoCal_BootProbeNeeded(void);
bool EncoderAutoCal_ConfirmBootStatus(uint8_t status);
uint8_t EncoderAutoCal_Prepare(uint32_t now);
uint8_t EncoderAutoCal_Start(uint32_t now);
void EncoderAutoCal_Poll(uint32_t now);
void EncoderAutoCal_OnStatusISR(uint8_t status, uint32_t now);
bool EncoderAutoCal_Read(uint16_t selector, uint32_t *value);
#else
static inline void EncoderAutoCal_Init(bool cold) { (void)cold; }
static inline bool EncoderAutoCal_Supported(void) { return false; }
static inline bool EncoderAutoCal_OutputBlocked(void) { return false; }
static inline bool EncoderAutoCal_BlocksActions(void) { return false; }
static inline bool EncoderAutoCal_BlocksReset(void) { return false; }
static inline bool EncoderAutoCal_BootHeld(void) { return false; }
static inline bool EncoderAutoCal_BootProbeNeeded(void) { return false; }
static inline bool EncoderAutoCal_ConfirmBootStatus(uint8_t s) { (void)s; return true; }
static inline uint8_t EncoderAutoCal_Prepare(uint32_t t) { (void)t; return 11U; }
static inline uint8_t EncoderAutoCal_Start(uint32_t t) { (void)t; return 11U; }
static inline void EncoderAutoCal_Poll(uint32_t t) { (void)t; }
static inline void EncoderAutoCal_OnStatusISR(uint8_t s,uint32_t t) { (void)s;(void)t; }
static inline bool EncoderAutoCal_Read(uint16_t s,uint32_t *v) { (void)s;(void)v;return false; }
#endif
#endif
