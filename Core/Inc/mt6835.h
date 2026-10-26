#ifndef MT6835_H
#define MT6835_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MT6835_INVALID_ANGLE UINT32_MAX
#define MT6835_BURST_SIZE 6U
typedef enum {
    MT6835_DIAG_OK = 0, MT6835_DIAG_UNCHECKED, MT6835_DIAG_SPI,
    MT6835_DIAG_NO_DATA, MT6835_DIAG_CRC, MT6835_DIAG_OVERSPEED,
    MT6835_DIAG_MAG_LOW, MT6835_DIAG_UNDERVOLTAGE, MT6835_DIAG_UNSTABLE,
    MT6835_DIAG_POWER_CYCLE
} mt6835_diagnostic_status_t;
typedef struct {
    uint32_t angle;
    uint32_t valid, crc_errors, status_errors, spi_errors, no_sample;
    HAL_StatusTypeDef spi_status;
    mt6835_diagnostic_status_t cause;
    uint8_t status, crc_received, crc_expected, attempts;
} mt6835_diagnostics_t;

void MT6835_Init(void);
uint8_t MT6835_CheckDiagnostics(void);
uint8_t MT6835_DiagnosticsHealthy(void);
void MT6835_GetDiagnostics(mt6835_diagnostics_t *snapshot);
const char *MT6835_DiagnosticStatusName(mt6835_diagnostic_status_t cause);
/* CRC-8, polynomial 0x07, initial 0, MSB first over registers 0x003..0x005. */
uint8_t MT6835_Crc8(const uint8_t data[3]);
HAL_StatusTypeDef MT6835_DMA_StartRequest(void);
uint32_t MT6835_DMA_GetAngleCallback(uint32_t *sample_cycle);
uint8_t MT6835_DMA_IsBusy(void);
HAL_StatusTypeDef MT6835_DMA_Reset(void);
uint8_t MT6835_DMA_ExpectedGap(void);
HAL_StatusTypeDef MT6835_ConfigureAutocal(void);
void MT6835_RequestCalStatus(void);
void MT6835_HoldSamplingISR(void);

#ifdef __cplusplus
}
#endif
#endif
