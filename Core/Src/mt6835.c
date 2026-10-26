#include "mt6835.h"
#include "cmsis_compiler.h"
#include "encoder_autocal.h"

#if APP_ENCODER_TYPE == APP_ENCODER_MT6835
extern SPI_HandleTypeDef hspi1;
static uint8_t mt_tx[MT6835_BURST_SIZE] = {0xA0U, 0x03U, 0U, 0U, 0U, 0U};
static uint8_t mt_rx[MT6835_BURST_SIZE];
static volatile uint8_t mt_busy, mt_ready, mt_healthy;
static volatile uint32_t mt_request_cycle, mt_sample_cycle;
static volatile uint8_t mt_hold, mt_status_pending, mt_request_kind, mt_sample_kind;
static uint8_t mt_expected_gap;
static volatile mt6835_diagnostics_t mt_diag;

/* At 170 MHz this bounded loop exceeds the 100 ns CS setup and the
 * half-SCK-period hold requirement, including before a DMA launch failure. */
static void mt_cs_delay(void)
{
    volatile uint32_t i;
    for (i = 0U; i < 20U; i++) { __NOP(); }
}
static void mt_select(void)
{
    mt_request_cycle = DWT->CYCCNT;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    mt_cs_delay();
}
static void mt_deselect(void)
{
    mt_cs_delay();
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    mt_cs_delay();
}
static HAL_StatusTypeDef mt_register(uint16_t address,uint8_t *value,uint8_t write)
{
    uint8_t tx[3],rx[3]={0};
    HAL_StatusTypeDef result;
    if (mt_busy) return HAL_BUSY;
    tx[0]=(uint8_t)((write?0x60U:0x30U)|((address>>8)&15U));
    tx[1]=(uint8_t)address; tx[2]=write?*value:0U;
    mt_select();
    result=HAL_SPI_TransmitReceive(&hspi1,tx,rx,3U,2U);
    mt_deselect();
    if (result!=HAL_OK) return result;
    /* Ordinary register writes leave MISO Hi-Z (datasheet 7.6.5).
     * 0x55 acknowledges other commands, not this one. The caller must
     * verify a write using a separate register readback. */
    if (write) return HAL_OK;
    *value=rx[2]; return HAL_OK;
}
HAL_StatusTypeDef MT6835_ConfigureAutocal(void)
{
    uint8_t current,desired,readback,status;
    if (mt_hold || mt_busy) return HAL_BUSY;
    if (mt_register(0x113U,&status,0U)!=HAL_OK || status==255U || (status&0xC0U)) return HAL_ERROR;
    if (mt_register(0x00EU,&current,0U)!=HAL_OK) return HAL_ERROR;
    /* AUTOCAL_FREQ occupies bits[6:4]; preserve GPIO_DS and factory bits. */
    desired=(uint8_t)((current&0x8FU)|0x70U);
    if (desired!=current && mt_register(0x00EU,&desired,1U)!=HAL_OK) return HAL_ERROR;
    if (mt_register(0x00EU,&readback,0U)!=HAL_OK || readback!=desired) return HAL_ERROR;
    return HAL_OK;
}
void MT6835_RequestCalStatus(void) { if (!mt_hold) mt_status_pending=1U; }
void MT6835_HoldSamplingISR(void) { mt_hold=1U; mt_status_pending=0U; }
uint8_t MT6835_DMA_ExpectedGap(void) { return mt_expected_gap; }
uint8_t MT6835_Crc8(const uint8_t data[3])
{
    uint8_t crc = 0U, i, bit;
    for (i = 0U; i < 3U; i++) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0x07U : 0U));
    }
    return crc;
}
static uint32_t mt_decode(const uint8_t frame[MT6835_BURST_SIZE])
{
    uint32_t angle;
    mt_diag.status = frame[4] & 7U;
    mt_diag.crc_received = frame[5];
    mt_diag.crc_expected = MT6835_Crc8(frame + 2);
    if (frame[2] == 255U && frame[3] == 255U && frame[4] == 255U && frame[5] == 255U) {
        mt_diag.cause = MT6835_DIAG_NO_DATA;
        return MT6835_INVALID_ANGLE;
    }
    if (mt_diag.crc_expected != frame[5]) {
        mt_diag.crc_errors++;
        mt_diag.cause = MT6835_DIAG_CRC;
        return MT6835_INVALID_ANGLE;
    }
    if (mt_diag.status) {
        mt_diag.status_errors++;
        mt_diag.cause = (mt_diag.status & 4U) ? MT6835_DIAG_UNDERVOLTAGE :
                        (mt_diag.status & 2U) ? MT6835_DIAG_MAG_LOW : MT6835_DIAG_OVERSPEED;
        mt_healthy = 0U; /* Valid warning is latched until explicit startup. */
        return MT6835_INVALID_ANGLE;
    }
    angle = ((uint32_t)frame[2] << 13) | ((uint32_t)frame[3] << 5) | ((uint32_t)frame[4] >> 3);
    mt_diag.angle = angle;
    mt_diag.cause = MT6835_DIAG_OK;
    mt_diag.valid++;
    return angle;
}
void MT6835_Init(void)
{
    mt6835_diagnostics_t empty = {0};
    mt_diag = empty;
    mt_diag.cause = MT6835_DIAG_UNCHECKED;
    mt_diag.spi_status = HAL_BUSY;
    mt_busy = mt_ready = mt_healthy = 0U;
    mt_hold=(EncoderAutoCal_BootHeld() || EncoderAutoCal_BootProbeNeeded())?1U:0U;
    mt_status_pending=mt_request_kind=mt_sample_kind=mt_expected_gap=0U;
    mt_request_cycle = mt_sample_cycle = 0U;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(100U); /* Datasheet power-up is 64 ms typical. No EEPROM writes. */
}
uint8_t MT6835_CheckDiagnostics(void)
{
    uint8_t attempt, consecutive = 0U;
    if (!mt_hold || EncoderAutoCal_BootProbeNeeded()) {
        uint8_t status;
        if (mt_register(0x113U,&status,0U)!=HAL_OK || !EncoderAutoCal_ConfirmBootStatus(status)) {
            mt_diag.cause=MT6835_DIAG_POWER_CYCLE; return 0U;
        }
        mt_hold=0U;
    }
    if (mt_hold) { mt_diag.cause=MT6835_DIAG_POWER_CYCLE; return 0U; }
    if (mt_busy) return 0U;
    mt_healthy = 0U;
    for (attempt = 0U; attempt < 10U; attempt++) {
        HAL_StatusTypeDef result;
        mt_select();
        result = HAL_SPI_TransmitReceive(&hspi1, mt_tx, mt_rx, MT6835_BURST_SIZE, 2U);
        mt_deselect();
        mt_diag.spi_status = result;
        mt_diag.attempts = attempt + 1U;
        if (result == HAL_OK && mt_decode(mt_rx) != MT6835_INVALID_ANGLE) {
            if (++consecutive >= 2U) { mt_healthy = 1U; break; }
        } else {
            consecutive = 0U;
            if (result != HAL_OK) { mt_diag.spi_errors++; mt_diag.cause = MT6835_DIAG_SPI; }
        }
        if (attempt + 1U < 10U) HAL_Delay(2U);
    }
    if (!mt_healthy && mt_diag.cause == MT6835_DIAG_OK) mt_diag.cause = MT6835_DIAG_UNSTABLE;
    return mt_healthy;
}
HAL_StatusTypeDef MT6835_DMA_StartRequest(void)
{
    uint32_t key = __get_BASEPRI();
    HAL_StatusTypeDef result = HAL_BUSY;
    __set_BASEPRI(0x10U);
    if (!mt_hold && !mt_busy && !mt_ready) {
        mt_request_kind=mt_status_pending;
        mt_status_pending=0U;
        mt_tx[0]=mt_request_kind?0x31U:0xA0U;
        mt_tx[1]=mt_request_kind?0x13U:0x03U;
        mt_busy = 1U;
        mt_select();
        result = HAL_SPI_TransmitReceive_DMA(&hspi1, mt_tx, mt_rx,mt_request_kind?3U:MT6835_BURST_SIZE);
        mt_diag.spi_status = result;
        if (result != HAL_OK) {
            mt_deselect(); mt_busy = 0U;
            mt_diag.spi_errors++; mt_diag.cause = MT6835_DIAG_SPI;
        }
    }
    __set_BASEPRI(key);
    return result;
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1 && mt_busy) {
        mt_deselect();
        mt_sample_cycle = mt_request_cycle;
        mt_sample_kind=mt_request_kind;
        if (mt_sample_kind) EncoderAutoCal_OnStatusISR(mt_rx[2],HAL_GetTick());
        mt_ready = 1U;
        mt_busy = 0U;
    }
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        mt_deselect();
        mt_diag.spi_errors++; mt_diag.cause = MT6835_DIAG_SPI;
        mt_diag.spi_status = HAL_ERROR;
        mt_busy = mt_ready = 0U;
    }
}
uint32_t MT6835_DMA_GetAngleCallback(uint32_t *cycle)
{
    uint8_t frame[MT6835_BURST_SIZE], i;
    uint32_t key, captured, angle;
    if (cycle) *cycle = 0U;
    mt_expected_gap=mt_hold;
    key = __get_BASEPRI();
    __set_BASEPRI(0x10U);
    if (!mt_ready) {
        mt_diag.no_sample++;
        __set_BASEPRI(key);
        return MT6835_INVALID_ANGLE;
    }
    for (i = 0U; i < MT6835_BURST_SIZE; i++) frame[i] = mt_rx[i];
    captured = mt_sample_cycle;
    mt_ready = 0U;
    if (mt_sample_kind) {
        mt_expected_gap=1U;
        __set_BASEPRI(key);
        return MT6835_INVALID_ANGLE;
    }
    __set_BASEPRI(key);
    angle = mt_decode(frame);
    if (angle != MT6835_INVALID_ANGLE && cycle) *cycle = captured;
    return angle;
}
uint8_t MT6835_DMA_IsBusy(void) { return mt_busy; }
uint8_t MT6835_DiagnosticsHealthy(void) { return mt_healthy; }
HAL_StatusTypeDef MT6835_DMA_Reset(void)
{
    if (mt_busy) return HAL_BUSY;
    mt_ready = 0U;
    mt_status_pending=mt_request_kind=mt_sample_kind=mt_expected_gap=0U;
    mt_request_cycle = mt_sample_cycle = 0U;
    return HAL_OK;
}
void MT6835_GetDiagnostics(mt6835_diagnostics_t *snapshot)
{
    if (snapshot) {
        uint32_t key = __get_PRIMASK();
        __disable_irq();
        *snapshot = mt_diag;
        __set_PRIMASK(key);
    }
}
const char *MT6835_DiagnosticStatusName(mt6835_diagnostic_status_t cause)
{
    switch (cause) {
    case MT6835_DIAG_OK: return "OK";
    case MT6835_DIAG_SPI: return "SPI";
    case MT6835_DIAG_NO_DATA: return "NO_DATA";
    case MT6835_DIAG_CRC: return "CRC";
    case MT6835_DIAG_OVERSPEED: return "OVERSPEED";
    case MT6835_DIAG_MAG_LOW: return "MAG_LOW";
    case MT6835_DIAG_UNDERVOLTAGE: return "UNDERVOLTAGE";
    case MT6835_DIAG_UNSTABLE: return "UNSTABLE";
    case MT6835_DIAG_POWER_CYCLE: return "POWER_CYCLE_REQUIRED";
    default: return "UNCHECKED";
    }
}
#endif
