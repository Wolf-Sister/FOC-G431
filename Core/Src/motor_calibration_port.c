#include "adc.h"
#include "app_config.h"
#include "as5047p.h"
#include "can.h"
#include "fdcan.h"
#include "foc.h"
#include "ina219.h"
#include "motor_calibration.h"
#include "motor_port.h"
#include "motor_supervisor.h"
#include "tim.h"
#include <string.h>

#if !defined(STM32G431xx) || FLASH_PAGE_SIZE != 0x800U
#error "Calibration layout requires STM32G431 category 2, 2 KiB pages"
#endif
static uint32_t calTimer1, calTimer2, calDier1, calDier2;
static bool calPaused, calResumeFailed;
static volatile uint32_t calReadPage, calReadActive, calReadFailed, calEccSlots;
static bool cal_capacity(void) { return *(volatile uint16_t *)FLASHSIZE_BASE == 128U; }
static bool cal_range(uint32_t address, uint32_t n)
{
    return cal_capacity() && address >= CAL_SLOT_A && address < CAL_SLOT_B + CAL_PAGE_SIZE &&
           n <= CAL_SLOT_B + CAL_PAGE_SIZE - address;
}
static bool cal_read_ecc(uint32_t address)
{
    return address >= CAL_SLOT_A && address < CAL_SLOT_B + CAL_PAGE_SIZE &&
           (calEccSlots & (address < CAL_SLOT_B ? 1U : 2U)) != 0U;
}
__attribute__((noinline)) static uint32_t cal_load_word(uint32_t address)
{
    return *(volatile uint32_t *)address;
}
static bool cal_read(uint32_t address, uint8_t *bytes, uint32_t n)
{
    uint32_t key, offset, word;
    if (!bytes || !n || (address & 3U) || (n & 3U) || !cal_range(address, n) ||
        n > CAL_PAGE_SIZE - (address & (CAL_PAGE_SIZE - 1U)) ||
        __get_IPSR() != 0U || (MotorSupervisor_Active() && !calPaused) || cal_read_ecc(address))
        return false;
    key = MotorPort_Lock();
    Motor_HardwareDisable();
    calReadPage = address & ~(CAL_PAGE_SIZE - 1U);
    calReadFailed = 0U;
    __DMB();
    calReadActive = 1U;
    for (offset = 0U; offset < n; offset += 4U)
    {
        /* ECCC freezes ADDR_ECC too. Clear it before each load so a later ECCD
         * can be attributed. Never clear a pending ECCD here. */
        FLASH->ECCR = (FLASH->ECCR & FLASH_ECCR_ECCIE) | FLASH_ECCR_ECCC;
        __DSB();
        word = cal_load_word(address + offset);
        __DSB();
        __ISB();
        if (calReadFailed)
            break; /* Never pass an uncorrectable word to the record decoder. */
        memcpy(bytes + offset, &word, 4U);
    }
    __DSB();
    calReadActive = 0U;
    MotorPort_Unlock(key);
    return calReadFailed == 0U;
}
bool MotorCalibrationPort_HandleNmi(void)
{
    uint32_t ecc = FLASH->ECCR;
    uint32_t address = FLASH_BASE + (ecc & FLASH_ECCR_ADDR_ECC);
    /* A simultaneous ECCC makes ADDR_ECC ambiguous. Clock/system/program-area
     * errors and all unguarded accesses retain the fatal NMI shutdown. */
    if (!calReadActive || !__get_PRIMASK() || !(ecc & FLASH_ECCR_ECCD) ||
        (ecc & (FLASH_ECCR_SYSF_ECC | FLASH_ECCR_ECCC)) || (RCC->CIFR & RCC_CIFR_CSSF) ||
        (calReadPage != CAL_SLOT_A && calReadPage != CAL_SLOT_B) || (address & 7U) ||
        address < calReadPage || address >= calReadPage + CAL_PAGE_SIZE)
        return false;
    Motor_HardwareDisable();
    calReadFailed = 1U;
    calEccSlots |= calReadPage == CAL_SLOT_A ? 1U : 2U;
    __DMB();
    FLASH->ECCR = (ecc & FLASH_ECCR_ECCIE) | FLASH_ECCR_ECCD | FLASH_ECCR_ECCC;
    __DSB();
    return true;
}
/* Entire busy interval executes from SRAM, including literals. No HAL calls,
 * library calls, flash constants or SysTick timeouts are used here. The scatter
 * file copies this section at startup. Call only with PRIMASK set and output off.
 * If BSY fails to clear, reset directly from SRAM instead of fetching flash. */
__attribute__((section(".ram_flash"), noinline)) static uint32_t
cal_flash_command(uint32_t address, uint32_t lo, uint32_t hi, uint32_t erase, uint32_t budget)
{
    uint32_t start = DWT->CYCCNT, error;
    FLASH->SR = FLASH_FLAG_SR_ERRORS | FLASH_SR_EOP;
    if (erase)
    {
        FLASH->CR = (FLASH->CR & ~(FLASH_CR_PNB | FLASH_CR_PG)) | FLASH_CR_PER |
                    (((address - 0x08000000U) / 0x800U) << FLASH_CR_PNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
    }
    else
    {
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)address = lo;
        __ISB();
        *(volatile uint32_t *)(address + 4U) = hi;
    }
    __DSB();
    while (FLASH->SR & FLASH_SR_BSY)
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= budget)
        {
            SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
            __DSB();
            for (;;)
                __NOP();
        }
    }
    error = FLASH->SR & FLASH_FLAG_SR_ERRORS;
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PG | FLASH_CR_PNB);
    return error;
}
static bool cal_execute(uint32_t address, uint32_t lo, uint32_t hi, bool erase)
{
    uint32_t key, start, elapsed, error, cache;
    if (!calPaused || calResumeFailed || !MotorCalibration_CanWrite() || !cal_range(address, 8U) ||
        (address & 7U) || (erase && address != CAL_SLOT_A && address != CAL_SLOT_B) ||
        !(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) || (FLASH->SR & FLASH_SR_BSY))
        return false;
    Motor_HardwareDisable();
    if (HAL_FLASH_Unlock() != HAL_OK)
        return false;
    key = MotorPort_Lock();
    if (!MotorCalibration_CanWrite())
    {
        (void)HAL_FLASH_Lock();
        MotorPort_Unlock(key);
        return false;
    }
    start = DWT->CYCCNT;
    cache = FLASH->ACR & (FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    error = cal_flash_command(address, lo, hi, erase ? 1U : 0U, SystemCoreClock / 10U);
    /* Invalidate flash caches before readback of a freshly erased/programmed slot. */
    FLASH->ACR |= FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH->ACR &= ~(FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    FLASH->ACR |= cache;
    (void)HAL_FLASH_Lock();
    elapsed = (uint32_t)(DWT->CYCCNT - start) / (SystemCoreClock / 1000U);
    while (elapsed--)
        HAL_IncTick();
    /* SysTick can coalesce during the masked interval; discard the stale pend. */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    MotorPort_Unlock(key);
    return error == 0U;
}
static bool cal_erase(uint32_t address)
{
    if (!cal_execute(address, 0, 0, true))
        return false;
    calEccSlots &= ~(address == CAL_SLOT_A ? 1U : 2U);
    return true;
}
static bool cal_program(uint32_t address, const uint8_t *bytes)
{
    uint32_t words[2];
    memcpy(words, bytes, 8);
    return cal_execute(address, words[0], words[1], false);
}
const cal_io_t MotorCalibration_IO = {cal_read, cal_erase, cal_program, cal_read_ecc};
bool MotorCalibrationPort_Init(cal_binding_t *b)
{
    int direction, pairs;
    calPaused = calResumeFailed = false;
    if (!cal_capacity())
        return false;
    memset(b, 0, sizeof(*b));
    MotorPort_GetElectricalConfig(&direction, &pairs);
    b->revision = APP_ELECTRICAL_CAL_CONFIG_REV;
    b->uid[0] = HAL_GetUIDw0();
    b->uid[1] = HAL_GetUIDw1();
    b->uid[2] = HAL_GetUIDw2();
    b->profile = MotorProfile_Selected.profileId;
    b->profileRevision = MotorProfile_Selected.revision;
    b->pairs = (uint16_t)pairs;
    b->counts = 16384U;
    b->direction = (int8_t)direction;
    b->angleMode = APP_ENCODER_RAW_ANGLE ? 1U : 2U;
    b->algorithm = 1U;
    return pairs > 0 && pairs <= 65535 && (direction == 1 || direction == -1);
}
bool MotorCalibrationPort_Quiesce(void)
{
    uint32_t key = MotorPort_Lock();
    if (!MotorCalibration_CanWrite() || !INA219_ServiceStorageReady() || AS5047P_DMA_IsBusy())
    {
        MotorPort_Unlock(key);
        return false;
    }
    Motor_HardwareDisable();
    calTimer1 = TIM1->CR1;
    calTimer2 = TIM2->CR1;
    calDier1 = TIM1->DIER;
    calDier2 = TIM2->DIER;
    TIM1->DIER &= ~TIM_DIER_UIE;
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    calPaused = true;
    MotorPort_Unlock(key);
    if (HAL_ADCEx_InjectedStop_IT(&hadc1) != HAL_OK || HAL_ADCEx_InjectedStop(&hadc2) != HAL_OK)
    {
        calResumeFailed = true;
        return true; /* Save refuses writes, resume latches fault. */
    }
    return true;
}
bool MotorCalibrationPort_Resume(void)
{
    uint32_t key = MotorPort_Lock();
    bool ok = !calResumeFailed;
    Motor_HardwareDisable();
    AS5047P_Sensor_Resync(&AngleSensor);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_OVR);
    HAL_NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    foc_storage_resume();
    MotorSupervisor_StorageProgress(HAL_GetTick());
    MotorPort_Unlock(key);
    if (ok)
        ok = HAL_ADCEx_InjectedStart(&hadc2) == HAL_OK && HAL_ADCEx_InjectedStart_IT(&hadc1) == HAL_OK;
    if (ok)
    {
        TIM2->DIER = calDier2;
        TIM1->DIER = calDier1;
        TIM2->CR1 = calTimer2;
        TIM1->CR1 = calTimer1;
    }
    INA219_ServiceStorageResume(HAL_GetTick());
    calPaused = false;
    return ok;
}
bool MotorCalibrationPort_TxConfirmed(uint32_t buffer)
{
#if APP_CAN_ENABLE
    return buffer && (hfdcan1.Instance->TXBTO & buffer) && !(hfdcan1.Instance->TXBRP & buffer);
#else
    (void)buffer;
    return false;
#endif
}
void MotorCalibrationPort_Flush(void) { CAN_StorageFlush(); }
