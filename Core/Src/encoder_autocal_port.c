#include "encoder_autocal.h"
#if APP_ENCODER_TYPE == APP_ENCODER_MT6835
#include "encoder_autocal_port.h"
#include "encoder.h"
#include "encoder_bench.h"
#include "motor_port.h"
#include "motor_supervisor.h"
#include "motor_calibration.h"
#include "tim.h"

#define EAC_COOKIE 0x45414331UL
/* Own TAMP backup registers 14/15. No RTC/LSE clock is required for backup
 * data access; APB clock and DBP write access are enabled explicitly. */
bool EncoderAutoCalPort_Init(void)
{
    uint32_t budget=100000U;
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    PWR->CR1 |= PWR_CR1_DBP;
    while (!(PWR->CR1 & PWR_CR1_DBP) && budget) budget--;
    return budget!=0U;
}
bool EncoderAutoCalPort_Cookie(void)
{
    /* These two registers belong exclusively to this feature. Partial writes
     * or damaged markers must also quarantine sampling until a cold boot. */
    return TAMP->BKP14R!=0U || TAMP->BKP15R!=0U;
}
bool EncoderAutoCalPort_Mark(void)
{
    TAMP->BKP15R=~EAC_COOKIE;
    TAMP->BKP14R=EAC_COOKIE;
    __DSB();
    return TAMP->BKP14R==EAC_COOKIE && TAMP->BKP15R==~EAC_COOKIE;
}
void EncoderAutoCalPort_Clear(void)
{
    TAMP->BKP14R=0U; TAMP->BKP15R=0U; __DSB();
}
void EncoderAutoCalPort_Pin(bool high)
{
    HAL_GPIO_WritePin(MT6835_CAL_EN_GPIO_Port,MT6835_CAL_EN_Pin,high?GPIO_PIN_SET:GPIO_PIN_RESET);
}
uint8_t EncoderAutoCalPort_Configure(void)
{
    uint32_t cr,dier,key=MotorPort_Lock();
    HAL_StatusTypeDef result;
    if (MT6835_DMA_IsBusy()) { MotorPort_Unlock(key); return 1U; }
    cr=TIM2->CR1; dier=TIM2->DIER;
    TIM2->DIER &= ~TIM_DIER_UIE; TIM2->CR1 &= ~TIM_CR1_CEN;
    __HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE); HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    MotorPort_Unlock(key);
    result=MT6835_ConfigureAutocal();
    (void)MT6835_DMA_Reset();
    Encoder_Sensor_Resync(&AngleSensor);
    key=MotorPort_Lock();
    __HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE); HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    TIM2->DIER=dier; TIM2->CR1=cr;
    MotorPort_Unlock(key);
    return result==HAL_OK?0U:2U;
}
void EncoderAutoCalPort_Input(encoder_autocal_input_t *in)
{
    motor_health_t h;
    encoder_cache_t encoder={0};
    CAN_NodeStatus status={0};
    bool gotSample;
    bool encoderHealthy;
    uint32_t key;
    uint32_t cyclesPerMs=SystemCoreClock/1000U;
    MotorPort_ReadHealth(&h);
    MotorSupervisor_ReadStatus(&status,HAL_GetTick());
    /* Use the same full-rate encoder cache as FOC. The downsampled ADC status
     * snapshot and millisecond timestamp are not a second encoder watchdog. */
    /* Copy once for both speed and health. Block TIM2/SPI publication only;
     * priority-0 ADC/current protection can still preempt this short read. */
    key=__get_BASEPRI();
    __set_BASEPRI(0x10U);
    gotSample=Encoder_Cache_Read(&encoder);
    encoderHealthy=Encoder_DiagnosticsHealthy() && !EncoderBench_IsActive();
    __set_BASEPRI(key);
    in->sampleAgeMs=gotSample && cyclesPerMs ? encoder.age_cycles/cyclesPerMs : UINT32_MAX;
    in->healthFaults=(!gotSample ? EAC_HEALTH_NO_SAMPLE:0U) |
        (gotSample && !encoder.data_valid ? EAC_HEALTH_SAMPLE_INVALID:0U) |
        (gotSample && encoder.stale ? EAC_HEALTH_SAMPLE_STALE:0U) |
        (!encoderHealthy ? EAC_HEALTH_ENCODER:0U) |
        (!h.currentCalibrated ? EAC_HEALTH_CURRENT:0U) |
        (!h.busValid ? EAC_HEALTH_BUS:0U) |
        (h.driverFault ? EAC_HEALTH_DRIVER:0U) |
        (status.faultBits ? EAC_HEALTH_FAULT:0U);
    in->healthy=in->healthFaults==0U;
    in->enabled=h.outputEnabled;
    in->electrical=h.electricalCalibrated;
    in->saved=(MotorCalibration_Status()&CAL_STATUS_SAVED)!=0U;
    in->role=status.role; in->mode=status.mode;
    in->velocity=encoder.velocity_rad_s;
    in->counts=(int64_t)encoder.turn_count*16384+encoder.raw_angle;
}
#endif
