#include "ina219.h"
#include "i2c.h"
#include "foc.h"
#include <math.h>

/* One static transaction buffer, owned until its completion callback.
 * Errors latch this service off; no blocking abort/reset/retry is hidden here. */
enum { INA_PROBE, INA_SET_CAL, INA_TRIGGER, INA_WAIT, INA_BUS,
       INA_CHECK_CONFIG, INA_CHECK_CAL, INA_CURRENT, INA_IDLE, INA_FAILED };
static uint8_t phase, buffer[2];
static volatile uint8_t active;
static volatile uint8_t completion; /* 0=pending, 1=complete, 2=error */
static uint32_t started, conversionStarted, lastSample;
static uint16_t pendingBus;
#define INA_TRIGGER_CONFIG ((INA219_CONFIG_VALUE & ~7U) | 3U)

static void service_fail(void)
{
    /* Peripheral-local cancellation: static storage is never reused after failure. */
    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI |
                               I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI);
    __HAL_I2C_DISABLE(&hi2c1);
    active = 0U; phase = INA_FAILED;
    ina219_bus_valid = ina219_current_valid = 0U;
    ina219_ibus = NAN;
    motor_fault_trip(MOTOR_FAULT_BUS);
}

static void transaction(uint8_t reg, uint16_t value, uint8_t write, uint32_t now)
{
    HAL_StatusTypeDef result;
    completion = 0U; active = 1U; started = now;
    if (write) {
        buffer[0] = (uint8_t)(value >> 8); buffer[1] = (uint8_t)value;
        result = HAL_I2C_Mem_Write_IT(&hi2c1, INA219_I2C_ADDR << 1, reg,
                                     I2C_MEMADD_SIZE_8BIT, buffer, 2U);
    } else result = HAL_I2C_Mem_Read_IT(&hi2c1, INA219_I2C_ADDR << 1, reg,
                                       I2C_MEMADD_SIZE_8BIT, buffer, 2U);
    if (result != HAL_OK) service_fail();
}

void INA219_ServiceInit(void)
{
    ina219_bus_valid = ina219_current_valid = 0U;
    ina219_ibus = NAN;
    phase = INA_PROBE; active = 0U; completion = 0U;
    transaction(INA219_REG_CONFIG, 0U, 0U, HAL_GetTick());
}

void INA219_ServicePoll(uint32_t now)
{
    uint16_t value;
    if (phase == INA_FAILED) return;
    if (!active) {
        if (phase == INA_IDLE && (uint32_t)(now - lastSample) >= 200U) {
            phase = INA_TRIGGER;
            transaction(INA219_REG_CONFIG, INA_TRIGGER_CONFIG, 1U, now);
        } else if (phase == INA_WAIT && (uint32_t)(now - conversionStarted) >= 2U) {
            phase = INA_BUS;
            transaction(INA219_REG_BUS, 0U, 0U, now);
        }
        return;
    }
    if (completion == 2U || (!completion && (uint32_t)(now - started) >= INA219_I2C_TIMEOUT)) {
        service_fail(); return;
    }
    if (!completion) return;
    __DMB();
    active = 0U;
    value = (uint16_t)((uint16_t)buffer[0] << 8) | buffer[1];
    switch (phase) {
    case INA_PROBE:
        if (value == 0xFFFFU) { service_fail(); break; }
        phase = INA_SET_CAL;
        transaction(INA219_REG_CALIB, INA219_CALIB_VALUE, 1U, now); break;
    case INA_SET_CAL:
        phase = INA_TRIGGER;
        transaction(INA219_REG_CONFIG, INA_TRIGGER_CONFIG, 1U, now); break;
    case INA_TRIGGER:
        conversionStarted = now; phase = INA_WAIT; break;
    case INA_BUS:
        if (!(value & INA219_BUS_CNVR)) {
            if ((uint32_t)(now - conversionStarted) >= 8U) service_fail();
            else phase = INA_WAIT;
            break;
        }
        pendingBus = value; phase = INA_CHECK_CONFIG;
        transaction(INA219_REG_CONFIG, 0U, 0U, now); break;
    case INA_CHECK_CONFIG:
        if (value != INA_TRIGGER_CONFIG) { service_fail(); break; }
        phase = INA_CHECK_CAL;
        transaction(INA219_REG_CALIB, 0U, 0U, now); break;
    case INA_CHECK_CAL:
        if (value != INA219_CALIB_VALUE) {
            ina219_bus_valid = ina219_current_valid = 0U;
            if (foc_stage2_driver_enabled()) motor_fault_trip(MOTOR_FAULT_BUS);
            phase = INA_SET_CAL;
            transaction(INA219_REG_CALIB, INA219_CALIB_VALUE, 1U, now);
            break;
        }
        phase = INA_CURRENT;
        transaction(INA219_REG_CURRENT, 0U, 0U, now); break;
    case INA_CURRENT: {
        float voltage = (float)(pendingBus >> 3) * INA219_BUSV_LSB_V;
        if (!isfinite(voltage) || voltage < FOC_BUS_MIN_VALID_V || voltage > FOC_BUS_MAX_VALID_V) {
            service_fail(); break;
        }
        uint32_t key = __get_PRIMASK(); __disable_irq();
        ina219_vbus = voltage; motor_config.voltage_supply = voltage;
        ina219_bus_sample_ms = now;
        ina219_ibus = (pendingBus & INA219_BUS_OVF) ? NAN : (float)(int16_t)value * INA219_CURRENT_LSB_A;
        ina219_current_valid = (pendingBus & INA219_BUS_OVF) == 0U;
        ina219_bus_valid = 1U;
        __set_PRIMASK(key);
        lastSample = now; phase = INA_IDLE; break;
    }
    default: service_fail(); break;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *handle)
{
    if (handle == &hi2c1 && active) { __DMB(); completion = 1U; }
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *handle)
{
    if (handle == &hi2c1 && active) { __DMB(); completion = 1U; }
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *handle)
{
    if (handle == &hi2c1 && active) completion = 2U;
}
