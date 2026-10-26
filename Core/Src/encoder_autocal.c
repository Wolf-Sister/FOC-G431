#include "encoder_autocal.h"
#if APP_ENCODER_TYPE == APP_ENCODER_MT6835
#include "encoder_autocal_port.h"
#include "motor_supervisor.h"
#include "motor_port.h"
#include "mt6835.h"
#include <math.h>

/* Qualify near the 40 RPM target before asserting CAL_EN. During calibration
 * the chip's actual 25..50 RPM setting is the permitted operating band. */
#define EAC_SPEED_TOLERANCE_RAD_S 0.4188790f
static bool speed_in_band(float speed) { return speed>=2.6179939f && speed<5.2359878f; }

static volatile uint8_t phase, error, chipStatus, seenRunning;
static uint8_t supported, coldBoot;
static volatile uint8_t triggered;
static volatile uint32_t phaseMs, lastStatusMs;
static uint32_t startedMs, pollMs;
static uint8_t speedBad;
static uint32_t meanStartedMs, meanLastMs;
static float meanArea, meanLastSpeed, meanSpeed;
static uint32_t badSpeedMs;
static int64_t initialCounts;
static uint32_t turns10;
static uint32_t failureHealth, failureSampleAge;

static void mean_reset(uint32_t now, float speed)
{
    meanStartedMs=meanLastMs=now;
    meanArea=meanSpeed=0.0f; meanLastSpeed=speed;
}
static bool mean_update(uint32_t now, float speed)
{
    uint32_t dt=(uint32_t)(now-meanLastMs), span;
    if (!dt) return false;
    /* Time weighting prevents main-loop polling frequency from biasing the
     * average. Short speed ripple does not restart a consecutive-sample timer. */
    meanArea+=(meanLastSpeed+speed)*0.5f*(float)dt;
    meanLastMs=now; meanLastSpeed=speed;
    span=(uint32_t)(now-meanStartedMs);
    if (span<1000U) return false;
    meanSpeed=meanArea/(float)span;
    meanArea=0.0f; meanStartedMs=now;
    return true;
}

static void fail(uint8_t reason)
{
    uint32_t key=MotorPort_Lock();
    /* A success IRQ can interrupt main's health check. Never turn CAL_EN off
     * once EEPROM saving has begun, even if that old health snapshot failed. */
    if (phase==EAC_SAVING || phase==EAC_POWER_CYCLE) { MotorPort_Unlock(key); return; }
    MotorSupervisor_StopISR();
    if (triggered) MT6835_HoldSamplingISR();
    EncoderAutoCalPort_Pin(false);
    error=reason; phase=EAC_FAILED;
    MotorPort_Unlock(key);
}
static void fail_health(const encoder_autocal_input_t *in, bool triggerCheck)
{
    uint32_t key=MotorPort_Lock();
    if (phase==EAC_SAVING || phase==EAC_POWER_CYCLE) { MotorPort_Unlock(key); return; }
    /* Preserve the input that caused the stop, before DISARM changes health. */
    failureHealth=in->healthFaults |
        (in->saved ? EAC_HEALTH_SAVED:0U) |
        (in->role!=2U ? EAC_HEALTH_ROLE:0U) |
        (phase!=EAC_PREPARING && in->mode!=1U ? EAC_HEALTH_MODE:0U) |
        (!isfinite(in->velocity) ? EAC_HEALTH_VELOCITY:0U) |
        (phase!=EAC_PREPARING && !in->enabled ? EAC_HEALTH_OUTPUT_OFF:0U) |
        (!in->electrical ? EAC_HEALTH_ELECTRICAL:0U) |
        (phase==EAC_PREPARING && in->enabled ? EAC_HEALTH_OUTPUT_ON:0U) |
        (triggerCheck && !speed_in_band(in->velocity) ? EAC_HEALTH_TRIGGER_SPEED:0U);
    failureSampleAge=in->sampleAgeMs;
    fail(EAC_ERR_HEALTH);
    MotorPort_Unlock(key);
}
void EncoderAutoCal_Init(bool cold)
{
    coldBoot=cold ? 1U:0U;
    supported=EncoderAutoCalPort_Init() ? 1U:0U;
    triggered=EncoderAutoCalPort_Cookie();
    phase=triggered ? EAC_POWER_CYCLE:EAC_IDLE;
    error=chipStatus=seenRunning=speedBad=0U;
    phaseMs=startedMs=pollMs=lastStatusMs=turns10=0U;
    mean_reset(0U,0.0f);
    failureHealth=failureSampleAge=0U;
    EncoderAutoCalPort_Pin(false);
}
bool EncoderAutoCal_Supported(void) { return supported != 0U || triggered != 0U; }
bool EncoderAutoCal_OutputBlocked(void)
{
    return phase==EAC_PREPARING || phase==EAC_SAVING || phase==EAC_POWER_CYCLE ||
           (phase==EAC_FAILED && triggered);
}
bool EncoderAutoCal_BlocksActions(void)
{
    return phase==EAC_PREPARING || phase==EAC_WAIT_SPEED || phase==EAC_RUNNING ||
           EncoderAutoCal_OutputBlocked();
}
bool EncoderAutoCal_BlocksReset(void)
{
    return triggered || phase==EAC_PREPARING || phase==EAC_PREPARED || phase==EAC_WAIT_SPEED;
}
bool EncoderAutoCal_BootHeld(void) { return triggered && !coldBoot; }
bool EncoderAutoCal_BootProbeNeeded(void) { return triggered && coldBoot; }
bool EncoderAutoCal_ConfirmBootStatus(uint8_t status)
{
    if (status==255U) return false;
    if (status & 0xC0U) {
        (void)EncoderAutoCalPort_Mark(); triggered=1U; phase=EAC_POWER_CYCLE; return false;
    }
    if (triggered && !coldBoot) return false;
    if (triggered) EncoderAutoCalPort_Clear();
    if (EncoderAutoCalPort_Cookie()) return false;
    triggered=0U; phase=EAC_IDLE; return true;
}
uint8_t EncoderAutoCal_Prepare(uint32_t now)
{
    encoder_autocal_input_t in;
    if (!supported) return CAN_RESULT_UNSUPPORTED;
    if (triggered || (phase!=EAC_IDLE && phase!=EAC_PREPARED && phase!=EAC_FAILED))
        return CAN_RESULT_BAD_STATE;
    EncoderAutoCalPort_Input(&in);
    /* A persistent tombstone must precede sensor EEPROM changes. Temporary
     * electrical alignment restores RAM calibration without saving it. */
    if (!in.healthy || !in.electrical || in.enabled || in.saved || in.role!=2U)
        return CAN_RESULT_BAD_STATE;
    error=seenRunning=speedBad=0U;
    chipStatus=0U; turns10=0U; phaseMs=now; phase=EAC_PREPARING;
    failureHealth=failureSampleAge=0U;
    return CAN_RESULT_OK;
}
uint8_t EncoderAutoCal_Start(uint32_t now)
{
    encoder_autocal_input_t in;
    if (phase!=EAC_PREPARED || triggered) return CAN_RESULT_BAD_STATE;
    EncoderAutoCalPort_Input(&in);
    if (!in.healthy || !in.electrical || !in.enabled || in.saved || in.role!=2U || in.mode!=1U)
        return CAN_RESULT_BAD_STATE;
    speedBad=0U; mean_reset(now,in.velocity); phaseMs=now; phase=EAC_WAIT_SPEED;
    return CAN_RESULT_OK;
}
void EncoderAutoCal_OnStatusISR(uint8_t value,uint32_t now)
{
    uint8_t status=(uint8_t)(value >> 6);
    if (phase!=EAC_RUNNING) return;
    lastStatusMs=now; chipStatus=value;
    if (value==255U) { fail(EAC_ERR_CHIP); return; }
    if (status==1U) seenRunning=1U;
    else if (status==2U) fail(EAC_ERR_CHIP);
    else if (status==3U && seenRunning) {
        /* Stop at the status DMA completion, before another angle transaction.
         * CAL_EN remains high throughout the >6 s EEPROM wait. */
        MT6835_HoldSamplingISR();
        MotorSupervisor_StopISR();
        phaseMs=now; phase=EAC_SAVING;
    }
}
void EncoderAutoCal_Poll(uint32_t now)
{
    encoder_autocal_input_t in;
    if (phase==EAC_SAVING) {
        if ((uint32_t)(now-phaseMs)>6500U) phase=EAC_POWER_CYCLE;
        return;
    }
    if (phase==EAC_IDLE || phase==EAC_FAILED || phase==EAC_POWER_CYCLE) return;
    EncoderAutoCalPort_Input(&in);
    if (phase==EAC_PREPARING) {
        uint8_t result;
        if (in.enabled || !in.healthy || in.saved) { fail_health(&in,false); return; }
        result=EncoderAutoCalPort_Configure();
        if (!result) { phase=EAC_PREPARED; phaseMs=now; }
        else if (result!=1U || (uint32_t)(now-phaseMs)>50U) fail(EAC_ERR_PREPARE);
        return;
    }
    if (phase==EAC_PREPARED) {
        if ((uint32_t)(now-phaseMs)>30000U) fail(EAC_ERR_TIMEOUT);
        return;
    }
    if (!in.enabled) { fail(EAC_ERR_CANCELLED); return; }
    if (!in.healthy || in.role!=2U || in.mode!=1U || in.saved || !isfinite(in.velocity)) {
        fail_health(&in,false); return;
    }
    if (phase==EAC_WAIT_SPEED) {
        if ((uint32_t)(now-phaseMs)>20000U) { fail(EAC_ERR_SPEED); return; }
        if (mean_update(now,in.velocity) && isfinite(meanSpeed) &&
            fabsf(meanSpeed-4.1887902f)<=EAC_SPEED_TOLERANCE_RAD_S && speed_in_band(in.velocity)) {
            uint32_t key=MotorPort_Lock();
            /* Serialize the physical trigger with the immediate CAN stop ISR.
             * The input snapshot taken above can predate a DISARM interrupt. */
            EncoderAutoCalPort_Input(&in);
            if (!in.healthy || !in.enabled || in.saved || in.role!=2U || in.mode!=1U ||
                !isfinite(in.velocity)) {
                MotorPort_Unlock(key); fail_health(&in,true); return;
            }
            if (!speed_in_band(in.velocity)) {
                mean_reset(now,in.velocity); MotorPort_Unlock(key); return;
            }
            if (!EncoderAutoCalPort_Mark()) {
                MotorPort_Unlock(key); fail(EAC_ERR_BACKUP); return;
            }
            triggered=1U; initialCounts=in.counts;
            startedMs=pollMs=lastStatusMs=now; seenRunning=0U;
            phase=EAC_RUNNING; EncoderAutoCalPort_Pin(true);
            MotorPort_Unlock(key);
        }
        return;
    }
    if (phase==EAC_RUNNING) {
        int64_t delta=in.counts-initialCounts;
        if (delta<0) delta=-delta;
        turns10=(uint32_t)(delta*10/16384);
        if (!seenRunning && (uint32_t)(now-startedMs)>2000U) {
            fail(EAC_ERR_CHIP); return;
        }
        if ((uint32_t)(now-startedMs)>180000U || (uint32_t)(now-lastStatusMs)>500U) {
            fail(EAC_ERR_TIMEOUT); return;
        }
        if (!speed_in_band(in.velocity)) {
            if (!speedBad) { speedBad=1U; badSpeedMs=now; }
            if ((uint32_t)(now-badSpeedMs)>=100U) { fail(EAC_ERR_SPEED); return; }
        } else speedBad=0U;
        if ((uint32_t)(now-pollMs)>=20U) { pollMs=now; MT6835_RequestCalStatus(); }
    }
}
bool EncoderAutoCal_Read(uint16_t selector,uint32_t *value)
{
    if (!EncoderAutoCal_Supported() || !value) return false;
    switch(selector) {
    case 0x0240: *value=phase; break;
    case 0x0241: *value=chipStatus; break;
    case 0x0242: *value=turns10; break;
    case 0x0243: *value=error; break;
    case 0x0244: *value=triggered; break;
    case 0x0245: *value=failureHealth; break;
    case 0x0246: *value=failureSampleAge; break;
    default:return false;
    }
    return true;
}
#endif
