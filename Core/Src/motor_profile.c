#include "motor_profile.h"
#include "board_safety_profile.h"
#include "motor_axis.h"
#include <stdint.h>

#if APP_MOTOR_PROFILE == 0U
/* Existing uninstalled bench motor. Keep its proven WHEEL behavior unchanged. */
const motor_profile_t MotorProfile_Selected = {
    0x0001U, 1U, MOTOR_PROFILE_NODE_ID, MOTOR_ROLE_WHEEL, 1, 0U, 0U, 0,
    BOARD_DEFAULT_POSITION_MIN, BOARD_DEFAULT_POSITION_MAX, 0U,
    BOARD_REF_MAX_MA, BOARD_INITIAL_IQ_LIMIT_MA,
    BOARD_SPEED_MAX_MRAD_S, BOARD_POSITION_SPEED_MAX_MRAD_S, BOARD_ACCEL_MAX_MRAD_S2,
    BOARD_DEFAULT_SPEED_LIMIT_MRAD_S, BOARD_DEFAULT_POSITION_SPEED_MRAD_S,
    BOARD_DEFAULT_ACCEL_MRAD_S2,
    BOARD_ACCEL_MAX_MRAD_S2, 20U, 0U, 1,
    0U, 1U, 0U, 0U, 1U
};
#elif APP_MOTOR_PROFILE == 1U
/* Uncalibrated JOINT template: diagnostic only until real limits and tests exist. */
const motor_profile_t MotorProfile_Selected = {
    0x0101U, 1U, MOTOR_PROFILE_NODE_ID, MOTOR_ROLE_JOINT, 1, 0U, 0U, 0,
    0, 0, 0U,
    0U, 0U,
    0U, 0U, 0U,
    0U, 0U, 0U,
    0U, 0U, 0U, 1,
    0U, 1U, 0U, 0U, 0U
};
#elif APP_MOTOR_PROFILE == 2U
/* Read-only coordinate bench profile for UID 00410009/34354B14/33383735.
 * User-confirmed zero: raw=10140. CCW viewed from the shaft front is positive.
 * +/-90 degrees is a software diagnostic window, not measured joint stops.
 * All output limits and installation/motion/alignment qualifications stay off. */
const motor_profile_t MotorProfile_Selected = {
    .profileId = 0x0102U,
    .revision = 1U,
    .nodeId = MOTOR_PROFILE_NODE_ID,
    .role = MOTOR_ROLE_JOINT,
    .axisDirection = -1,
    .referencePresent = 1U,
    .referenceRaw = 10140U,
    .referenceQ = 0,
    .minimumQ = -15708,
    .maximumQ = 15708,
    .iqDirection = 1,
    .encoderAngleRaw = 1U
};
#else
#error "Unknown APP_MOTOR_PROFILE. Add and validate a compile-time motor profile."
#endif

uint32_t MotorProfile_Flags(void)
{
    return ((uint32_t)(MotorProfile_Selected.referencePresent != 0U) << 0) |
           ((uint32_t)(MotorProfile_Selected.iqDirectionValidated != 0U) << 1) |
           ((uint32_t)(MotorProfile_Selected.installationQualified != 0U) << 2) |
           ((uint32_t)(MotorProfile_Selected.motionQualified != 0U) << 3) |
           ((uint32_t)(MotorProfile_Selected.encoderAngleRaw != 0U) << 4) |
           ((uint32_t)(MotorProfile_Selected.alignmentAllowed != 0U) << 5);
}

uint8_t MotorProfile_Validate(void)
{
    if (MotorProfile_Selected.nodeId > 63U || MotorProfile_Selected.role > MOTOR_ROLE_WHEEL)
        return 0U;
    if (MotorProfile_Selected.currentMaxMa > BOARD_REF_MAX_MA ||
        MotorProfile_Selected.speedMaxMradS > BOARD_SPEED_MAX_MRAD_S ||
        MotorProfile_Selected.positionSpeedMaxMradS > BOARD_POSITION_SPEED_MAX_MRAD_S ||
        MotorProfile_Selected.accelerationMaxMradS2 > BOARD_ACCEL_MAX_MRAD_S2 ||
        MotorProfile_Selected.positionSpeedMaxMradS > MotorProfile_Selected.speedMaxMradS ||
        MotorProfile_Selected.speedDefaultMradS > MotorProfile_Selected.speedMaxMradS ||
        MotorProfile_Selected.positionSpeedDefaultMradS > MotorProfile_Selected.positionSpeedMaxMradS ||
        MotorProfile_Selected.positionSpeedDefaultMradS > MotorProfile_Selected.speedDefaultMradS ||
        MotorProfile_Selected.accelerationDefaultMradS2 > MotorProfile_Selected.accelerationMaxMradS2)
        return 0U;
    if ((MotorProfile_Selected.currentMaxMa == 0U) !=
        (MotorProfile_Selected.initialCurrentMa == 0U) ||
        MotorProfile_Selected.initialCurrentMa > MotorProfile_Selected.currentMaxMa)
        return 0U;
    if (MotorProfile_Selected.role == MOTOR_ROLE_UNCONFIGURED)
        return (MotorProfile_Selected.currentMaxMa == 0U &&
                MotorProfile_Selected.motionQualified == 0U) ? 1U : 0U;
    if (MotorProfile_Selected.role == MOTOR_ROLE_JOINT) {
        if (MotorProfile_Selected.axisDirection != 1 && MotorProfile_Selected.axisDirection != -1)
            return 0U;
        if (MotorProfile_Selected.iqDirection != 1 && MotorProfile_Selected.iqDirection != -1)
            return 0U;
        if (MotorProfile_Selected.referencePresent &&
            (MotorProfile_Selected.referenceRaw >= MOTOR_AXIS_ENCODER_COUNTS ||
             MotorProfile_Selected.minimumQ >= MotorProfile_Selected.maximumQ ||
             MotorProfile_Selected.referenceQ < MotorProfile_Selected.minimumQ ||
             MotorProfile_Selected.referenceQ > MotorProfile_Selected.maximumQ ||
             (int64_t)MotorProfile_Selected.maximumQ - MotorProfile_Selected.minimumQ +
                 2LL * MotorProfile_Selected.mapMarginQ >= MOTOR_AXIS_TURN_Q)) return 0U;
        if (!MotorProfile_Selected.motionQualified &&
            (!MotorProfile_Selected.installationQualified && !MotorProfile_Selected.alignmentAllowed))
            return 1U;
    }
    if (MotorProfile_Selected.currentMaxMa == 0U ||
        MotorProfile_Selected.initialCurrentMa == 0U ||
        MotorProfile_Selected.initialCurrentMa > MotorProfile_Selected.currentMaxMa)
        return 0U;
    if (MotorProfile_Selected.role == MOTOR_ROLE_JOINT) {
        if (MotorProfile_Selected.motionQualified &&
            (!MotorProfile_Selected.referencePresent ||
             MotorProfile_Selected.minimumQ >= MotorProfile_Selected.maximumQ ||
             ((int64_t)MotorProfile_Selected.maximumQ - MotorProfile_Selected.minimumQ) >= MOTOR_AXIS_TURN_Q ||
             !MotorProfile_Selected.installationQualified || !MotorProfile_Selected.iqDirectionValidated ||
             !MotorProfile_Selected.brakingDecelerationMradS2 || !MotorProfile_Selected.reactionTimeMs ||
             !MotorProfile_Selected.alignmentAllowed || !MotorProfile_Selected.speedMaxMradS ||
             !MotorProfile_Selected.positionSpeedMaxMradS || !MotorProfile_Selected.accelerationMaxMradS2 ||
             !MotorProfile_Selected.speedDefaultMradS || !MotorProfile_Selected.positionSpeedDefaultMradS ||
             !MotorProfile_Selected.accelerationDefaultMradS2))
            return 0U;
    }
    return 1U;
}
