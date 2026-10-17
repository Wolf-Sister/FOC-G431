#ifndef MOTOR_PROFILE_H
#define MOTOR_PROFILE_H

#include <stdint.h>

/* Current uninstalled bench: WHEEL profile 0. Use profile 2 only for disabled
 * joint-coordinate diagnostics; real installed joint motion remains unqualified. */
#ifndef APP_MOTOR_PROFILE
#define APP_MOTOR_PROFILE 0U
#endif

#define MOTOR_ROLE_UNCONFIGURED 0U
#define MOTOR_ROLE_JOINT 1U
#define MOTOR_ROLE_WHEEL 2U

#if APP_MOTOR_PROFILE == 0U
#define MOTOR_PROFILE_DEFAULT_NODE_ID 3U
#define MOTOR_PROFILE_ROLE MOTOR_ROLE_WHEEL
#define MOTOR_PROFILE_ENCODER_RAW_ANGLE 1
#elif APP_MOTOR_PROFILE == 1U || APP_MOTOR_PROFILE == 2U
#define MOTOR_PROFILE_DEFAULT_NODE_ID 1U
#define MOTOR_PROFILE_ROLE MOTOR_ROLE_JOINT
#define MOTOR_PROFILE_ENCODER_RAW_ANGLE 1
#else
#error "Unknown APP_MOTOR_PROFILE. Add and validate a compile-time motor profile."
#endif

#ifndef APP_MOTOR_NODE_ID
#define APP_MOTOR_NODE_ID MOTOR_PROFILE_DEFAULT_NODE_ID
#endif
#define MOTOR_PROFILE_NODE_ID APP_MOTOR_NODE_ID

typedef struct {
    uint16_t profileId;
    uint16_t revision;
    uint8_t nodeId;
    uint8_t role;
    int8_t axisDirection;
    uint8_t referencePresent;
    uint16_t referenceRaw;
    int32_t referenceQ;
    int32_t minimumQ;
    int32_t maximumQ;
    uint16_t mapMarginQ;
    uint32_t currentMaxMa;
    uint32_t initialCurrentMa;
    uint32_t speedMaxMradS;
    uint32_t positionSpeedMaxMradS;
    uint32_t accelerationMaxMradS2;
    uint32_t speedDefaultMradS;
    uint32_t positionSpeedDefaultMradS;
    uint32_t accelerationDefaultMradS2;
    uint32_t brakingDecelerationMradS2;
    uint16_t reactionTimeMs;
    uint16_t positionMarginQ;
    int8_t iqDirection;
    uint8_t iqDirectionValidated;
    uint8_t encoderAngleRaw;
    uint8_t installationQualified;
    uint8_t motionQualified;
    uint8_t alignmentAllowed;
} motor_profile_t;

extern const motor_profile_t MotorProfile_Selected;

uint32_t MotorProfile_Flags(void);
uint8_t MotorProfile_Validate(void);

#endif
