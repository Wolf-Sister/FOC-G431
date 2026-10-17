#ifndef MOTOR_PORT_H
#define MOTOR_PORT_H
#include "motor_types.h"

uint32_t MotorPort_Lock(void);
void MotorPort_Unlock(uint32_t key);
void MotorPort_Disable(void);
void MotorPort_ReadHealth(motor_health_t *health);
bool MotorPort_BeginDriver(void);
bool MotorPort_ApplyIq(int32_t iqMa, uint32_t limitMa, bool first);
bool MotorPort_ApplyTarget(const motor_command_t *command, bool first);
bool MotorPort_RunOuterLoop(void);
uint16_t MotorPort_MotionFaultBits(void);
uint16_t MotorPort_MotionFaultDetail(void);
/* First outer-loop failure, latched before disarm resets PI state. Keil watch. */
extern volatile motor_motion_fault_snapshot_t motor_motion_fault;
bool MotorPort_PositionReady(int32_t target, const motor_motion_config_t *config,
                             uint8_t role, const motor_axis_config_t *axis);
bool MotorPort_ApplyAlignment(float voltage);
bool MotorPort_CompleteAlignment(float zero);
bool MotorPort_RestoreElectrical(float zero);
void MotorPort_InvalidateElectrical(void);
void MotorPort_CaptureSample(motor_snapshot_t *sample, uint32_t now);
void MotorPort_GetElectricalConfig(int *direction, int *polePairs);
float MotorPort_GetElectricalZero(void);
#endif
