#ifndef MOTOR_SUPERVISOR_H
#define MOTOR_SUPERVISOR_H
#include "can_protocol.h"
#include "motor_types.h"

#define MOTOR_FAULT_BIT_CONTROL_STALLED (1U << 8)
#define MOTOR_FAULT_BIT_COMM_TIMEOUT    (1U << 10)
#define MOTOR_FAULT_BIT_BUS_OFF         (1U << 11)
#define MOTOR_FAULT_BIT_INTERNAL        (1U << 15)

void MotorSupervisor_Init(uint32_t seed, uint32_t now, bool automaticCalibration);
bool MotorSupervisor_Active(void);
/* Call with IRQs locked before accepting a main-loop software reset request. */
bool MotorSupervisor_ResetReady(void);
bool MotorSupervisor_RestoreElectrical(float zero);
void MotorSupervisor_StorageProgress(uint32_t now);
void MotorSupervisor_EndStorage(uint32_t now);
bool MotorSupervisor_PermitsOutput(void);
void MotorSupervisor_ReadFault(uint8_t *state, uint8_t *mode, uint16_t *seq, uint16_t *detail);
uint32_t MotorSupervisor_Epoch(void);
void MotorSupervisor_StopISR(void);
void MotorSupervisor_FaultISR(uint16_t bits, uint16_t detail);
bool MotorSupervisor_TargetISR(const uint8_t data[8], uint32_t now);
void MotorSupervisor_Tick(uint32_t now);
void MotorSupervisor_Request(const uint8_t request[8], uint32_t receivedEpoch,
                              uint32_t receivedMs, uint32_t now, uint8_t reply[8]);
void MotorSupervisor_ReadStatus(CAN_NodeStatus *status, uint32_t now);
bool MotorSupervisor_TakeEvent(motor_event_t *event);
void MotorSupervisor_OnAdcProgress(uint32_t now);
void MotorSupervisor_ApplyAtAdcBoundary(uint32_t now);
void MotorSupervisor_PublishSample(uint32_t now);
bool MotorSupervisor_ReadSample(motor_snapshot_t *sample);
void MotorSupervisor_MainProgress(uint32_t now);
bool MotorSupervisor_ReadAxisStatus(int32_t *positionQ, uint8_t *coordinateValid,
                                    uint8_t *reason, uint8_t *dirty);
#endif
