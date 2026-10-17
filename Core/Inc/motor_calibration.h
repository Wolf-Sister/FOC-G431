#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H
#include "motor_calibration_store.h"
#include "motor_types.h"
#define CAL_STATUS_SAVED 1U
#define CAL_STATUS_LOADED 2U
#define CAL_STATUS_DIRTY 4U
#define CAL_STATUS_BUSY 8U
#define CAL_STATUS_ERROR 16U
#define CAL_STATUS_INVALIDATED 32U
void MotorCalibration_Init(void);
void MotorCalibration_Poll(void); /* Main context only. */
bool MotorCalibration_Busy(void);
bool MotorCalibration_CanWrite(void);
bool MotorCalibration_HasRestore(void);
void MotorCalibration_CancelRestore(void);
uint8_t MotorCalibration_Request(const uint8_t request[8]);
void MotorCalibration_AcceptedTx(uint8_t requestId, uint32_t buffer);
bool MotorCalibration_HoldsTx(void);
bool MotorCalibration_TakeEvent(motor_event_t *event);
uint32_t MotorCalibration_Status(void);
uint32_t MotorCalibration_Sequence(void);
uint32_t MotorCalibration_Error(void);
bool MotorCalibration_RecoveryRequired(void);
/* Hardware adapter; production code and host models share the state machine. */
extern const cal_io_t MotorCalibration_IO;
bool MotorCalibrationPort_Init(cal_binding_t *binding);
bool MotorCalibrationPort_Quiesce(void);
bool MotorCalibrationPort_Resume(void);
bool MotorCalibrationPort_TxConfirmed(uint32_t buffer);
void MotorCalibrationPort_Flush(void);
/* NMI only; true only for an attributed, guarded calibration-page read. */
bool MotorCalibrationPort_HandleNmi(void);
#endif
