#ifndef MOTOR_CALIBRATION_STORE_H
#define MOTOR_CALIBRATION_STORE_H
#include <stdbool.h>
#include <stdint.h>

#define CAL_RECORD_SIZE 72U
#define CAL_SLOT_A 0x0801F000UL
#define CAL_SLOT_B 0x0801F800UL
#define CAL_PAGE_SIZE 0x800U
#define CAL_ZERO_MAX 6283185UL
enum
{
    CAL_OK,
    CAL_EMPTY,
    CAL_CORRUPT,
    CAL_CONFLICT,
    CAL_INCOMPATIBLE,
    CAL_INVALIDATED,
    CAL_IO,
    CAL_VERIFY,
    CAL_TX_TIMEOUT,
    CAL_HEALTH,
    CAL_CANCELLED,
    CAL_CAPACITY,
    CAL_ECC
};
typedef struct
{
    uint32_t revision, uid[3];
    uint16_t profile, profileRevision, pairs, counts, algorithm;
    int8_t direction;
    uint8_t angleMode;
} cal_binding_t;
typedef struct
{
    uint32_t sequence, zero;
    bool valid, supported;
    cal_binding_t binding;
} cal_record_t;
typedef struct
{
    bool (*read)(uint32_t address, uint8_t *bytes, uint32_t length);
    bool (*erase)(uint32_t address);
    bool (*program)(uint32_t address, const uint8_t bytes[8]);
    /* Optional: classify a failed read as a quarantined ECC page, not generic IO. */
    bool (*read_ecc)(uint32_t address);
} cal_io_t;
typedef struct
{
    cal_record_t latest;
    int8_t slot;
    uint8_t error;
    uint8_t eccSlots;
} cal_store_t;
uint32_t CalStore_Crc(const uint8_t *bytes, uint32_t length);
void CalStore_Encode(const cal_record_t *record, uint8_t bytes[CAL_RECORD_SIZE]);
/* True means an intact committed envelope. Check record->supported before use. */
bool CalStore_Decode(const uint8_t bytes[CAL_RECORD_SIZE], cal_record_t *record);
bool CalStore_BindingEqual(const cal_binding_t *a, const cal_binding_t *b);
uint8_t CalStore_Load(cal_store_t *store, const cal_io_t *io, const cal_binding_t *binding);
uint8_t CalStore_Save(cal_store_t *store, const cal_io_t *io, const cal_record_t *record);
#endif
