#include "motor_calibration_store.h"
#include <string.h>

static uint32_t cal_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t cal_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static void cal_w32(uint8_t *p, uint32_t v)
{
    unsigned i;
    for (i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8U * i));
}
static void cal_w16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static const uint8_t cal_commit[8] = {0x43, 0x41, 0x4c, 0x31, 0xbc, 0xbe, 0xb3, 0xce};
uint32_t CalStore_Crc(const uint8_t *p, uint32_t n)
{
    uint32_t crc = 0xffffffffU, i;
    while (n--)
    {
        crc ^= *p++;
        for (i = 0; i < 8U; i++)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
    }
    return crc ^ 0xffffffffU;
}
void CalStore_Encode(const cal_record_t *r, uint8_t p[CAL_RECORD_SIZE])
{
    unsigned i;
    memset(p, 0, CAL_RECORD_SIZE);
    cal_w32(p, 0x314c4143U);
    cal_w16(p + 4, 1U);
    cal_w16(p + 6, CAL_RECORD_SIZE);
    cal_w32(p + 8, r->sequence);
    cal_w32(p + 12, r->binding.revision);
    for (i = 0; i < 3; i++)
        cal_w32(p + 16 + 4 * i, r->binding.uid[i]);
    cal_w16(p + 28, r->binding.profile);
    cal_w16(p + 30, r->binding.profileRevision);
    cal_w32(p + 32, r->valid ? r->zero : 0U);
    p[36] = (uint8_t)r->binding.direction;
    p[37] = r->binding.angleMode;
    cal_w16(p + 38, r->binding.pairs);
    cal_w16(p + 40, r->binding.counts);
    cal_w16(p + 42, r->binding.algorithm);
    cal_w32(p + 44, r->valid ? 1U : 0U);
    cal_w32(p + 56, CalStore_Crc(p, 56));
    cal_w32(p + 60, ~cal_u32(p + 56));
    memcpy(p + 64, cal_commit, 8);
}
bool CalStore_Decode(const uint8_t p[CAL_RECORD_SIZE], cal_record_t *r)
{
    unsigned i;
    if (cal_u32(p) != 0x314c4143U || memcmp(p + 64, cal_commit, 8) ||
        cal_u32(p + 56) != CalStore_Crc(p, 56) || cal_u32(p + 60) != ~cal_u32(p + 56))
        return false;
    memset(r, 0, sizeof(*r));
    r->supported = cal_u16(p + 4) == 1U && cal_u16(p + 6) == CAL_RECORD_SIZE && cal_u32(p + 44) <= 1U;
    for (i = 48; i < 56; i++)
        if (p[i])
            r->supported = false;
    r->sequence = cal_u32(p + 8);
    r->binding.revision = cal_u32(p + 12);
    for (i = 0; i < 3; i++)
        r->binding.uid[i] = cal_u32(p + 16 + 4 * i);
    r->binding.profile = cal_u16(p + 28);
    r->binding.profileRevision = cal_u16(p + 30);
    r->zero = cal_u32(p + 32);
    r->binding.direction = p[36] == 255U ? -1 : (int8_t)p[36];
    r->binding.angleMode = p[37];
    r->binding.pairs = cal_u16(p + 38);
    r->binding.counts = cal_u16(p + 40);
    r->binding.algorithm = cal_u16(p + 42);
    r->valid = (cal_u32(p + 44) & 1U) != 0U;
    /* Payload semantics are checked after newest-record selection. A committed
     * incompatible/invalid value must never resurrect an older calibration. */
    return true;
}
bool CalStore_BindingEqual(const cal_binding_t *a, const cal_binding_t *b)
{
    return a->revision == b->revision && a->uid[0] == b->uid[0] && a->uid[1] == b->uid[1] &&
           a->uid[2] == b->uid[2] && a->profile == b->profile && a->profileRevision == b->profileRevision &&
           a->direction == b->direction && a->angleMode == b->angleMode && a->pairs == b->pairs &&
           a->counts == b->counts && a->algorithm == b->algorithm;
}
uint8_t CalStore_Load(cal_store_t *s, const cal_io_t *io, const cal_binding_t *b)
{
    uint8_t p[2][CAL_RECORD_SIZE];
    cal_record_t r[2];
    bool ok[2];
    bool ioError = false;
    unsigned i;
    uint32_t delta;
    memset(s, 0, sizeof(*s));
    s->slot = -1;
    for (i = 0; i < 2; i++)
    {
        uint32_t address = i ? CAL_SLOT_B : CAL_SLOT_A;
        ok[i] = false;
        if (!io->read(address, p[i], CAL_RECORD_SIZE))
        {
            if (io->read_ecc && io->read_ecc(address))
                s->eccSlots |= (uint8_t)(1U << i);
            else
                ioError = true;
        }
        else
            ok[i] = CalStore_Decode(p[i], &r[i]);
    }
    if (ioError)
        return s->error = CAL_IO;
    if (!ok[0] && !ok[1])
    {
        if (s->eccSlots)
            return s->error = CAL_ECC;
        for (i = 0; i < 2 * CAL_RECORD_SIZE; i++)
            if (((uint8_t *)p)[i] != 255U)
                return s->error = CAL_CORRUPT;
        return s->error = CAL_EMPTY;
    }
    i = ok[1] ? 1U : 0U;
    if (ok[0] && ok[1])
    {
        delta = r[1].sequence - r[0].sequence;
        if (delta == 0x80000000U || (!delta && memcmp(p[0], p[1], CAL_RECORD_SIZE)))
            return s->error = CAL_CONFLICT;
        i = delta && delta < 0x80000000U ? 1U : 0U;
    }
    s->slot = (int8_t)i;
    s->latest = r[i];
    /* Keep a readable sequence only for a later explicit repair. An unreadable
     * page could hold a newer tombstone: never restore the readable old value. */
    if (s->eccSlots)
        return s->error = CAL_ECC;
    if (!r[i].supported || !CalStore_BindingEqual(&r[i].binding, b))
        return s->error = CAL_INCOMPATIBLE;
    if (!r[i].valid)
        return s->error = CAL_INVALIDATED;
    if (r[i].zero > CAL_ZERO_MAX || (b->direction != -1 && b->direction != 1) || !b->pairs ||
        b->counts != 16384U || (b->angleMode != 1U && b->angleMode != 2U))
        return s->error = CAL_INCOMPATIBLE;
    return s->error = CAL_OK;
}
uint8_t CalStore_Save(cal_store_t *s, const cal_io_t *io, const cal_record_t *value)
{
    cal_record_t r = *value, verify;
    uint8_t bytes[CAL_RECORD_SIZE], readback[CAL_RECORD_SIZE];
    uint32_t base, offset;
    int8_t slot;
    if (s->error == CAL_CONFLICT || s->error == CAL_IO)
        return s->error;
    if (r.valid && r.zero > CAL_ZERO_MAX)
        return s->error = CAL_INCOMPATIBLE;
    if (!s->eccSlots && s->slot >= 0 && s->latest.supported && CalStore_BindingEqual(&s->latest.binding, &r.binding) && s->latest.valid == r.valid &&
        (!r.valid || s->latest.zero == r.zero))
        return s->error = CAL_OK;
    slot = s->eccSlots ? ((s->eccSlots & 1U) ? 0 : 1) : (s->slot == 0 ? 1 : 0);
    base = slot ? CAL_SLOT_B : CAL_SLOT_A;
    r.sequence = s->slot < 0 ? 1U : s->latest.sequence + 1U;
    r.supported = true;
    CalStore_Encode(&r, bytes);
    /* Repair is explicitly requested while disabled. Erase the other page
     * first, while the target still carries ECC damage. Interrupted repair
     * must not resurrect an older calibration/tombstone of unknown ordering. */
    if (s->eccSlots && !io->erase(slot ? CAL_SLOT_A : CAL_SLOT_B))
        return s->error = CAL_IO;
    if (!io->erase(base))
        return s->error = CAL_IO;
    for (offset = 0; offset < 64U; offset += 8U)
        if (!io->program(base + offset, bytes + offset))
            return s->error = CAL_IO;
    if (!io->read(base, readback, 64U))
    {
        if (io->read_ecc && io->read_ecc(base))
        {
            s->eccSlots |= (uint8_t)(1U << slot);
            return s->error = CAL_ECC;
        }
        return s->error = CAL_VERIFY;
    }
    if (memcmp(readback, bytes, 64U))
        return s->error = CAL_VERIFY;
    if (!io->program(base + 64U, bytes + 64U))
        return s->error = CAL_IO;
    if (!io->read(base, readback, CAL_RECORD_SIZE))
    {
        if (io->read_ecc && io->read_ecc(base))
        {
            s->eccSlots |= (uint8_t)(1U << slot);
            return s->error = CAL_ECC;
        }
        return s->error = CAL_VERIFY;
    }
    if (memcmp(readback, bytes, CAL_RECORD_SIZE) ||
        !CalStore_Decode(readback, &verify))
        return s->error = CAL_VERIFY;
    s->latest = r;
    s->slot = slot;
    s->eccSlots = 0U;
    return s->error = CAL_OK;
}
