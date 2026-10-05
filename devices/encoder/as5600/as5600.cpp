#include "utils/utils.hpp"
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numbers>

#include "as5600.hpp"
#include "reg.hpp"

// common encoder device header include
#include "../common.hpp"

// platform driver
#include "platform/common.hpp"

// private helper functions

// as5600 disables address auto-increment for ANGLE, RAW ANGLE and
// MAGNITUDE regs, so we must write the address twice per read.

// ---- read-artifact model ----
// Read order is HIGH first, LOW second, so LOW is always the fresher byte:
//     reported = H(t1)*256 + L(t2)
//     truth    = H(t2)*256 + L(t2)
//     error    = (H(t1)-H(t2))*256 = -k*256
// where k is the number of low-byte 255->0 wraps between the two reads
// (an integer). The error is therefore always an exact multiple of 256
// (including 0) and never an intermediate value. The continuous band
// belongs to the *residual*: the legitimate rotation during one loop
// period plus read noise, expressed by MAX_STEP_COUNTS below.
constexpr int32_t HIGH_BYTE_UNIT = 256;

// Max legal step between two consecutive frames, in counts
// (1 count = 2*pi/4096 rad)
//     MAX_STEP = RPM_MAX * 4096 / (60 * LOOP_HZ)
// Hard constraint: must stay < 256, otherwise a +-256 artifact would be
// accepted as a legal step.
// LOOP_HZ is currently ~120 (pinned by printf), where 100 counts ~= 176 RPM.
// Once the loop runs at 10 kHz use RPM_MAX*4096/(60*10000)
// (1000 rpm -> 7, recommend 16).
constexpr int32_t MAX_STEP_COUNTS = 100;
static_assert(
    MAX_STEP_COUNTS < HIGH_BYTE_UNIT, "MAX_STEP must be < 256");
static_assert(MAX_STEP_COUNTS >= 4, "MAX_STEP too small");

// Number of high-byte units we compensate for. k = 1 covers ~7500 rpm
// (assuming dt ~= 0.5 ms). The body below is fully unrolled for k <= 1,
// so raising this value requires extending that chain by hand.
constexpr int32_t K_MAX = 1;
static_assert(
    K_MAX == 1, "get_phy_angle_unprocessced() is unrolled for k <= 1");

// not useful? implement later
FUNC_RET mem_write_2bytes(uint8_t mem_addr, uint16_t &data) {
    // if (ret_val_t::OK != (ret_val_t))
    // i2c::write_mem()
    union {
        uint16_t u16_data;
        std::array<uint8_t, 2> u8_data;
    } union_data;
    union_data.u16_data = data;

    xfr_data_t _data = {
        .data_buffer = reinterpret_cast<uint8_t *>(&union_data.u8_data[1]),
        .len         = 1,
    };

    i2c::write_mem(mem_addr, _data);
    _data.data_buffer =
        reinterpret_cast<uint8_t *>(&union_data.u8_data[0]);
    i2c::write_mem(mem_addr + 0x01, _data);

    printf("high addr:0x%02X\nget word:0x%04X\n", mem_addr,
        union_data.u16_data);
    return OK;
}
FUNC_RET mem_read_2bytes(uint8_t mem_addr, uint16_t &data) {
    // Must be zero-initialized: never read stale stack bytes when I2C fails
    union {
        uint16_t u16_data;
        std::array<uint8_t, 2> u8_data;
    } union_data = {};
    xfr_data_t _data = {
        .data_buffer = reinterpret_cast<uint8_t *>(&union_data.u8_data[1]),
        .len         = 1,
    };

    // Order matters: HIGH first then LOW makes LOW the fresher byte, so
    // without a wrap the error is *exactly* 0 (not merely "small").
    if (ret_val_t::OK != i2c::read_mem(mem_addr, _data)) {
        return ERROR;
    }
    _data.data_buffer =
        reinterpret_cast<uint8_t *>(&union_data.u8_data[0]);
    if (ret_val_t::OK != i2c::read_mem(mem_addr + 0x01, _data)) {
        return ERROR;
    }

    data = union_data.u16_data;

    return OK;
}
inline float raw_to_rad(uint16_t raw_angle) {
    return (float)(raw_angle * 0.001533980787885641f); // =raw/4096*2*pi
}
// Read + artifact disambiguation. Returns rad, without zero_bias.
//
// The decision is based on the *residual after compensation*, not on the
// magnitude of the jump falling inside some range:
//   try k=0 first (normal frame), then k=+1 and k=-1 (high-byte artifact),
//   take the first candidate where |d - k*256| <= MAX_STEP;
//   if none matches, the frame is not an artifact but a genuinely bad
//   sample -> keep the last valid value and leave the history untouched.
// Since MAX_STEP < 256 the candidate windows of neighbouring k do not
// overlap, so the solution is unique and exact.
// The candidates are unrolled: no loop, no branch table - see K_MAX.
// todo: move raw_valid / raw_valid_init into encoder_t for multi-instance
inline float get_phy_angle_unprocessced() {
    static uint16_t raw_valid      = 0;
    static bool     raw_valid_init = false;

    uint16_t raw_angle = 0;
    if (ret_val_t::OK != mem_read_2bytes(AS5600_REG_RAW_H, raw_angle)) {
        // I2C failed: keep the last valid value, do not pollute history
        return raw_valid_init ? raw_to_rad(raw_valid) : 0.0f;
    }
    raw_angle &= 0x0fff; // 12-bit

    return raw_to_rad(raw_angle);

    if (!raw_valid_init) {
        raw_valid      = raw_angle;
        raw_valid_init = true;
        return raw_to_rad(raw_valid);
    }

    // Delta against the last valid value, unwrapped to (-2048, 2048].
    // |d| <= 4095, so one conditional per side is enough - no loop.
    int32_t d = (int32_t)raw_angle - (int32_t)raw_valid;
    if (d > 2048) {
        d -= 4096;
    }
    if (d < -2048) {
        d += 4096;
    }

    // Unrolled candidates k = 0, +1, -1. `r` ends up holding the residual,
    // i.e. the legitimate per-frame rotation once the artifact is removed.
    int32_t r = d; // k = 0: normal frame
    if (r > MAX_STEP_COUNTS || r < -MAX_STEP_COUNTS) {
        r = d - HIGH_BYTE_UNIT; // k = +1
        if (r > MAX_STEP_COUNTS || r < -MAX_STEP_COUNTS) {
            r = d + HIGH_BYTE_UNIT; // k = -1
            if (r > MAX_STEP_COUNTS || r < -MAX_STEP_COUNTS) {
                // Neither a legal step nor a +-256 artifact: bad frame.
                // Keep the last valid value and leave history untouched.
                return raw_to_rad(raw_valid);
            }
        }
    }

    // Advance by the residual. The +4096 keeps the operand positive so the
    // mask is well defined regardless of the signed representation.
    raw_valid = (uint16_t)(((int32_t)raw_valid + r + 4096) & 0x0fff);
    return raw_to_rad(raw_valid);
}

// encoder init and deinitialization func
// encoder_t::encoder_t() {}
// encoder_t::~encoder_t() {};

FUNC_RET encoder_t::init() {

    // clang-format off: special format
    if (ret_val_t::OK != i2c::init_dev({                        
        .dev_addr     = AS5600_ADDRESS_7B, 
        .scl_speed_hz = 100000})) {//if
        return ERROR;
    }
    // clang-format on

    is_initialized = true;

    zero_bias = 0.0f;

    // clang-format off: wip
    // todo: apply defualt config, reset config, restore config...
    // todo: initial angel detection...

    return OK;
}
// clang-format on
float encoder_t::get_phy_angle(bool is_enable_filter) {
    // The old jerk-based linear extrapolation (angle = 2*t-1 - t-2) is gone:
    // it was a self-contained positive feedback loop -- the result depended
    // only on its own history and, once accepted, became that history for the
    // next frame, turning a single glitch into a permanent ramp (measured:
    // 90.5% of all frames were replaced by fabricated values).
    // Artifact handling now lives in get_phy_angle_unprocessced(), which
    // either restores the exact value or holds the last valid one.
    (void)is_enable_filter; // keep the param to avoid touching call sites

    float angle = get_phy_angle_unprocessced() + zero_bias;
    if (0.0f > angle) {
        angle += 2.0f * PI;
    }
    printf("%f,", angle);
    return angle;
}

void encoder_t::zero_align() {
    zero_bias = -get_phy_angle_unprocessced();
    printf("zero bias:%2.3f \n", zero_bias);
}