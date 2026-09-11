#include "utils/utils.hpp"
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numbers>

#include "as5600.hpp"
#include "reg.hpp"

// common encoder device header include
#include "devices/encoder/common.hpp"

// platform driver
#include "platform/common.hpp"

// private helper functions

constexpr float angle_episilon = 0.3f;

// as5600 disables address auto-increment for ANGLE, RAW ANGLE and
// MAGNITUDE regs, so we must write the address twice per read.

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
    union {
        uint16_t u16_data;
        std::array<uint8_t, 2> u8_data;
    } union_data;
    xfr_data_t _data = {
        .data_buffer = reinterpret_cast<uint8_t *>(&union_data.u8_data[1]),
        .len         = 1,
    };

    i2c::read_mem(mem_addr, _data);
    _data.data_buffer =
        reinterpret_cast<uint8_t *>(&union_data.u8_data[0]);
    i2c::read_mem(mem_addr + 0x01, _data);

    data = union_data.u16_data;

    return OK;
}
inline float raw_to_rad(uint16_t raw_angle) {
    return (float)(raw_angle * 0.001533980787885641f); // =raw/4096*2*pi
}
// return rad
inline float get_phy_angle_unprocessced() {
    uint16_t raw_angle = 0.0f;
    if (ret_val_t::OK != mem_read_2bytes(AS5600_REG_RAW_H, raw_angle)) {
        return 0.0f;
    }
    return raw_to_rad(raw_angle & 0x0fff);
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
    static std::array<float, 2> last_read = {-1.0f, -1.0f};
    float angle = get_phy_angle_unprocessced() + zero_bias;
    if (0.0f > angle) {
        angle += 2.0f * PI;
    }

    // linear-extrapolation (jerk) check:
    //   t0=angle, t-1=last_read[0], t-2=last_read[1]
    //   v_prev = t-1 - t-2   (上一帧速度)
    //   v_cur  = t0  - t-1   (本次速度)
    //   若 角加速度 v_cur - v_prev 突变超阈值 → 判本次读数为跳变，
    //   用匀速外推 t0' = 2*t-1 - t-2 替代。
    //   注：所有速度差先回绕到 (-PI, PI]，避免跨 0/2PI 时误判；
    //       外推结果再 wrap 回 [0, 2PI)，防止出现 >2PI 的假值。
    if (is_enable_filter && last_read[0] >= 0.0f && last_read[1] >= 0.0f) {
        auto wrap = [](float x) {
            while (x > PI) {
                x -= 2.0f * PI;
            }
            while (x <= -PI) {
                x += 2.0f * PI;
            }
            return x;
        };
        float v_prev = wrap(last_read[0] - last_read[1]); // t-1 - t-2
        float v_cur  = wrap(angle - last_read[0]);        // t0  - t-1
        float jerk   = v_cur - v_prev;                    // 二阶差分

        if (jerk > angle_episilon || jerk < -angle_episilon) {
            angle = 2.0f * last_read[0] - last_read[1]; // linear scaling
            while (angle < 0.0f) {
                angle += 2.0f * PI;
            }
            while (angle >= 2.0f * PI) {
                angle -= 2.0f * PI;
            }
        }
    }
    printf("%f,%f,%f,", angle, last_read[0], last_read[1]);
    last_read[1] = last_read[0];
    last_read[0] = angle;
    return angle;
}

void encoder_t::zero_align() {
    zero_bias = -get_phy_angle_unprocessced();
    printf("zero bias:%2.3f \n", zero_bias);
}