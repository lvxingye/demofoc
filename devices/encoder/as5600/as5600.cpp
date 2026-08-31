#include "utils/utils.hpp"
#include <array>
#include <cstdint>

#include "as5600.hpp"
#include "reg.hpp"

// common encoder device header include
#include "devices/encoder/common.hpp"

// platform driver
#include "platform/common.hpp"

// private helper functions

// as5600 disables address auto-increment for ANGLE, RAW ANGLE and MAGNITUDE
// regs, so we must write the address twice per read.

// not useful? implement later
INLINE_RET mem_write_2bytes(uint8_t mem_addr, uint16_t &data) {
    // if (ret_val_t::OK != (ret_val_t))
    // i2c::write_mem()
    return OK;
}
INLINE_RET mem_read_2bytes(uint8_t mem_addr, uint16_t &data) {
    union {
        uint16_t u16_data;
        std::array<uint8_t, 2> u8_data;
    } union_data;
    xfr_data_t _data = {
        .data_buffer = reinterpret_cast<uint8_t *>(&union_data.u8_data[1]),
        .len         = 1};

    i2c::write_mem(mem_addr, _data);
    _data.data_buffer = reinterpret_cast<uint8_t *>(&union_data.u8_data[0]);
    i2c::write_mem(mem_addr, _data);

    data=union_data.u16_data;

    return OK;
}
inline float raw_to_rad(uint16_t raw_angle){
    return (float)(raw_angle/4096.0f*6.283185307179586f); // =raw/4096*2*pi
}

// encoder init and deinitialization func
encoder_t::encoder_t() {}
encoder_t::~encoder_t() {};

FUNC_RET encoder_t::init() {

    // clang-format off: special format
    if (ret_val_t::OK != i2c::init_dev({                        
        .dev_addr     = AS5600_ADDRESS_7B, 
        .scl_speed_hz = 100000})) {//if
        return ERROR;
    }
    // clang-format on

    is_initialized = true;

    // clang-format off: wip
    // todo: apply defualt config, reset config, restore config...
    // todo: initial angel detection...

    return OK;
}
inline float encoder_t::get_phy_angle() {
    uint16_t raw_angle=0.0f;
    if(ret_val_t::OK!=mem_read_2bytes(AS5600_REG_RAW_H,raw_angle)){
        return 0.0f;
    }
    return raw_to_rad(raw_angle);
}
