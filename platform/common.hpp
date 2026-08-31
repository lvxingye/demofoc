#pragma once

#include "utils/utils.hpp"
#include <cstdint>


// namespace foc::pltf{
namespace i2c {
// public:
//     i2c();
//     ~i2c();

struct bus_config_t{};// to be added in the future
struct dev_config_t{
    // uint8_t is_use_10_bit_addr=0;
    uint16_t dev_addr=0x0000;
    uint32_t scl_speed_hz=0;
};

FUNC_RET init_bus(const bus_config_t &config);
FUNC_RET deinit_bus();

INLINE_RET init_dev(const dev_config_t &config);
INLINE_RET deinit_dev();


    // wrapper functions for platform iic bus send data
INLINE_RET send_bytes(const uint8_t mem_addr, const xfr_data_t &data);
    // wrapper functions for platform iic bus receive data
INLINE_RET recv_bytes(const uint8_t mem_addr, const xfr_data_t &data);
INLINE_RET write_mem(const uint8_t mem_addr, const xfr_data_t &data);
INLINE_RET read_mem(const uint8_t mem_addr, const xfr_data_t &data);

}; // namespace i2c

// namespace spi;

namespace uart {

// public:
//     uart();
//     ~uart();
FUNC_RET init();
FUNC_RET deinit();

INLINE_RET send_bytes(const uint8_t mem_addr, const xfr_data_t &data);
INLINE_RET recv_bytes(const uint8_t mem_addr, const xfr_data_t &data);
}; // namespace uart
// namespace pwm {
//     pwm();
//     ~pwm();
// };

namespace current_sensor {
// public:
//     current_sensor_t();
//     ~current_sensor_t();
FUNC_RET init();
FUNC_RET deinit();

abc_phase_t get_phase_current();

// private:
}; // namespace current_sensor

namespace pwm_controller {
// public:
//     controller_t();
//     ~controller_t();
FUNC_RET init();
FUNC_RET deinit();

void apply_phase_pwm(abc_phase_t);
}; // namespace controller

// }