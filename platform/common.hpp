#pragma once

#include "utils/utils.hpp"
#include <array>
#include <cstdint>

// namespace foc::pltf{
namespace gpio {
    enum mode:uint8_t{
        off=0,
        in,
        out,
        out_od,
        in_out_od,
        in_out
    };
    enum intr:uint8_t{
        disable=0,
        posedge,
        negedge,
        dualedge,
        low_level,
        high_level
    };
    enum level:uint8_t{
        low=0,
        high=1
    };

    struct config_t {
        uint64_t pin_bit_mask;  /*!< GPIO pin: set with bit mask, each bit maps
                                   to a GPIO */
        mode mode ;       
        uint8_t pull_up_en ; 
        uint8_t pull_down_en ; 
        intr intr_type ;    
    };

    FUNC_RET init(const config_t &config);
    // FUNC_RET init(const config_t* &config_ptr,uint8_t num);
    FUNC_RET deinit();

    INLINE_RET set_pin_level(uint8_t pin, level level);
    INLINE_RET set_pin_level(uint64_t pin_mask, level level);
    INLINE_RET get_pin_level(uint8_t pin, level &level);
    INLINE_RET get_pin_level(uint64_t pin_mask, level &level);
} // namespace gpio

namespace uart {
    FUNC_RET init();
    FUNC_RET deinit();

    INLINE_RET send_bytes(const uint8_t mem_addr, const xfr_data_t &data);
    INLINE_RET recv_bytes(const uint8_t mem_addr, const xfr_data_t &data);
} // namespace uart

namespace i2c {
    struct bus_config_t {}; // to be added in the future
    struct dev_config_t {
        // uint8_t is_use_10_bit_addr=0;
        uint16_t dev_addr     = 0x0000;
        uint32_t scl_speed_hz = 0;
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

} // namespace i2c

// namespace spi;

namespace pwm {
    struct config_t{
        uint32_t resolution_hz;
        uint32_t period_ticks;
        std::array<uint8_t, 3>pin_num;
    };

    FUNC_RET init(const config_t &config);
    FUNC_RET deinit();

    INLINE_RET apply_pwm_duty(uint8_t phase, float duty);
} // namespace pwm

namespace adc {
    FUNC_RET init();
    FUNC_RET deinit();

    abc_phase_t get_phase_current();

    // private:
} // namespace adc

// }