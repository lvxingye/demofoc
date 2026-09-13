#pragma once

#include "utils/utils.hpp"
#include <array>
#include <cstdint>

inline namespace pltf {
    namespace gpio {
        enum level_t : uint8_t { LOW = 0, HIGH = 1 };

        struct config_t {
            uint64_t pin_bit_mask; /*!< GPIO pin: set with bit mask,
                                      each bit maps to a GPIO */
            enum : uint8_t {
                OFF = 0,
                IN,
                OUT,
                OUT_OD,
                IN_OUT_OD,
                IN_OUT
            } mode;
            uint8_t pull_up_en;
            uint8_t pull_down_en;
            enum : uint8_t {
                DISABLE = 0,
                POSEDGE,
                NEGEDGE,
                DUALEDGE,
                LOW_LEVEL,
                HIGH_LEVEL
            } intr_type;
        };

        FUNC_RET init(const config_t &config);
        // FUNC_RET init(const config_t* &config_ptr,uint8_t num);
        FUNC_RET deinit();

        FUNC_RET set_pin_level(uint8_t pin, level_t level);
        FUNC_RET set_pin_level(uint64_t pin_mask, level_t level);
        FUNC_RET get_pin_level(uint8_t pin, level_t &level);
        FUNC_RET get_pin_level(uint64_t pin_mask, level_t &level);
    } // namespace gpio

    // namespace uart {
    //     FUNC_RET init();
    //     FUNC_RET deinit();

    //     FUNC_RET send_bytes(
    //         const uint8_t mem_addr, const xfr_data_t &data);
    //     FUNC_RET recv_bytes(
    //         const uint8_t mem_addr, const xfr_data_t &data);
    // } // namespace uart

    namespace i2c {
        struct bus_config_t {
            uint8_t sda_pin_num;
            uint8_t scl_pin_num;
        }; // to be added in the future
        struct dev_config_t {
            // uint8_t is_use_10_bit_addr=0;
            uint16_t dev_addr     = 0x0000;
            uint32_t scl_speed_hz = 0;
        };

        FUNC_RET init_bus(const bus_config_t &config);
        FUNC_RET deinit_bus();

        FUNC_RET init_dev(const dev_config_t &config);
        FUNC_RET deinit_dev();

        // wrapper functions for platform iic bus send data
        FUNC_RET send_bytes(
            const uint8_t mem_addr, const xfr_data_t &data);
        // wrapper functions for platform iic bus receive data
        FUNC_RET recv_bytes(
            const uint8_t mem_addr, const xfr_data_t &data);
        FUNC_RET write_mem(const uint8_t mem_addr, const xfr_data_t &data);
        FUNC_RET read_mem(const uint8_t mem_addr, const xfr_data_t &data);

    } // namespace i2c

    // namespace spi;

    namespace pwm {

        enum timer_action_t : uint8_t { STOP = 0, START };
        struct timer_config_t {
            uint32_t resolution_hz;
            uint32_t period_ticks;
            enum : uint8_t {
                PAUSE = 0, // 0x00  00
                UP,        // 0x01  01
                DOWN,      // 0x02  10
                UP_DOWN,   // 0x03  11
            } count_mode;
        };
        struct channel_config_t {
            uint8_t pin_num;
            enum : uint8_t {
                KEEP = 0,
                TOGGLE,
                LOW,
                HIGH,
            } action_on_exceed;
        };

        FUNC_RET init_timer(const timer_config_t &config);
        FUNC_RET add_channel(const channel_config_t &config, uint8_t &ch);
        FUNC_RET deinit();

        FUNC_RET set_timer_state(timer_action_t action);

        FUNC_RET set_channel_duty(uint8_t channel, float duty);
    } // namespace pwm

    namespace adc {

        struct adc_res_t {
            uint8_t ch;
            uint16_t data;
        };

        FUNC_RET add_channel(uint8_t &ch);
        FUNC_RET deinit();

        FUNC_RET get_channel_votage(
            uint8_t ch, std::array<adc_res_t, 5> &data);

        void calibration(uint8_t &ch);

        // private:
    } // namespace adc

    struct platform_config_t {};
    FUNC_RET init_platform_driver(platform_config_t &config);

} // namespace pltf
