
// esp32 header
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "esp_err.h"

// foc library header
#include "platform/common.hpp"
#include "utils/utils.hpp"

#include <array>
#include <cstdint>

namespace gpio {

    FUNC_RET init(const config_t &config) {
        gpio_config_t _gpio_config = {
            .pin_bit_mask = config.pin_bit_mask,
            .mode         = GPIO_MODE_DISABLE,
            .pull_up_en   = (gpio_pullup_t)0,
            .pull_down_en = (gpio_pulldown_t)0,
            .intr_type    = (gpio_int_type_t)config.intr_type};

        switch (config.mode) {
        case off:
        case in:
        case out:
            _gpio_config.mode = (gpio_mode_t)(config.mode);
            break;
        case out_od:
            _gpio_config.mode = GPIO_MODE_OUTPUT_OD;
            break;
        case in_out_od:
            _gpio_config.mode = GPIO_MODE_INPUT_OUTPUT_OD;
            break;
        case in_out:
            _gpio_config.mode = GPIO_MODE_INPUT_OUTPUT;
            break;
        default:
            _gpio_config.mode = GPIO_MODE_DISABLE;
            break;
        }

        if (ESP_OK != gpio_config(&_gpio_config)) {
            return ERROR;
        }

        return OK;
    }
    // FUNC_RET init(const config_t* &config_ptr,uint8_t num);
    FUNC_RET deinit();

    INLINE_RET set_pin_level(uint8_t pin, level level) {
        if (ESP_OK !=
            gpio_set_level((gpio_num_t)pin, (uint32_t)level)) {
            return ERROR;
        }
        return OK;
    }
    // INLINE_RET set_pin_level(uint64_t pin_mask, uint8_t
    // level);
    INLINE_RET get_pin_level(uint8_t pin, uint8_t &level) {

        level = gpio_get_level((gpio_num_t)pin);
        return OK;
    }
    // INLINE_RET get_pin_level(uint64_t pin_mask, uint8_t
    // &level);
} // namespace gpio

namespace i2c {
    // assume only one device and only one bus would be used
    // here. so call functions init_dev and init_bus, not
    // add_device and add_bus
#define TIMEOUT 1
#define IS_10BIT_ADDR(x)                                        \
    (((x) >> 8) ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7)

    // handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    uint8_t is_bus_initialized = false;

    // init and deinit functions
    INLINE_RET init_bus(const bus_config_t &config) {
        i2c_master_bus_config_t i2c_bus_config = {
            .i2c_port          = -1,
            .sda_io_num        = GPIO_NUM_21,
            .scl_io_num        = GPIO_NUM_22,
            .clk_source        = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = {.enable_internal_pullup = true},
        };

        if (ESP_OK !=
            i2c_new_master_bus(&i2c_bus_config, &bus_handle)) {
            return ERROR;
        }
        is_bus_initialized = true;

        return OK;
    }

    // clang-format off-next-line
    INLINE_RET deinit_bus() { return OK; }

    INLINE_RET init_dev(const dev_config_t &config) {
        if (!is_bus_initialized) {
            return ERROR;
        }

        i2c_device_config_t _dev_config = {
            .dev_addr_length = IS_10BIT_ADDR(config.dev_addr),
            .device_address  = config.dev_addr,
            .scl_speed_hz    = config.scl_speed_hz,
        };
        if (ESP_OK != i2c_master_bus_add_device(bus_handle,
                          &_dev_config, &dev_handle)) {
            return ERROR;
        }
        return OK;
    }

    // clang-format off-next-line
    INLINE_RET deinit_dev() { return OK; }

    // data transfer functions
    // wrapper functions for platform iic bus send data
    INLINE_RET send_bytes(
        const uint8_t mem_addr, const xfr_data_t &data) {
        if (ESP_OK != i2c_master_transmit(dev_handle,
                          data.data_buffer, data.len, TIMEOUT)) {
            return ERROR;
        }
        return OK;
    }
    INLINE_RET recv_bytes(
        const uint8_t mem_addr, const xfr_data_t &data) {
        if (ESP_OK != i2c_master_receive(dev_handle,
                          data.data_buffer, data.len, TIMEOUT)) {
            return ERROR;
        }
        return OK;
    }
    INLINE_RET write_mem(
        const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(
            dev_handle, &mem_addr, 1, TIMEOUT);
        if (ret != ESP_OK) {
            return ERROR;
        }
        ret = i2c_master_transmit(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
        if (ret != ESP_OK) {
            return ERROR;
        }

        return OK;
    }
    INLINE_RET read_mem(
        const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(
            dev_handle, &mem_addr, 1, TIMEOUT);
        if (ret != ESP_OK) {
            return ERROR;
        }
        ret = i2c_master_receive(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
        if (ret != ESP_OK) {
            return ERROR;
        }

        return OK;
    }
    // clang-format off: wip
    // clang-format on
} // namespace i2c

namespace pwm {

    mcpwm_timer_handle_t timer_handle;
    struct phase_handle_t {
        mcpwm_oper_handle_t op_handle;
        mcpwm_cmpr_handle_t cmp_handle;
        mcpwm_gen_handle_t gen_handle;
    };
    std::array<phase_handle_t, 3> phase_handle;

    config_t config = {};

    FUNC_RET init(const config_t &config) {
        pwm::config = config;
        // we use up and down mode to gen symmetric wave

        // pwm timer init
        mcpwm_timer_config_t pwm_config = {
            .group_id      = 0,
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = config.resolution_hz,
            .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
            .period_ticks  = 2 * config.resolution_hz,
            .intr_priority = 0,
            .flags         = {.update_period_on_empty = 1},
        };
        mcpwm_timer_event_callbacks_t timer_callbacks;
        if (ESP_OK !=
            mcpwm_new_timer(&pwm_config, &timer_handle)) {
            return ERROR;
        }
        // no timer callbacks are used currently
        // if (ESP_OK !=
        //     mcpwm_timer_register_event_callbacks(
        //         timer_handle, &timer_callbacks, nullptr)) {
        //     return ERROR;
        // }

        // phase init
        mcpwm_operator_config_t operator_config = {
            .group_id = pwm_config.group_id,
        };
        mcpwm_comparator_config_t comparator_config = {
            .flags = {.update_cmp_on_tez = 1},
        };
        mcpwm_generator_config_t generator_config;
        for (uint8_t i = 0; i < PHASE_NUM; i++) {
            if (ESP_OK !=
                mcpwm_new_operator(&operator_config,
                    &(phase_handle[i].op_handle))) { // oper init
                return ERROR;
            }
            if (ESP_OK !=
                mcpwm_new_comparator(phase_handle[i].op_handle,
                    &comparator_config,
                    &(phase_handle[i]
                            .cmp_handle))) { // cmpr init
                return ERROR;
            }
            generator_config.gen_gpio_num = config.pin_num[i];
            if (ESP_OK !=
                mcpwm_new_generator(phase_handle[i].op_handle,
                    &generator_config,
                    &(phase_handle[i].gen_handle))) { // gen init
                return ERROR;
            }

            // gen high level voltage in the middle
            mcpwm_generator_set_action_on_compare_event(
                phase_handle[i].gen_handle,
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    phase_handle[i].cmp_handle,
                    MCPWM_GEN_ACTION_HIGH));
            mcpwm_generator_set_action_on_compare_event(
                phase_handle[i].gen_handle,
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_DOWN,
                    phase_handle[i].cmp_handle,
                    MCPWM_GEN_ACTION_LOW));

            if (ESP_OK !=
                mcpwm_operator_connect_timer(
                    phase_handle[i].op_handle,
                    timer_handle)) { // op connect timer
                return ERROR;
            }
        }

        // enable and start timer
        mcpwm_timer_enable(timer_handle);
        mcpwm_timer_start_stop(
            timer_handle, MCPWM_TIMER_START_NO_STOP);

        return OK;
    }

    FUNC_RET deinit();

    INLINE_RET apply_pwm_duty(uint8_t phase, float duty) {
        if (ESP_OK !=
            mcpwm_comparator_set_compare_value(
                phase_handle[phase].cmp_handle,
                (uint32_t)(0.5f * duty * config.period_ticks))) {
            return ERROR;
        };
        return OK;
    }
} // namespace pwm
