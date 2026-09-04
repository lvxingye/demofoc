
// esp32 header
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
// #include "hal/mcpwm_types.h"
#include "esp_task_wdt.h"
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
            .pull_up_en   = (gpio_pullup_t)config.pull_up_en,
            .pull_down_en = (gpio_pulldown_t)config.pull_down_en,
            .intr_type    = (gpio_int_type_t)config.intr_type,
        };

        switch (config.mode) {
        case config_t::OFF:
        case config_t::IN:
        case config_t::OUT:
            _gpio_config.mode = (gpio_mode_t)(config.mode);
            break;
        case config_t::OUT_OD:
            _gpio_config.mode = GPIO_MODE_OUTPUT_OD;
            break;
        case config_t::IN_OUT_OD:
            _gpio_config.mode = GPIO_MODE_INPUT_OUTPUT_OD;
            break;
        case config_t::IN_OUT:
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

    FUNC_RET set_pin_level(uint8_t pin, level_t level) {
        if (ESP_OK != gpio_set_level((gpio_num_t)pin, (uint32_t)level)) {
            return ERROR;
        }
        return OK;
    }
    // FUNC_RET set_pin_level(uint64_t pin_mask, uint8_t
    // level);
    FUNC_RET get_pin_level(uint8_t pin, level_t &level) {

        level = (level_t)gpio_get_level((gpio_num_t)pin);
        return OK;
    }
    // FUNC_RET get_pin_level(uint64_t pin_mask, uint8_t
    // &level);
} // namespace gpio

namespace i2c {
    // assume only one device and only one bus would be used
    // here. so call functions init_dev and init_bus, not
    // add_device and add_bus
#define TIMEOUT 1
#define IS_10BIT_ADDR(x)                                                  \
    (((x) >> 8) ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7)

    // handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    uint8_t is_bus_initialized = false;

    // init and deinit functions
    FUNC_RET init_bus(const bus_config_t &config) {
        i2c_master_bus_config_t i2c_bus_config = {
            .i2c_port          = -1,
            .sda_io_num        = (gpio_num_t)config.sda_pin_num,
            .scl_io_num        = (gpio_num_t)config.scl_pin_num,
            .clk_source        = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags             = {.enable_internal_pullup = true},
        };

        if (ESP_OK != i2c_new_master_bus(&i2c_bus_config, &bus_handle)) {
            return ERROR;
        }
        is_bus_initialized = true;

        return OK;
    }

    // clang-format off-next-line
    FUNC_RET deinit_bus() { return OK; }

    FUNC_RET init_dev(const dev_config_t &config) {
        if (!is_bus_initialized) {
            return ERROR;
        }

        i2c_device_config_t _dev_config = {
            .dev_addr_length = IS_10BIT_ADDR(config.dev_addr),
            .device_address  = config.dev_addr,
            .scl_speed_hz    = config.scl_speed_hz,
        };
        if (ESP_OK != i2c_master_bus_add_device(
                          bus_handle, &_dev_config, &dev_handle)) {
            return ERROR;
        }
        return OK;
    }

    // clang-format off-next-line
    FUNC_RET deinit_dev() { return OK; }

    // data transfer functions
    // wrapper functions for platform iic bus send data
    FUNC_RET send_bytes(const uint8_t mem_addr, const xfr_data_t &data) {
        if (ESP_OK != i2c_master_transmit(dev_handle, data.data_buffer,
                          data.len, TIMEOUT)) {
            return ERROR;
        }
        return OK;
    }
    FUNC_RET recv_bytes(const uint8_t mem_addr, const xfr_data_t &data) {
        if (ESP_OK != i2c_master_receive(dev_handle, data.data_buffer,
                          data.len, TIMEOUT)) {
            return ERROR;
        }
        return OK;
    }
    FUNC_RET write_mem(const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(dev_handle, &mem_addr, 1, TIMEOUT);
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
    FUNC_RET read_mem(const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(dev_handle, &mem_addr, 1, TIMEOUT);
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

#define TIMER_GROUP_ID 0
#define MAX_PWM_CH_NUM 5

    mcpwm_timer_handle_t timer_handle;
    struct phase_handle_t {
        mcpwm_oper_handle_t op_handle;
        mcpwm_cmpr_handle_t cmp_handle;
        mcpwm_gen_handle_t gen_handle;
    };
    std::array<phase_handle_t, MAX_PWM_CH_NUM> phase_handle;

    mcpwm_timer_config_t timer_config;

    uint8_t channel_nums = 0;
    float half_duty_coef = 1.0f; // not sure weather is correct

    // private helper functions
    FUNC_RET assign_new_channel(uint8_t &ch) {
        if (MAX_PWM_CH_NUM <= channel_nums + 1) {
            return ERROR;
        }
        ch = ++channel_nums - 1;
        return OK;
    }

    FUNC_RET init_timer(const timer_config_t &config) {
        timer_config = {
            .group_id      = TIMER_GROUP_ID,
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = config.resolution_hz,
            .count_mode    = (mcpwm_timer_count_mode_t)config.count_mode,
            .period_ticks  = config.period_ticks,
            .intr_priority = 0,
            .flags         = {.update_period_on_empty = 1},
        };

        // !!verified.!! See:
        // https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32/api-reference/peripherals/mcpwm.html#_CPPv420mcpwm_timer_config_t
        // hardware will automatically divide by 2
        if (config.count_mode == timer_config_t::UP_DOWN) {
            half_duty_coef = 0.5f;
            timer_config.resolution_hz *= 2;
        }

        mcpwm_timer_event_callbacks_t timer_callbacks;

        if (ESP_OK != mcpwm_new_timer(&timer_config, &timer_handle)) {
            return ERROR;
        }
        // no timer callbacks are used currently
        // if (ESP_OK !=
        //     mcpwm_timer_register_event_callbacks(
        //         timer_handle, &timer_callbacks, nullptr)) {
        //     return ERROR;
        // }

        // enable timer but not start for now
        if (ESP_OK != mcpwm_timer_enable(timer_handle)) {
            return ERROR;
        }
        return OK;
    }
    FUNC_RET deinit();

    FUNC_RET add_channel(const channel_config_t &config, uint8_t &ch) {
        mcpwm_operator_config_t operator_config = {
            .group_id = TIMER_GROUP_ID,
        };
        mcpwm_comparator_config_t comparator_config = {
            .flags = {.update_cmp_on_tez = 1},
        };
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = config.pin_num,
        };

        if (OK != assign_new_channel(ch)) {
            return ERROR;
        }

        if (ESP_OK != mcpwm_new_operator(&operator_config,
                          &(phase_handle[ch].op_handle))) { // oper init
            return ERROR;
        }
        if (ESP_OK != mcpwm_new_comparator(phase_handle[ch].op_handle,
                          &comparator_config,
                          &(phase_handle[ch].cmp_handle))) { // cmpr init
            return ERROR;
        }
        if (ESP_OK != mcpwm_new_generator(phase_handle[ch].op_handle,
                          &generator_config,
                          &(phase_handle[ch].gen_handle))) { // gen init
            return ERROR;
        }
        // could causing problem if not using up & down

        mcpwm_generator_action_t gen_action;
        switch (config.action_on_exceed) {
        case channel_config_t::LOW:
            gen_action = MCPWM_GEN_ACTION_LOW;
            break;
        case channel_config_t::HIGH:
            gen_action = MCPWM_GEN_ACTION_HIGH;
            break;
        case channel_config_t::TOGGLE:
            gen_action = MCPWM_GEN_ACTION_TOGGLE;
            break;
        case channel_config_t::KEEP:
        default:
            gen_action = MCPWM_GEN_ACTION_KEEP;
            break;
        }

        // could causing problem if not using up & down
        // may apply a fix when is truly affectted later.
        if (timer_config.count_mode & 0x1) {
            mcpwm_generator_set_action_on_compare_event(
                phase_handle[ch].gen_handle,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                    phase_handle[ch].cmp_handle, MCPWM_GEN_ACTION_HIGH));
            // for testing, not use gen_action for now
        }
        if (timer_config.count_mode & 0x2) {
            mcpwm_generator_set_action_on_compare_event(
                phase_handle[ch].gen_handle,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                    phase_handle[ch].cmp_handle, MCPWM_GEN_ACTION_LOW));
            // for testing, not use gen_action for now
        }

        if (ESP_OK !=
            mcpwm_operator_connect_timer(phase_handle[ch].op_handle,
                timer_handle)) { // op connect timer
            return ERROR;
        }

        return OK;
    }

    FUNC_RET set_timer_state(timer_action_t action) {
        mcpwm_timer_start_stop_cmd_t cmd = MCPWM_TIMER_STOP_EMPTY;

        switch (action) {
        case START:
            cmd = MCPWM_TIMER_START_NO_STOP;
            break;
        case STOP:
        default:
            cmd = MCPWM_TIMER_STOP_EMPTY;
            break;
        }

        if (ESP_OK != mcpwm_timer_start_stop(timer_handle, cmd)) {
            return ERROR;
        }
        return OK;
    }

    FUNC_RET set_channel_duty(uint8_t channel, float duty) {
        // constraint to [0.0,1.0]
        if (0.0f > duty) {
            duty = 0.0f;
        }
        if (1.0f < duty) {
            duty = 1.0f;
        }

        if (channel >= channel_nums) {
            return ERROR;
        }
        if (ESP_OK != mcpwm_comparator_set_compare_value(
                          phase_handle[channel].cmp_handle,
                          (uint32_t)(half_duty_coef * duty *
                                     timer_config.period_ticks))) {
            return ERROR;
        };
        return OK;
    }
} // namespace pwm

// initialize common peripherals like timers and buses
FUNC_RET pltf::init_platform_driver(platform_config_t &config) {

    esp_task_wdt_deinit();
    
    // initialization for gpio pins will be done by device driver
    // respectively

    i2c::bus_config_t i2c_bus_config = {
        .sda_pin_num = GPIO_NUM_21,
        .scl_pin_num = GPIO_NUM_22,
    };
    if (ret_val_t::OK != i2c::init_bus(i2c_bus_config)) {
        return ERROR;
    }

    pwm::timer_config_t timer_config = {
        .resolution_hz = 40000000,
        .period_ticks  = 1000,
        .count_mode    = pwm::timer_config_t::UP_DOWN,
    };
    if (ret_val_t::OK != pwm::init_timer(timer_config)) {
        return ERROR;
    }

    // spi bus initialization, if used

    // uart initialization

    return OK;
}