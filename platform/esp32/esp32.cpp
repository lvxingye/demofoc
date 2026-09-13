
// esp32 header
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"

#include "esp_adc/adc_continuous.h"

// #include "hal/mcpwm_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

// foc library header
#include "esp32.hpp"
#include "hal/adc_types.h"
#include "platform/common.hpp"
#include "utils/utils.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
// #include <cstdio>
#include <cstring>
#include <string>

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
#define TIMEOUT 10000
#define IS_10BIT_ADDR(x)                                                  \
    (((x) >> 8) ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7)

    // handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    uint8_t is_bus_initialized = false;

    // init and deinit functions
    FUNC_RET init_bus(const bus_config_t &config) {
        esp_err_t driver_ret                   = ESP_OK;
        i2c_master_bus_config_t i2c_bus_config = {
            .i2c_port          = -1,
            .sda_io_num        = (gpio_num_t)config.sda_pin_num,
            .scl_io_num        = (gpio_num_t)config.scl_pin_num,
            .clk_source        = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags             = {.enable_internal_pullup = false},
        };

        driver_ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
        if (ESP_OK != driver_ret) {
            ESP_LOGE("paltf::i2c::init_bus", "init error");
            return ERROR;
        }
        is_bus_initialized = true;

        return OK;
    }

    // clang-format off-next-line
    FUNC_RET deinit_bus() { return OK; }

    FUNC_RET init_dev(const dev_config_t &config) {
        esp_err_t driver_ret = ESP_OK;

        if (!is_bus_initialized) {
            return ERROR;
        }

        i2c_device_config_t _dev_config = {
            .dev_addr_length = IS_10BIT_ADDR(config.dev_addr),
            .device_address  = config.dev_addr,
            .scl_speed_hz    = config.scl_speed_hz,
        };
        driver_ret = i2c_master_bus_add_device(
            bus_handle, &_dev_config, &dev_handle);
        if (ESP_OK != driver_ret) {
            ESP_LOGE("paltf::i2c::init_dev", "init error");
            return ERROR;
        }
        return OK;
    }

    // clang-format off-next-line
    FUNC_RET deinit_dev() { return OK; }

    // data transfer functions
    // wrapper functions for platform iic bus send data
    FUNC_RET send_bytes(const uint8_t mem_addr, const xfr_data_t &data) {
        esp_err_t driver_ret = i2c_master_transmit(
            dev_handle, data.data_buffer, data.len, TIMEOUT);

        if (ESP_OK != driver_ret) {
            ESP_LOGE("paltf::i2c::send_bytes", "send error");
            return ERROR;
        }
        return OK;
    }
    FUNC_RET recv_bytes(const uint8_t mem_addr, const xfr_data_t &data) {
        esp_err_t driver_ret = i2c_master_receive(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
        if (ESP_OK != driver_ret) {
            ESP_LOGE("paltf::i2c::recv_bytes", "recv error");
            return ERROR;
        }
        return OK;
    }
    FUNC_RET write_mem(const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(dev_handle, &mem_addr, 1, TIMEOUT);
        if (ret != ESP_OK) {
            ESP_LOGE("paltf::i2c::write_mem", "write error");
            return ERROR;
        }
        ret = i2c_master_transmit(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
        if (ret != ESP_OK) {
            ESP_LOGE("paltf::i2c::write_mem", "write error");
            return ERROR;
        }

        return OK;
    }
    FUNC_RET read_mem(const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(dev_handle, &mem_addr, 1, TIMEOUT);
        if (ret != ESP_OK) {
            ESP_LOGE("paltf::i2c::read_mem", "read error");
            // std::string str(mem_addr);
            // ESP_LOGE("paltf::i2c::read_mem",str.c_str())
            return ERROR;
        }
        ret = i2c_master_receive(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
        if (ret != ESP_OK) {
            ESP_LOGE("paltf::i2c::read_mem", "read error");
            return ERROR;
        }
        // printf("get mem addr: 0x%02X \nget
        // data:0x%02X\n",mem_addr,(uint8_t)*(data.data_buffer));

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

// work in progress
// and haven't done much abstract yet!!!
// APIs are under testing and learning!!!
namespace adc {
    static adc_continuous_evt_data_t adc_raw_buffer = {};
    adc_continuous_handle_t adc_handle;
    uint8_t adc_channels_used = 0;

    static bool IRAM_ATTR _on_conv_done(adc_continuous_handle_t handle,
        const adc_continuous_evt_data_t *edata, void *user_data) {
        // printf("isr");
        // todo: lock acquire 
        std::memcpy(&adc_raw_buffer, edata,
            sizeof(adc_continuous_evt_data_t)); // copy adc buffer
        // todo: lock release

        return true;
    }
    FUNC_RET add_channel(uint8_t &ch) {

        if (MAX_ADC_CHANNEL <= adc_channels_used) {
            return ERROR;
        }

        // shall not cache data for too long ?
        adc_continuous_handle_cfg_t adc_handle_config = {
            .max_store_buf_size =
                SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 4, // double buffer
            .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2,
            .flags           = {.flush_pool = 1},
        };
        ESP_ERROR_CHECK(
            adc_continuous_new_handle(&adc_handle_config, &adc_handle));

        std::array<adc_digi_pattern_config_t, 2> adc_pattern = {};
        adc_pattern[0]                                       = {
            .atten     = ADC_ATTEN_DB_12,
            .channel   = ADC_CHANNEL_0, // sensor vp
            .unit      = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        adc_pattern[1] = {
            .atten     = ADC_ATTEN_DB_12,
            .channel   = ADC_CHANNEL_3, // sensor vn
            .unit      = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        };
        // for debug !!!
        adc_continuous_config_t adc_config = {
            .pattern_num    = 2,
            .adc_pattern    = adc_pattern.data(),
            .sample_freq_hz = 40000,
            .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        };
        adc_continuous_config(adc_handle, &adc_config);

        adc_continuous_evt_cbs_t adc_callbacks = {
            .on_conv_done = _on_conv_done,
            .on_pool_ovf  = nullptr,
        };
        adc_continuous_register_event_callbacks(
            adc_handle, &adc_callbacks, nullptr);

        adc_continuous_start(adc_handle);

        ch = adc_channels_used++;
        return OK;
    }
    FUNC_RET deinit(uint8_t ch) {
        if (ch >= adc_channels_used) {
            return ERROR;
        }
        ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
        return OK;
    }

    FUNC_RET get_channel_votage(uint8_t ch,std::array<adc_res_t,5> &data) {
        std::array<adc_continuous_data_t,5> parsed={};
        uint32_t samples;
        adc_continuous_parse_data(adc_handle,
            adc_raw_buffer.conv_frame_buffer, adc_raw_buffer.size, parsed.data(),
            &samples);
        for (uint8_t i=0;i<5;i++){
            data[i].ch=parsed[i].channel;
            data[i].data=parsed[i].raw_data;
        }
        return OK;
    }

    void calibration(uint8_t &ch) {}

} // namespace adc

// initialize common peripherals like timers and buses
FUNC_RET pltf::init_platform_driver(platform_config_t &config) {

    esp_task_wdt_deinit();

    // initialization for gpio pins will be done by device driver
    // respectively

    i2c::bus_config_t i2c_bus_config = {
        .sda_pin_num = GPIO_NUM_19,
        .scl_pin_num = GPIO_NUM_18,
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

    gpio::config_t led_config = {
        .pin_bit_mask = (uint64_t)(1) << 2,
        .mode         = gpio::config_t::OUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = gpio::config_t::DISABLE,
    };
    gpio::init(led_config);
    // spi bus initialization, if used

    // uart initialization

    return OK;
}