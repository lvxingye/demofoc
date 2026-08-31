// library header
#include "driver/i2c_types.h"
#include "platform/common.hpp"
#include "soc/gpio_num.h"
#include "utils/utils.hpp"

// esp32 header
#include "driver/i2c_master.h"
#include <cstdint>

namespace i2c {
    // assume only one device and only one bus would be used here.
    // so call functions init_dev and init_bus, not add_device and add_bus
#define TIMEOUT 1
#define IS_10BIT_ADDR(x) (((x) >> 8) ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7)

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
            .flags             = {.enable_internal_pullup = true},
        };

        if (ret_val_t::OK !=
            (ret_val_t)i2c_new_master_bus(&i2c_bus_config, &bus_handle)) {
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

        return (ret_val_t)i2c_master_bus_add_device(
            bus_handle, &_dev_config, &dev_handle);
    }

    // clang-format off-next-line
    INLINE_RET deinit_dev() { return OK; }

    // data transfer functions
    // wrapper functions for platform iic bus send data
    INLINE_RET send_bytes(const uint8_t mem_addr, const xfr_data_t &data) {
        return (ret_val_t)i2c_master_transmit(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
    }
    INLINE_RET recv_bytes(const uint8_t mem_addr, const xfr_data_t &data) {
        return (ret_val_t)i2c_master_transmit(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
    }
    INLINE_RET write_mem(const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(dev_handle, &mem_addr, 1, TIMEOUT);
        if (ret != OK) {
            return ERROR;
        }
        ret = i2c_master_transmit(
            dev_handle, data.data_buffer, data.len, TIMEOUT);
        if (ret != OK) {
            return ERROR;
        }

        return OK;
    }
    INLINE_RET read_mem(const uint8_t mem_addr, const xfr_data_t &data) {
        uint32_t ret = 0;

        ret = i2c_master_transmit(dev_handle, &mem_addr, 1, TIMEOUT);
        if (ret != OK) {
            return ERROR;
        }
        ret =
            i2c_master_receive(dev_handle, data.data_buffer, data.len, TIMEOUT);
        if (ret != OK) {
            return ERROR;
        }

        return OK;
    }
    // clang-format off: wip
    // clang-format on
} // namespace i2c
