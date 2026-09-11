#pragma once

#include "platform/common.hpp"
#include "utils/utils.hpp"
#include <cstdint>

#define AS5600_TAG "device.as5600"
#define I2C_DELAY 1000

// slave addresses
#define AS5600_ADDRESS_7B (0x36) // 0b0110110  	(0x6C) //0b0110110
#define AS5600_ADDRESS (AS5600_ADDRESS_7B << 1)
#define AS5600_READ (AS5600_ADDRESS | 0x01)
#define AS5600_WRITE (AS5600_ADDRESS | 0x00)
