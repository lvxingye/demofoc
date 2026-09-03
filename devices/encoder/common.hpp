
#pragma once

#include <cstdint>

#include "utils/utils.hpp"

class encoder_t {
public:
    encoder_t();
    ~encoder_t();

    FUNC_RET init();

    float get_phy_angle(); // return radius

private:
    uint8_t is_initialized = false;
};