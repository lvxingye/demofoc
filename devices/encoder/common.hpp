
#pragma once

#include <cstdint>

#include "utils/utils.hpp"

class encoder_t {
public:
    encoder_t()  = default;
    ~encoder_t() = default;

    FUNC_RET init();

    float get_phy_angle(bool is_enable_filter=false); // return radius
    void zero_align();

private:
    uint8_t is_initialized = false;

    float zero_bias = 0.0f;
};