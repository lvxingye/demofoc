
#pragma once

#include <array>
#include <cstdint>

#include "utils/utils.hpp"

#define PHASE_NUM 3

class pwm_controller_t {
public:
    pwm_controller_t()  = default;
    ~pwm_controller_t() = default;

    FUNC_RET init();

    // return radius
    FUNC_RET apply_pwm_for_phases(const abc_phase_t &phase);
    FUNC_RET reverse_phase_order();

private:
    uint8_t is_initialized = false;

    std::array<uint8_t, 3> phases;
    std::array<uint8_t, 4> pins = {25, 26, 27, 33}; // phase a,b,c; en
};