
#pragma once

#include <cstdint>

#include "utils/utils.hpp"

class wave_gen_controller_t {
public:
    wave_gen_controller_t();
    ~wave_gen_controller_t();

    FUNC_RET init();

    INLINE_RET apply_pwm_for_phases(const abc_phase_t &phase); // return radius

private:
    uint8_t is_initialized = false;
};