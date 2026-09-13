
#pragma once

#include <cstdint>

#include "utils/utils.hpp"

class current_sense {
public:
    current_sense()  = default;
    ~current_sense() = default;

    FUNC_RET init();

    float get_phase_current(bool is_enable_filter=false); // return radius

private:
};