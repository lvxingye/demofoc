#pragma once

#include "utils/utils.hpp"
#include <cstdint>

// namespace foc::pltf{
class i2c {
    i2c();
    ~i2c();

    ret_val send_byte(const uint8_t dev_addr, const uint8_t data);
    ret_val send_bytes(const uint8_t dev_addr, const trans_data_t& data);
    ret_val recv_byte(const uint8_t dev_addr, const uint8_t data);
    ret_val recv_bytes(const uint8_t dev_addr, const trans_data_t& data);
};
// class spi;
class uart {

    uart();
    ~uart();

    ret_val send_bytes(const uint8_t dev_addr, const trans_data_t& data);
    ret_val recv_bytes(const uint8_t dev_addr, const trans_data_t& data);
};
// class pwm {
//     pwm();
//     ~pwm();
// };

class current_sensor_t {
public:
    current_sensor_t()  = default;
    ~current_sensor_t() = default;

    void get_phase_current();

private:
};
class controller_t {
public:
    controller_t()  = default;
    ~controller_t() = default;

    void apply_phase_pwm(abc_phase_t);
};




// }