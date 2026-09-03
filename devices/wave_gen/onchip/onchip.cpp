
#include "devices/wave_gen/common.hpp"

#include "platform/common.hpp"
#include "utils/utils.hpp"
#include <cstdint>
// inline namespace onchip {

#define USE_PWM_EN_PIN // may move to config later?

FUNC_RET pwm_controller_t::init() {
#ifdef  USE_PWM_EN_PIN
    gpio::config_t gpio_config = {
        .pin_bit_mask = (uint64_t)(1) << pins[3],
        .mode         = gpio::config_t::OUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = gpio::config_t::DISABLE,
    };
    gpio::init(gpio_config);
    gpio::set_pin_level(pins[3], gpio::HIGH); // enable pwm output
#endif
    for (uint8_t i = 0; i < PHASE_NUM; i++) {
        pwm::add_channel(
            {.pin_num             = pins[i],
                .action_on_exceed = pwm::channel_config_t::HIGH},
            phases[i]);
    }

    pwm::set_timer_state(pwm::timer_action_t::START);

    return OK;
}
FUNC_RET pwm_controller_t::apply_pwm_for_phases(
    const abc_phase_t &phase) {

    pwm::set_channel_duty(phases[0], phase.a);
    pwm::set_channel_duty(phases[1], phase.b);
    pwm::set_channel_duty(phases[2], phase.c);

    return OK;
}
// }