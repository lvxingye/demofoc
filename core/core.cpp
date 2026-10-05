#include "core.hpp"
#include "algorithm.hpp"
#include "platform/common.hpp"
#include "utils/utils.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <unistd.h>

#include <cstddef>

pid_controller_t current(1000.0f, {0.15f,0.01f,0.01f});

// debuging only!!
float raw_to_current(float raw) {
    // clang-format off
    return (raw / 4096.0f * 3.3f - 1849.5f / 4096.0f * 3.3f) / 50.0f / 0.01f;
    // clang-format on
}
// foc class
ret_val_t foc_t::init(void) {
    uint8_t ch = 0;

    platform_config_t config = {};
    pltf::init_platform_driver(config);

    encoder.init();
    pwm_controller.init();
    adc::add_channel(ch);

    u_dc               = 1.0f;
    configs.pole_pairs = 7.0f;
    // todo: adc init

    return OK;
}

ret_val_t foc_t::test_correct_phase() {
    float shaft_angle_0 = 0.0f, d_shaft_angle = 0.0f;
    float current_read = 0.0f, last_read = 0.0f;
    d_q_t target_d_q            = {.d = 0.2f, .q = 0.0f};
    alpha_beta_t alpha_beta_res = {.alpha = 0.0f, .beta = 0.0f};
    abc_phase_t final_duty      = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    // initial state
    pwm_controller.apply_pwm_for_phases({.a = 0.0f, .b = 0.0f, .c = 0.0f});
    usleep(static_cast<useconds_t>(500 * 1000));

    // conventionally, u4(100) should be alianed to positive
    // direction of x-axis ? i have some doubt about that

    // swipe from 0-60deg(electrical angle)
    for (float ele_angle = 0.0f; ele_angle <= std::numbers::pi;
        ele_angle += std::numbers::pi / 30.0f) {
        alpha_beta_res = transforms::inv_park(target_d_q, ele_angle);
        final_duty     = transforms::svpwm(alpha_beta_res);
        pwm_controller.apply_pwm_for_phases(final_duty);
        usleep(static_cast<useconds_t>(100 * 1000));

        if (0.0f == ele_angle) {
            usleep(static_cast<useconds_t>(900 * 1000));
            encoder.zero_align(); // align at start
            shaft_angle_0 = encoder.get_phy_angle();
        }

        // check if is zero-crossing
        current_read = encoder.get_phy_angle();
        if (std::numbers::pi < current_read) {
            current_read -= 2.0f * std::numbers::pi;
        }
        last_read = current_read;
        printf("\n");
    }
    d_shaft_angle = last_read - shaft_angle_0;
    printf("d angle: %0.3f~%0.3f\n", shaft_angle_0, last_read);

    if ((d_shaft_angle) < 0.0f) {
        // swap b and c phases
        pwm_controller.reverse_phase_order();
        configs.is_swapped = 1;
        gpio::set_pin_level((uint8_t)2, pltf::gpio::HIGH);

        d_shaft_angle *= -1.0f;
    }

    if (0.0f == d_shaft_angle) {
        return ERROR;
    }
    // configs.pole_pairs =
    //     std::floor(1.047197551196598f / (d_shaft_angle)); //(pi/3)/theta
    printf("test pole pairs: %0.2f\n", configs.pole_pairs);

    return OK;
}

ret_val_t foc_t::loop(bool is_open_loop) {
    static d_q_t want_d_q       = {.d = 0.0f, .q = 0.2f};
    static d_q_t target_d_q     = {.d = 0.0f, .q = 0.0f};
    float angle                 = 0.0f;
    alpha_beta_t alpha_beta_res = {.alpha = 0.0f, .beta = 0.0f};
    abc_phase_t final_duty      = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    angle = encoder.get_phy_angle(true);

    // target_d_q.q = -(angle - PI) / PI * 0.2f;
    // printf("%f,",angle-PI);

    alpha_beta_res =
        transforms::inv_park(target_d_q, angle * configs.pole_pairs);
    if (0.0f == u_dc) {
        return ERROR;
    }
    alpha_beta_res = alpha_beta_res * (1.0f / u_dc); // normalization
    final_duty     = transforms::svpwm(alpha_beta_res);

    pwm_controller.apply_pwm_for_phases(final_duty);

    if (!is_open_loop) {
        std::array<adc::adc_res_t, 5> _adc_data = {};
        adc::get_channel_votage(0, _adc_data);

        float ia = raw_to_current(_adc_data[0].data),
              ib = raw_to_current(_adc_data[1].data), ic = -ia - ib;
        // todo: timer sync, current read

        alpha_beta_t clark_res = transforms::clark({
            .a = ia,
            .b = ib,
            .c = ic,
        });
        // todo: clark transform

        d_q_t park_res =
            transforms::park(clark_res, angle * configs.pole_pairs);
        // todo: park transform

        target_d_q.q = current.pid_flow(-park_res.q + want_d_q.q);
        // target_d_q.d = current.pid_flow(-park_res.d + want_d_q.d);
        // target_d_q.q = (want_d_q.q - park_res.q) * 0.15f;
        target_d_q.d = (want_d_q.d - park_res.d) * 0.15f;

        printf("%f,%f,%f,%f,", park_res.q, park_res.d, target_d_q.q,target_d_q.d);
        printf("%f,%f,%f", ia, ib, ic);
        // todo: pid controller
    }
    printf("\n");

    return OK;
}
