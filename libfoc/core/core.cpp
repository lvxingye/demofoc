#include "core.hpp"
#include "algorithm.hpp"
#include "platform/common.hpp"
#include "utils/utils.hpp"

// foc class
ret_val_t foc_t::init(void) {
    platform_config_t config = {};
    pltf::init_platform_driver(config);

    encoder.init();
    pwm_controller.init();

    u_dc               = 10.0f;
    configs.pole_pairs = 7;
    // todo: adc init

    return OK;
}

ret_val_t foc_t::test_correct_phase() {
    float shaft_angle_0 = 0.0f, shaft_angle_1 = 0.0f, d_shaft_angle = 0.0f;

#define TEST_1_PWM 0.5f
    // initial state
    pwm_controller.apply_pwm_for_phases({.a = 0.0f, .b = 0.0f, .c = 0.0f});
    // sleep(2)

    // conventionally, u4(100) should be alianed to positive
    // direction of x-axis ? i have some doubt about that
    pwm_controller.apply_pwm_for_phases(
        {.a = TEST_1_PWM, .b = 0.0f, .c = 0.0f});
    // sleep(2)
    shaft_angle_0 = encoder.get_phy_angle();

    // u6(110)
    pwm_controller.apply_pwm_for_phases(
        {.a = TEST_1_PWM, .b = TEST_1_PWM, .c = 0.0f});
    d_shaft_angle = encoder.get_phy_angle() - shaft_angle_0;

#undef TEST_1_PWM

    if ((d_shaft_angle) < 0.0f) {
        // swap b and c phases
        configs.is_swapped = 1;

        d_shaft_angle *= -1.0f;
    }

    if (0.0f == d_shaft_angle) {
        return ERROR;
    }
    configs.pole_pairs = (uint8_t)(1.047197551196598f / (d_shaft_angle) +
                                   0.5f); //(pi/3)/theta

    return OK;
}

ret_val_t foc_t::loop(bool is_open_loop) {
    float angle                 = 0.0f;
    d_q_t target_d_q            = {.d = 0.0f, .q = 5.0f};
    alpha_beta_t alpha_beta_res = {.alpha = 0.0f, .beta = 0.0f};
    abc_phase_t final_duty      = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    angle = encoder.get_phy_angle() * configs.pole_pairs;

    alpha_beta_res = transforms_t::inv_park_transform(target_d_q, angle);
    if (0.0f == u_dc) {
        return ERROR;
    }
    alpha_beta_res = alpha_beta_res * (1.0f / u_dc); // normalization
    final_duty     = transforms_t::svpwm(alpha_beta_res);

    pwm_controller.apply_pwm_for_phases(final_duty);

    if (!is_open_loop) {
        // todo: timer sync, current read

        // todo: park transform

        // todo: clark transform

        // todo: pid controller
    }

    return OK;
}
