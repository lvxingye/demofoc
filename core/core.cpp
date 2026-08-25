#include "core.hpp"
#include "algorithm.hpp"

// foc class
foc::foc()
    : algorithm(configs) {
        // todo :device init

        // todo :config init
        // configs
        // todo :algorithm init

        //...
    };

foc::~foc() {
    // todo :device deinit

    // todo :config deinit

    // todo :algorithm deinit

    //...
};

ret_val foc::test_correct_phase() {
    float shaft_angle_0 = 0.0f, shaft_angle_1 = 0.0f, d_shaft_angle = 0.0f;

#define TEST_1_PWM 0.5f
    // initial state
    controller.apply_phase_pwm({.a = 0.0f, .b = 0.0f, .c = 0.0f});
    // sleep(2)

    // conventionally, u4(100) should be alianed to positive direction of x-axis ?
    // i have some doubt about that
    controller.apply_phase_pwm({.a = TEST_1_PWM, .b = 0.0f, .c = 0.0f});
    // sleep(2)
    shaft_angle_0 = encoder.get_phy_angle();

    // u6(110)
    controller.apply_phase_pwm({.a = TEST_1_PWM, .b = TEST_1_PWM, .c = 0.0f});
    d_shaft_angle = encoder.get_phy_angle() - shaft_angle_0;

#undef TEST_1_PWM

    if ((d_shaft_angle) < 0.0f) {
        // swap b and c phases
        configs.is_swapped = 1;

        d_shaft_angle *= -1.0f;
    }

    if (d_shaft_angle) {
        return OK;
    }
    configs.pole_pairs =
        (uint8_t)(3.141592653589793f / 3.0f / (d_shaft_angle) + 0.5f);   //(pi/3)/theta

    return ERROR;
}

ret_val foc::loop(bool is_open_loop = true) {
    float        angle          = 0.0f;
    d_q_t        target_d_q     = {.q = 0.0f, .d = 0.0f};
    alpha_beta_t alpha_beta_res = {.alpha = 0.0f, .beta = 0.0f};
    abc_phase_t  abc_phase_res  = {.a = 0.0f, .b = 0.0f, .c = 0.0f},
                 final_duty     = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    angle = foc::encoder.get_phy_angle();

    alpha_beta_res = algorithm.inv_park_transform(target_d_q, angle);
    final_duty     = algorithm.svpwm(alpha_beta_res);

    controller.apply_phase_pwm(final_duty);

    if (!is_open_loop) {
        // todo: timer sync, current read

        // todo: park transform

        // todo: clark transform

        // todo: pid controller
    }
}
