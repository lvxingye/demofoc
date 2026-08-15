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

ret_val foc::loop(bool is_open_loop = true) {
    float        angle          = 0.0f;
    d_q_t        target_d_q     = {.q = 0.0f, .d = 0.0f};
    alpha_beta_t alpha_beta_res = {.alpha = 0.0f, .beta = 0.0f};
    abc_phase_t  abc_phase_res  = {.a = 0.0f, .b = 0.0f, .c = 0.0f},
                 final_duty     = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    angle = foc::encoder.get_phy_angle();

    alpha_beta_res = algorithm.inv_park_transform(target_d_q, angle);
    final_duty=algorithm.svpwm(alpha_beta_res);

    controller.apply_phase_pwm(final_duty);

    if (!is_open_loop){
        // todo: timer sync, current read

        // todo: park transform

        // todo: clark transform

        // todo: pid controller 
    }


}
