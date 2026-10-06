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

namespace {
constexpr float kCurrentTargetA    = 0.20f;
constexpr float kCurrentSoftLimitA = 0.20f;
constexpr float kVoltageLimit      = 0.04f;
constexpr float kSamplePeriodS     = 0.001f;
constexpr uint16_t kAdcInvalidRaw  = 16;
constexpr uint16_t kAdcWarmupLoops = 64;

float clampf(float x, float min, float max) {
    if (x < min) {
        return min;
    }
    if (x > max) {
        return max;
    }
    return x;
}

d_q_t limit_dq_vector(d_q_t x, float limit) {
    const float mag = sqrtf(x.d * x.d + x.q * x.q);
    if (mag > limit && mag > 0.0f) {
        const float scale = limit / mag;
        x.d *= scale;
        x.q *= scale;
    }
    return x;
}

float same_direction_guard(float measured, float output) {
    if (fabsf(measured) > kCurrentSoftLimitA &&
        measured * output > 0.0f) {
        return 0.0f;
    }
    return output;
}

bool read_phase_raw(
    const std::array<adc::adc_res_t, 5> &data, uint16_t &raw_a,
    uint16_t &raw_b) {
    bool has_a = false;
    bool has_b = false;

    for (const auto &sample : data) {
        if (sample.data <= kAdcInvalidRaw) {
            continue;
        }
        if (!has_a && sample.ch == 0) {
            raw_a = sample.data;
            has_a = true;
        } else if (!has_b && sample.ch == 3) {
            raw_b = sample.data;
            has_b = true;
        }
    }

    return has_a && has_b;
}

float raw_to_current(float raw, float zero_raw) {
    return ((raw - zero_raw) / 4096.0f * 3.3f) / 50.0f / 0.01f;
}
} // namespace

pid_controller_t current(kSamplePeriodS, {0.10f, 0.40f, 0.0001f},
    {-kVoltageLimit, kVoltageLimit}, {-0.02f, 0.02f});

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
    static d_q_t want_d_q       = {.d = 0.0f, .q = kCurrentTargetA};
    static d_q_t target_d_q     = {.d = 0.0f, .q = 0.0f};
    static uint16_t adc_warmup  = 0;
    static float raw_a_zero_sum = 0.0f, raw_b_zero_sum = 0.0f;
    static float raw_a_zero = 1849.5f, raw_b_zero = 1849.5f;
    static bool adc_zero_ready = false;
    float angle                 = 0.0f;
    alpha_beta_t alpha_beta_res = {.alpha = 0.0f, .beta = 0.0f};
    abc_phase_t final_duty      = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    angle = encoder.get_phy_angle(true);

    // target_d_q.q = -(angle - PI) / PI * 0.2f;
    // printf("%f,",angle-PI);

    target_d_q = limit_dq_vector(target_d_q, kVoltageLimit);
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

        uint16_t raw_a = 0;
        uint16_t raw_b = 0;
        if (!read_phase_raw(_adc_data, raw_a, raw_b)) {
            target_d_q = {};
            current.reset();
            printf("adc_wait");
            printf("\n");
            return OK;
        }

        if (!adc_zero_ready) {
            raw_a_zero_sum += raw_a;
            raw_b_zero_sum += raw_b;
            adc_warmup++;
            target_d_q = {};
            current.reset();
            if (adc_warmup >= kAdcWarmupLoops) {
                raw_a_zero = raw_a_zero_sum / adc_warmup;
                raw_b_zero = raw_b_zero_sum / adc_warmup;
                adc_zero_ready = true;
            }
            printf("adc_zero,%u,%u,%u,%f,%f", adc_warmup, raw_a,
                raw_b, raw_a_zero, raw_b_zero);
            printf("\n");
            return OK;
        }

        float ia = raw_to_current(raw_a, raw_a_zero),
              ib = raw_to_current(raw_b, raw_b_zero),
              ic = -ia - ib;
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

        // Current sensors / phase convention make positive q voltage produce
        // negative measured q current, so close the loop on the flipped q axis.
        const d_q_t sense_d_q = {
            .d = park_res.d,
            .q = -park_res.q,
        };

        target_d_q.q = current.pid_flow(want_d_q.q - sense_d_q.q);
        target_d_q.d =
            clampf((want_d_q.d - sense_d_q.d) * 0.05f,
                -kVoltageLimit, kVoltageLimit);
        target_d_q.q = same_direction_guard(sense_d_q.q, target_d_q.q);
        target_d_q.d = same_direction_guard(sense_d_q.d, target_d_q.d);
        target_d_q   = limit_dq_vector(target_d_q, kVoltageLimit);

        const auto terms = current.get_last_terms();
        printf("%f,%f,%f,%f,", sense_d_q.q, sense_d_q.d, target_d_q.q,
            target_d_q.d);
        printf("%f,%f,%f,%f,%f,%f", ia, ib, ic, terms.p, terms.i,
            terms.d);
        // todo: pid controller
    } else {
        target_d_q.q = -(angle - PI) / PI * kVoltageLimit;
        target_d_q.d = 0.0f;
    }
    printf("\n");

    return OK;
}
