#include "algorithm.hpp"

#include <cmath>

// scaler version. vector version may be implimented in the future

// abc_phase_t struct
abc_phase_t abc_phase_t::operator+(float s) {
    return {a + s, b + s, c + s};
}
abc_phase_t abc_phase_t::operator+(abc_phase_t s) {
    return {a + s.a, b + s.b, c + s.c};
}
abc_phase_t abc_phase_t::operator*(float s) {
    return {a * s, b * s, c * s};
}
abc_phase_t abc_phase_t::operator*(abc_phase_t s) {
    return {a * s.a, b * s.b, c * s.c};
}

// transforms_t class
// class init deinit func
transforms_t::transforms_t(const foc::configs_t& _configs)
    : configs(_configs) {}
// private helpers
inline float transforms_t::min(float a, float b) {
    return (a < b) ? a : b;
}
inline float transforms_t::max(float a, float b) {
    return (a > b) ? a : b;
}

// public apis
d_q_t transforms_t::park_transform(alpha_beta_t x, float theta) {
    d_q_t y = {.d = 0.0f, .q = 0.0f};
    y.d     = x.alpha * cosf(theta) + x.beta * sinf(theta);
    y.q     = -x.alpha * sinf(theta) + x.beta * cosf(theta);
    return y;
}
alpha_beta_t transforms_t::inv_park_transform(d_q_t x, float theta) {
    alpha_beta_t y = {.alpha = 0.0f, .beta = 0.0f};
    y.alpha        = x.d * cosf(theta) - x.q * sinf(theta);
    y.beta         = x.d * sinf(theta) + x.q * cosf(theta);
    return y;
}
alpha_beta_t transforms_t::clark_transform(abc_phase_t x) {
    alpha_beta_t y = {.alpha = 0.0f, .beta = 0.0f};
    y.alpha        = x.a - 0.5f * x.b - 0.5f * x.c;
    y.beta         = 0.8660254037844386f * (x.b - x.c);   // 0.8660254037844386f=sqrt(3)/2
    return y;
}
abc_phase_t transforms_t::inv_clark_transform(alpha_beta_t x) {
    abc_phase_t y = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    // but why should I multiply 3/2 for each
    // though simplefoc, ds and even codex all do that.
    y.a = x.alpha;                                          // 0.6666666666666667f=2/3
    y.b = -0.5f * x.alpha + 0.8660254037844386f * x.beta;   // 0.8660254037844386f=sqrt(3)/2
    y.c = -0.5f * x.alpha - 0.8660254037844386f * x.beta;

    return y;
}
abc_phase_t transforms_t::svpwm(alpha_beta_t x, float theta) {
    abc_phase_t y    = {.a = 0.0f, .b = 0.0f, .c = 0.0f};
    float       _min = 0.0f, _max = 0.0f, _center;
    // todo/optional:7 segment method segmentation

    y       = inv_clark_transform(x) * (1.0f / configs.u_dc);
    _min    = min(y.a, min(y.b, y.c));
    _max    = max(y.a, max(y.b, y.c));
    _center = 0.5f - (_min + _max) * 0.5f;

    y = y + _center;

    return y;
}


// pid controller
pid_controller_t::pid_controller_t(coef_t& coef, float sample_peroid)
    : coef(coef)
    , sample_freq(1.0f / sample_peroid) {}

float pid_controller_t ::pid_flow(float in) {
    float output = 0.0f;

    if (0.0f == sample_freq) {
        return 0.0f;
    }

    output = (coef.kp + coef.ki * sample_freq + coef.kd * sample_freq) * in +
             -coef.kd * sample_freq * last_input + coef.ki * last_output;

    last_input  = in;
    last_output = output;

    return output;
}
#ifdef BUILD_PID_ANALYSIS
void pid_controller_t::update_constants() {
    t_i = coef.kp / coef.ki;
    tau = coef.kd / coef.kp;
}
#endif
