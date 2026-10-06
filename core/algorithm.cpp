#include "algorithm.hpp"

#include <cmath>

// scaler version. vector version may be implimented in the future

// abc_phase_t struct
alpha_beta_t alpha_beta_t::operator*(float s) {
    return {.alpha = alpha * s, .beta = beta * s};
}
alpha_beta_t alpha_beta_t::operator*(alpha_beta_t s) {
    return {.alpha = alpha * s.alpha, .beta = beta * s.beta};
}

// abc_phase_t struct
abc_phase_t abc_phase_t::operator+(float s) {
    return {.a = a + s, .b = b + s, .c = c + s};
}
abc_phase_t abc_phase_t::operator+(abc_phase_t s) {
    return {.a = a + s.a, .b = b + s.b, .c = c + s.c};
}
abc_phase_t abc_phase_t::operator*(float s) {
    return {.a = a * s, .b = b * s, .c = c * s};
}
abc_phase_t abc_phase_t::operator*(abc_phase_t s) {
    return {.a = a * s.a, .b = b * s.b, .c = c * s.c};
}

// transforms_t class
// class init deinit func
// transforms_t::transforms_t()=default;
// transforms_t::~transforms_t()=default;

// private helpers
inline float transforms::min(float a, float b) { return (a < b) ? a : b; }
inline float transforms::max(float a, float b) { return (a > b) ? a : b; }

// public apis
/**
 * @brief Park transform: alpha-beta frame to d-q (rotating) frame.
 * @param x     Input in the stationary alpha-beta frame.
 * @param theta Electrical angle [rad].
 * @return Output in the rotating d-q frame.
 */
d_q_t transforms::park(alpha_beta_t x, float theta) {
    d_q_t y = {.d = 0.0f, .q = 0.0f};
    y.d     = x.alpha * cosf(theta) + x.beta * sinf(theta);
    y.q     = -x.alpha * sinf(theta) + x.beta * cosf(theta);
    return y;
}

/**
 * @brief Inverse Park transform: d-q (rotating) frame to alpha-beta frame.
 * @param x     Input in the rotating d-q frame.
 * @param theta Electrical angle [rad].
 * @return Output in the stationary alpha-beta frame.
 */
alpha_beta_t transforms::inv_park(d_q_t x, float theta) {
    alpha_beta_t y = {.alpha = 0.0f, .beta = 0.0f};
    y.alpha        = x.d * cosf(theta) - x.q * sinf(theta);
    y.beta         = x.d * sinf(theta) + x.q * cosf(theta);
    return y;
}

/**
 * @brief Clarke transform: three-phase abc frame to alpha-beta frame
 * (power invariant).
 * @param x Input three-phase quantities.
 * @return Output in the alpha-beta frame.
 */
alpha_beta_t transforms::clark(abc_phase_t x) {
    alpha_beta_t y = {.alpha = 0.0f, .beta = 0.0f};
    y.alpha        = x.a - 0.5f * x.b - 0.5f * x.c;
    y.beta =
        0.8660254037844386f * (x.b - x.c); // 0.8660254037844386f=sqrt(3)/2
    return y;
}

/**
 * @brief Inverse Clarke transform: alpha-beta frame to three-phase abc
 * frame.
 * @param x Input in the alpha-beta frame.
 * @return Output three-phase quantities.
 */
abc_phase_t transforms::inv_clark(alpha_beta_t x) {
    abc_phase_t y = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

    // but why should I multiply 3/2 for each
    // though simplefoc, ds and even codex all do that.
    y.a = x.alpha; // 0.6666666666666667f=2/3
    y.b = -0.5f * x.alpha +
          0.8660254037844386f * x.beta; // 0.8660254037844386f=sqrt(3)/2
    y.c = -0.5f * x.alpha - 0.8660254037844386f * x.beta;

    return y;
}

/**
 * @brief Space-vector PWM (centered/3rd-harmonic injection variant).
 *
 * Computes the centered abc duty cycles from an alpha-beta voltage vector.
 * The result is normalized to the [0, 1] range for PWM duty cycles.
 *
 * @param x     Normalized input (0-1) voltage vector in the alpha-beta
 * frame.
 * @param theta Electrical angle [rad] (unused in this implementation).
 * @return Centered three-phase duty cycles in the [0, 1] range.
 */
abc_phase_t transforms::svpwm(alpha_beta_t x, float theta) {
    abc_phase_t y = {.a = 0.0f, .b = 0.0f, .c = 0.0f};
    float _min = 0.0f, _max = 0.0f, _center;
    // todo/optional:7 segment method segmentation

    y       = inv_clark(x);
    _min    = min(y.a, min(y.b, y.c));
    _max    = max(y.a, max(y.b, y.c));
    _center = 0.5f - (_min + _max) * 0.5f;

    y = y + _center;

    return y;
}

// pid controller
pid_controller_t::pid_controller_t(float sample_period,
    const coef_t &coef, const limit_t &output_limit,
    const limit_t &integral_limit)
    : coef(coef), output_limit(output_limit),
      integral_limit(integral_limit), sample_period(sample_period) {}

/**
 * @brief Update the PID coefficients.
 * @param coef New kp/ki/kd coefficients.
 */
void pid_controller_t::set_coef(const coef_t &coef) { this->coef = coef; }

void pid_controller_t::set_output_limit(const limit_t &limit) {
    output_limit = limit;
}

void pid_controller_t::set_integral_limit(const limit_t &limit) {
    integral_limit = limit;
    integral       = clamp(integral, integral_limit);
}

void pid_controller_t::reset() {
    last_input     = 0.0f;
    integral       = 0.0f;
    last_terms     = {};
    has_last_input = false;
}

pid_controller_t::terms_t pid_controller_t::get_last_terms() const {
    return last_terms;
}

float pid_controller_t::clamp(float x, const limit_t &limit) {
    if (x < limit.min) {
        return limit.min;
    }
    if (x > limit.max) {
        return limit.max;
    }
    return x;
}

/**
 * @brief Compute the PID output for a single sample.
 * @param in Current error/input value.
 * @return Limited controller output, or 0 if the sample period is invalid.
 */
float pid_controller_t ::pid_flow(float in) {
    if (0.0f >= sample_period) {
        return 0.0f;
    }

    integral = clamp(integral + coef.ki * in * sample_period,
        integral_limit);

    const float derivative =
        has_last_input ? (in - last_input) / sample_period : 0.0f;
    last_terms = {
        .p = coef.kp * in,
        .i = integral,
        .d = coef.kd * derivative,
    };

    float output = last_terms.p + last_terms.i + last_terms.d;
    output       = clamp(output, output_limit);

    last_input     = in;
    has_last_input = true;
    return output;
}
#ifdef BUILD_PID_ANALYSIS
void pid_controller_t::update_constants() {
    t_i = coef.kp / coef.ki;
    tau = coef.kd / coef.kp;
}
#endif
