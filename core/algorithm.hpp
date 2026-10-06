#pragma once

// #include "utils/configs.hpp"
#include "utils/utils.hpp"

/*
    alpha-beta system: Fixed relative to the stator;the
   alpha-axis corresponds to the horizontal direction dq
   system: Rotates with the rotor coordinate system; the
   q-axis corresponds to the vertical direction phase
   defines: a:=0deg b:=120deg c:=-120deg/240deg Park
   transform: alpha-beta
   -> d-q Inverse Park transform: d-q -> alpha-beta Clarke
   transform: transforms three-phase currents into the
   Cartesian coordinate system (alpha and beta) Inverse
   Clarke transform: transforms the Cartesian coordinate
   system into three-phase currents
*/

// transforms_t static class def
class transforms {
public:
    transforms()  = delete;
    ~transforms() = delete;

    static d_q_t park(alpha_beta_t x, float theta);
    static alpha_beta_t inv_park(d_q_t x, float theta);
    static alpha_beta_t clark(abc_phase_t x);
    static abc_phase_t inv_clark(alpha_beta_t x);
    static abc_phase_t svpwm(alpha_beta_t x, float theta = 0.0f);

private:
    static inline float min(float a, float b);
    static inline float max(float a, float b);
};

class pid_controller_t {
public:
    struct coef_t {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
    };

    struct limit_t {
        float min = -1.0f;
        float max = 1.0f;
    };
    struct terms_t {
        float p = 0.0f;
        float i = 0.0f;
        float d = 0.0f;
    };

    pid_controller_t(float sample_period,
        const coef_t &coef = {0.0f, 0.0f, 0.0f},
        const limit_t &output_limit = {-1.0f, 1.0f},
        const limit_t &integral_limit = {-1.0f, 1.0f});
    ~pid_controller_t() = default;

    void set_coef(const coef_t &coef = {0.0f, 0.0f, 0.0f});
    void set_output_limit(const limit_t &limit);
    void set_integral_limit(const limit_t &limit);
    void reset();
    terms_t get_last_terms() const;
    float pid_flow(float in);

private:
    coef_t coef;
    limit_t output_limit;
    limit_t integral_limit;
    terms_t last_terms;
    float last_input = 0.0f, integral = 0.0f, sample_period = 0.0f;
    bool has_last_input = false;

    static float clamp(float x, const limit_t &limit);

#ifdef BUILD_PID_ANALYSIS
    float t_i = 0.0f,
          tau = 0.0f; // may reserved for analysis
    void update_constants();
#endif
};
