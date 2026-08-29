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
class transforms_t {
public:
    transforms_t()  = delete;
    ~transforms_t() = delete;

    static d_q_t park_transform(alpha_beta_t x, float theta);
    static alpha_beta_t inv_park_transform(d_q_t x, float theta);
    static alpha_beta_t clark_transform(abc_phase_t x);
    static abc_phase_t inv_clark_transform(alpha_beta_t x);
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

    pid_controller_t(coef_t &coef, float sample_peroid);
    ~pid_controller_t();

    float pid_flow(float in);

private:
    coef_t coef;
    float last_input = 0.0f, last_output = 0.0f,
          sample_freq =
              0.0f; // use freqency instead of sampling
                    // peroid, avoid division

#ifdef BUILD_PID_ANALYSIS
    float t_i = 0.0f,
          tau = 0.0f; // may reserved for analysis
    void update_constants();
#endif
};