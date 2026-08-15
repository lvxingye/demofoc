#pragma once

#include <cstdint>
#include "utils/utils.hpp"

//types
struct alpha_beta_t
{
    float alpha;
    float beta;
};
struct d_q_t
{
    float d;
    float q;
};
struct abc_phase_t
{
    float a;
    float b;
    float c;

    // for readiness and vector
    abc_phase_t operator+(float s);
    abc_phase_t operator+(abc_phase_t s);
    abc_phase_t operator*(float s);
    abc_phase_t operator*(abc_phase_t s);

};

class foc {
public:
    struct configs_t{
        float u_dc;
    };
    
    class encoder_t{
    public:
        encoder_t() = default;
        ~encoder_t() = default;

        float get_phy_angle();
    private:

    };

    class current_sensor_t{
    public:
        current_sensor_t() = default;
        ~current_sensor_t() = default;

        void get_phase_current();
    private:
    };

    class controller_t{
    public:
        controller_t() = default;
        ~controller_t() = default;

        void apply_phase_pwm(abc_phase_t);
    };

    // todo: pid controller
    // todo: close loop logic something
    foc();
    ~foc();

    ret_val loop(bool);
private:
    float u_dc=0.0f;

    configs_t configs;

    encoder_t encoder;
    current_sensor_t current_sensor;
    controller_t controller;

    algorithm_t algorithm;

};
