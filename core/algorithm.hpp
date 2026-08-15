#include "core.hpp"

/*
    alpha-beta system: Fixed relative to the stator;the alpha-axis corresponds to the horizontal direction
    dq system: Rotates with the rotor coordinate system; the q-axis corresponds to the vertical direction
    phase defines: a:=0deg b:=120deg c:=-120deg/240deg
    Park transform: alpha-beta -> d-q
    Inverse Park transform: d-q -> alpha-beta
    Clarke transform: transforms three-phase currents into the Cartesian coordinate system (alpha and beta)
    Inverse Clarke transform: transforms the Cartesian coordinate system into three-phase currents
*/

//algorithm_t class def
class algorithm_t{
public:
    algorithm_t(const foc::configs_t& _configs);
    ~algorithm_t();

    d_q_t park_transform(alpha_beta_t x, float theta);
    alpha_beta_t inv_park_transform(d_q_t x, float theta);
    alpha_beta_t clark_transform(abc_phase_t x);
    abc_phase_t inv_clark_transform(alpha_beta_t x);
    abc_phase_t svpwm(alpha_beta_t x, float theta=0.0f);

private:
    const foc::configs_t& configs;
    inline float min(float a, float b);
    inline float max(float a, float b);

};
