#pragma once

#include <cstdint>

#include "algorithm.hpp"
#include "devices/encoder/common.hpp"
#include "platform/common.hpp"
#include "utils/utils.hpp"
#include "utils/configs.hpp"

class foc
{
public:

  // todo: pid controller
  // todo: close loop logic something
  foc ();
  ~foc ();

  ret_val test_correct_phase ();
  ret_val loop (bool);

private:
  float u_dc = 0.0f;

  foc_configs_t configs;

  encoder_t encoder;
  current_sensor_t current_sensor;
  controller_t controller;

//   transforms_t algorithm;
// algorithms shall not be here?
};
