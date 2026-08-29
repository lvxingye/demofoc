#pragma once

#include <cstdint>

#include "algorithm.hpp"
#include "devices/encoder/common.hpp"
#include "platform/common.hpp"
#include "utils/utils.hpp"

class foc
{
public:
  struct configs_t
  {
    float u_dc;
    uint8_t pole_pairs;
    uint8_t is_swapped = 0; // weather the actual phase is reversed
                            // and weather b and c phase has been swapped
  };

  // todo: pid controller
  // todo: close loop logic something
  foc ();
  ~foc ();

  ret_val test_correct_phase ();
  ret_val loop (bool);

private:
  float u_dc = 0.0f;

  configs_t configs;

  encoder_t encoder;
  current_sensor_t current_sensor;
  controller_t controller;

  transforms_t algorithm;
};
