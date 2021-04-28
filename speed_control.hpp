#pragma once
#include <mbed.h>

enum class ControlMode {
  OPEN_LOOP,
  CLOSED_LOOP,
};
const auto N_MOTORS = 4;

enum class RobotMotors : size_t {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3,
};

struct RobotGeometry {
  float intra_axle_distance; // axle distance
  float wheel_base;          // axle length
  float wheel_radius;
  float center_of_mass_x_offset;
  float center_of_mass_y_offset;
};

const RobotGeometry TERRAPIN_GEOMETRY{
    1, 1, 1, 1, 1, // todo
};

struct Velocity {
  double linear;
  double angular;
};
