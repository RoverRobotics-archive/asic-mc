#pragma once

#include "imc099.h"
#include "imu.h"
#include <array>

extern IMUManager IMU;

const size_t NUM_MC = 1;
// anticlockwise from front left
extern std::array<iMotion::IMC099, NUM_MC> MOTOR_BOARDS;
