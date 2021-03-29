#pragma once

#include "BNO055.h"
#include "imc099.h"
#include <array>

extern BNO055 IMU;

// anticlockwise from front left
extern std::array<iMotion::IMC099, 4> MOTOR_BOARDS;
