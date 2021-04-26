#pragma once

#include "imc099.h"
#include <array>
#include <mbed.h>

const size_t NUM_MC = 1;
extern std::array<iMotion::IMC099, NUM_MC> MOTOR_CONTROLLERS;
