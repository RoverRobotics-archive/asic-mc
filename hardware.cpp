#include "BNO055.h"
#include "PeripheralNames.h"
#include "PinNames.h"
#include "imc099.h"

#include "hardware.h"

BNO055 IMU{PF_0, PF_1};
std::array<iMotion::IMC099, 4> MOTOR_BOARDS{
    {{PD_5, PD_6}, {NC, NC}, {NC, NC}, {NC, NC}}};
