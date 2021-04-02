#include "PeripheralNames.h"
#include "PinNames.h"
#include "imc099.h"

#include "hardware.h"
#include "imu.h"

IMUManager IMU{PF_0, PF_1};
std::array<iMotion::IMC099, NUM_MC> MOTOR_BOARDS{
    {{PD_5, PD_6}}};
    //, {NC, NC}, {NC, NC}, {NC, NC}}};
