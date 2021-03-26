#include "PinNames.h"
#include "ThisThread.h"
#include "imc099.h"
#include <array>
#include <cstdint>
#include <cstdio>
#include <mbed.h>
#include "BNO055.h"

void message_received_callback(iMotion::DataFrame df) {

  switch (iMotion::UartCommand(df.command)) {
  case iMotion::UartCommand::REGISTER_READ_REPLY: {
    auto del =  iMotion::AnyRegister(df.dataword0);
    auto dval = df.dataword1;
    printf("%d : %d : %d\n",int(del.app_id), int(del.register_id),int(dval));
    break;
  }
  default: {
    return;
  }
  }
}

void mc_task(){
      iMotion::IMC099 imc099{PD_5, PD_6};

  auto registers = {
      iMotion::SystemControlRegister::CPU_LOAD,
      iMotion::SystemControlRegister::CPU_LOAD_PEAK,
      iMotion::SystemControlRegister::SW_VERSION,
  };
  EventQueue q;

  Event<void(iMotion::DataFrame)> message_received_event =
      q.event(message_received_callback);

  imc099.add_listener(message_received_event);

  while (true) {
    for (auto reg : registers) {
      auto msg = iMotion::DataFrame::make_register_read(reg);
      imc099.send(msg);
    }
    q.dispatch_for(100ms);
  }
}

void imu_task(){
    BNO055 imu{PF_0,PF_1};

        imu.reset();
        DigitalOut led(LED1);

        while(!imu.check()){
            led=!led;
            ThisThread::sleep_for(200ms);
        }
        // can get:
        // absolute orientation
        // angular velocity
        // acceleration vector
        // gravity vector
    imu.setmode(OPERATION_MODE_NDOF);
    while (true){

        imu.get_accel();
        imu.get_angles();
        imu.get_calib();
        imu.get_grv();
        imu.get_gyro();
        imu.get_lia();
        imu.get_mag();
        imu.get_quat();
        imu.get_temp();
        auto x = imu.gyro;
        
    }
}

int main() {
    std::array<Thread, 2> threads;
    threads[0].start(mc_task);
    threads[1].start(imu_task);
    while (true){
    ThisThread::sleep_for(100ms);
    }
};