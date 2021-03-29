#include "BNO055.h"
#include "hardware.h"
#include "imc099.h"
#include <array>
#include <mbed.h>

void message_received_callback(iMotion::DataFrame df) {

  switch (iMotion::Command(df.command)) {
  case iMotion::Command::REGISTER_READ_REPLY: {
    auto del = iMotion::AnyRegister(df.dataword0);
    auto dval = df.dataword1;
    printf("%d : %d : %d\n", int(del.app_id), int(del.register_id), int(dval));
    break;
  }
  default: {
    return;
  }
  }
}

void mc_task() {
  iMotion::IMC099 &imc099 = MOTOR_BOARDS[0];

  auto registers = {
      iMotion::SystemControlRegister::CPU_LOAD,
      iMotion::SystemControlRegister::CPU_LOAD_PEAK,
      iMotion::SystemControlRegister::SW_VERSION,
  };

  EventQueue q;
  //   // this is one way to periodically request data:
  // void request_imotion(iMotion::IMC099 *imc) {
  //   auto req = iMotion::DataFrame::make_register_read(
  //       iMotion::SystemControlRegister::CPU_LOAD);
  //   imc->send(req);
  // }
  //   auto ev = q.event(&request_imotion, &imc099);
  //   ev.period(2s);
  //   ev.post();

  Event<void(iMotion::DataFrame)> message_received_event =
      q.event(message_received_callback);

  imc099.add_listener(message_received_event);

  while (true) {
    // for (auto reg : registers) {
    //   auto msg = iMotion::DataFrame::make_register_read(reg);
    //   imc099.send(msg);
    // }
    q.dispatch_for(100ms);
  }
}

void imu_task() {
  BNO055 &imu = IMU;

  imu.reset();
  DigitalOut led(LED1);

  while (!imu.check()) {
    led = !led;
    ThisThread::sleep_for(200ms);
  }
  // can get:
  // absolute orientation
  // angular velocity
  // acceleration vector
  // gravity vector
  imu.setmode(OPERATION_MODE_NDOF);
  while (true) {
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
  while (true) {
    ThisThread::sleep_for(100ms);
  }
};