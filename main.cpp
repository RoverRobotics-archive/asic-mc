#include "EventQueue.h"
#include "hardware.h"
#include "imc099.h"
#include "math_types.h"
#include <array>
#include <mbed.h>

void message_received_callback(iMotion::DataFrame df) {
  switch (iMotion::Command(df.command)) {
  case iMotion::Command::REGISTER_READ_REPLY: {
    auto del = iMotion::AnyRegister(df.dataword0);
    auto dval = df.dataword1;
    // debug("%d : %d : %d\n", int(del.app_id), int(del.register_id),
    // int(dval));
    break;
  }
  default: {
    return;
  }
  }
}

// void mc_task() {
//   iMotion::IMC099 &imc099 = MOTOR_BOARDS[0];

//   auto registers = {
//       iMotion::SystemControlRegister::CPU_LOAD,
//       iMotion::SystemControlRegister::CPU_LOAD_PEAK,
//       iMotion::SystemControlRegister::SW_VERSION,
//   };

//   EventQueue q;
//   Event<void(iMotion::DataFrame)> message_received_event =
//       q.event(message_received_callback);

//   imc099.add_listener(message_received_event);

//   while (true) {
//     q.dispatch_for(100ms);
//   }
// }

int main() {
  //   std::array<Thread, 3> threads;
  //   DebugMonitor dbg;
  debug("starting up...\n");
  i2c_imu.frequency(400000);

  IMUManager imu;
  imu.set_interface(&i2c_imu);

  //   threads[0].start(mc_task);
  while (true) {
    ThisThread::sleep_for(100ms);
  }
};