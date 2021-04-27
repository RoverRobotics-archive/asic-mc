#include "EventQueue.h"
#include "hardware.h"
#include "imc099.h"
#include "math_types.h"
#include <array>
#include <mbed.h>

class DebugMonitor {
  std::array<float, 4> motor_speeds;
  size_t mc_ev = -1;
  Thread t;
  EventQueue q;

public:
  DebugMonitor() {
    auto ev = q.event(this, &DebugMonitor::vel_listener, 0);
    mc_ev = MOTOR_BOARDS[0].add_listener(ev);
    q.call_every(1s, this, &DebugMonitor::poll_mc, &MOTOR_BOARDS[0]);
    q.call_every(1s, this, &DebugMonitor::emit_kinematics);
    t.start([this]() { q.dispatch_forever(); });
  };

//   ~DebugMonitor() { MOTOR_BOARDS[0].remove_listener(mc_ev); };

//   void vel_listener(size_t i, iMotion::DataFrame df) {
//     switch (df.command) {
//     case iMotion::Command::REGISTER_READ_REPLY: {
//       iMotion::AnyRegister reg(df.dataword0);
//       if (reg ==
//           iMotion::AnyRegister(iMotion::MotorControlRegister::MOTOR_SPEED)) {
//         motor_speeds[i] = df.dataword0 / 16384.0f;
//         break;
//       }
//     }
//     default:
//       break;
//     }
//   };

  void poll_mc(iMotion::IMC099 *mc) {
    auto req = iMotion::DataFrame::make_register_read(
        iMotion::MotorControlRegister::MOTOR_SPEED);
    mc->send(req);
  };

//   void emit_kinematics() {
//     return; // todo:
//     iMotion::IMC099 &imc099 = MOTOR_BOARDS[0];

//     debug("speed commanded / target:\n");
//     debug("m0\t%f\n", motor_speeds[0]);
//     // debug("m1\t%f\n", motor_speeds[1]);
//     // debug("m2\t%f\n", motor_speeds[2]);
//     // debug("m3\t%f\n", motor_speeds[3]);

//     debug("\n\n");
//   }
};

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