#include "BNO080.h"
#include "EventQueue.h"
#include "LowPowerTicker.h"
#include "PinNames.h"
#include "SerialStream.h"
#include "ThisThread.h"
#include "hardware.h"
#include "imc099.h"
#include "math_types.h"
#include <array>
#include <mbed.h>
#include <stdio.h>

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

    debug("\n\n");
  }
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

BufferedSerial serial(USBTX, USBRX, 9600);
SerialStream<BufferedSerial> pc(serial);
BNO080I2C imu{&pc, PB_9, PB_8, D13, D12};

Thread imu_thread;

void imu_task() {
  while (true) {
    while (!imu.begin()) {
      ThisThread::sleep_for(100ms);
    };
    imu.enableReport(BNO080Base::LINEAR_ACCELERATION, 200);
    imu.enableReport(BNO080Base::GAME_ROTATION, 100);

    while (true) {
      imu.updateData();
      ThisThread::sleep_for(1ms);
    }
  }
}

int main() {
  //   std::array<Thread, 3> threads;
  //   DebugMonitor dbg;
  debug("starting up...\n");
  imu_thread.start(imu_task);
  while (true) {
    if (imu.hasNewData(BNO080Base::LINEAR_ACCELERATION)) {
      auto acc = imu.linearAcceleration;
      debug("%f %f %f\n", acc.element(0, 0), acc.element(0, 1),
            acc.element(0, 2));
    }
    if (imu.hasNewData(BNO080Base::GAME_ROTATION)) {
      auto rot = imu.gameRotationVector;
      debug("%f\n", rot.real());
    }
    ThisThread::sleep_for(1s);
  }
};