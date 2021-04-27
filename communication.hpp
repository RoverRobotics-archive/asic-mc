#include <array>
#include <mbed.h>
#include <string>
#include <type_traits>

struct StatusMessage {
  std::array<uint32_t, 4> motor_rpm;
  std::array<uint32_t, 3> imu_linear_acceleration;
};

struct InfoMessage {
  std::string version = "0";
  std::string nickname = "Beepboop";
  std::uint64_t serial_number = 777;
};

class MockCommunication {
  Thread thread;
  EventQueue queue;

  void request_info() {
    queue.call_in(50ms, [this]() { on_info({}); });
  };

  void on_info(const InfoMessage &m) {
    char msg[256];
    snprintf(msg, 256, "INFO = {version=%s, nickname=%s, serial_number=%ju}",
             m.version.c_str(), m.nickname.c_str(), m.serial_number);
    write(msg);
  };

  void on_status(const StatusMessage &m) {
    char msg[256];
    snprintf(msg, 256,
             "STATUS = {\nmotor_rpm=[%f, %f, %f, "
             "%f],\nimu_linear_acceleration=%f,%f,%f\n}\n",
             m.motor_rpm[0], m.motor_rpm[1], m.motor_rpm[2], m.motor_rpm[3],
             m.imu_linear_acceleration[0], m.imu_linear_acceleration[1],
             m.imu_linear_acceleration[2]);
    write(msg);
  };

  void set_target_velocity(float transverse_meters_per_second,
                           float rotational_radians_per_second);

public:
  MockCommunication()
      : queue(32 * EVENTS_EVENT_SIZE) {
    queue.call_every(11111ms, [this]() { request_info(); });
    queue.call_every(30ms, [this]() { set_target_velocity(0.03, 0.05); });
    thread.start([this]() { queue.dispatch_forever(); });
  };

  void write(const std::string message) {
    fprintf(stderr, "-----\n%s\n-----\n", message.c_str());
  }
};

MockCommunication communication_service;

// class DebugMonitor {
//   std::array<float, 4> motor_speeds;
//   IMUFrame last_imu;
//   size_t mc_ev = -1;
//   Thread t;
//   EventQueue q;

// public:
//   DebugMonitor() {
//     auto ev = q.event(this, &DebugMonitor::vel_listener, 0);
//     mc_ev = MOTOR_BOARDS[0].add_listener(ev);
//     q.call_every(1s, this, &DebugMonitor::poll_mc, &MOTOR_BOARDS[0]);
//     q.call_every(1s, this, &DebugMonitor::emit_kinematics);
//     t.start([this]() { q.dispatch_forever(); });
//     // todo:
//     // IMU.add_listener(q.event(this, &DebugMonitor::on_imu_data));
//   };

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

//   void on_imu_data(IMUFrame f) { last_imu = f; };

//   void poll_mc(iMotion::IMC099 *mc) {
//     auto req = iMotion::DataFrame::make_register_read(
//         iMotion::MotorControlRegister::MOTOR_SPEED);
//     mc->send(req);
//   };

//   void emit_kinematics() {
//     return; // todo:
//     iMotion::IMC099 &imc099 = MOTOR_BOARDS[0];

//     debug("speed commanded / target:\n");
//     debug("m0\t%f\n", motor_speeds[0]);
//     // debug("m1\t%f\n", motor_speeds[1]);
//     // debug("m2\t%f\n", motor_speeds[2]);
//     // debug("m3\t%f\n", motor_speeds[3]);

//     debug("IMU motion:\n");
//     debug("accel:\t%f,\t%f,\t%f\n", last_imu.linearAcceleration.coords[0],
//           last_imu.linearAcceleration.coords[1],
//           last_imu.linearAcceleration.coords[2]);
//     debug("quat (%f):\t%f\t%f i\t%f j\t%f k\n",
//           norm(last_imu.angularOrientation),
//           last_imu.angularOrientation.real,
//           last_imu.angularOrientation.imag[0],
//           last_imu.angularOrientation.imag[1],
//           last_imu.angularOrientation.imag[2]);

//     debug("\n\n");
//   }
// };