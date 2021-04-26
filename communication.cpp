
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