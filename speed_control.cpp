#include "hardware.h"
#include "EventQueue.h"
#include "Span.h"
#include "hardware.h"
#include "imc099.h"
#include <array>
#include <mbed.h>

enum class ControlMode {
  OPEN_LOOP,
  CLOSED_LOOP,
};
const auto N_MOTORS = 4;

enum class robot_motors : size_t {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3,
};

struct robot_geometry {
  float intra_axle_distance; // axle distance
  float wheel_base;          // axle length
  float wheel_radius;
  float center_of_mass_x_offset;
  float center_of_mass_y_offset;
};

const robot_geometry TERRAPIN_GEOMETRY{
    1, 1, 1, 1, 1, // todo
};

const auto M_PI = 3.1415926535;

const double MOTOR_RPM_PER_DUTY = 1; // todo
const double RADIAN_PER_SECOND_PER_RPM = 60.0 / (2 * M_PI);

// xyz = fwd, left, up

Vec<4> ROBOT_METER_PER_SEC_PER_DUTY =
    (TERRAPIN_GEOMETRY.wheel_radius * RADIAN_PER_SECOND_PER_RPM *
     MOTOR_RPM_PER_DUTY) *
    Vec<4>{1, 1, 1, 1} / 4;

Vec<4> ROBOT_RADIAN_PER_SEC_PER_DUTY =
    (TERRAPIN_GEOMETRY.wheel_base * TERRAPIN_GEOMETRY.wheel_radius *
     RADIAN_PER_SECOND_PER_RPM * MOTOR_RPM_PER_DUTY) *
    Vec<4>{-1, +1, -1, +1} / 4;

struct Velocity {
  double transverse;
  double radial;
};

Vec<4> duties_from_velocity(Velocity v) {
  return v.transverse * pseudoinverse(ROBOT_METER_PER_SEC_PER_DUTY) +
         v.radial * pseudoinverse(ROBOT_RADIAN_PER_SEC_PER_DUTY);
};

Velocity velocity_from_duties(Vec<4> m) {
  Velocity v;
  v.transverse = inner(m, ROBOT_METER_PER_SEC_PER_DUTY);
  v.radial = inner(m, ROBOT_RADIAN_PER_SEC_PER_DUTY);
  return v;
};

float min_meters_per_radian;
float max_motor_speed;

float max_rotation_vel;
float max_rotational_accel;

float max_linear_accel;
float max_linear_vel;

struct SpeedControllerState {
  std::array<float, N_MOTOR> d_vel_d_wheel;
  std::array<float, N_MOTOR> d_rot_d_wheel;
  float wheel_perp_friction;
  float wheel_inline_friction;
};

struct SpeedDecayDelay {};

// struct SpeedControllerState {
//   std::array<int16_t, 4> target_speeds;
//   std::array<int16_t, 4> actual_speeds;
//   std::array<int16_t, 4> motor_currents;
// };

// const auto MOTOR_OVERRIDE_DURATION = 1s;
// const auto MOTOR_REPORT_PERIOD = 100ms;

// class SpeedController {
//   EventQueue q;

//   Event<void()> report_event;

//   Event<void()> motor_override_timeout_event;

//   void report_status(){};

//   std::array<uint16_t, 4> motor_speeds;

//   SpeedController()
//       : report_event(q.event(this, &SpeedController::report_status))
//       , motor_override_timeout_event(
//             q.event(this, &SpeedController::on_motor_override_timeout)) {
//     report_event = q.event(this, &SpeedController::report_status);
//     report_event.period(MOTOR_REPORT_PERIOD);
//     report_event.post();

//     for (auto i = 0; i < MOTOR_BOARDS.size(); ++i) {
//       MOTOR_BOARDS[i].add_listener(
//           q.event(this, &SpeedController::on_motor_message, i));
//     };
//     q.background({});
//   }
//   void update_linear_acceleration() {
//     //
//   }
//   void update_rotational_velocity() {
//     //
//   }
//   void update_angular_position() {
//     //
//   }
//   void update_motor_velocity(size_t which, float vel) {
//     //
//   }
//   void on_motor_override_timeout() {
//     for (auto i = 0; i < MOTOR_BOARDS.size(); ++i) {
//       auto df = iMotion::DataFrame::make_motor_control(0);
//       MOTOR_BOARDS[i].send(df);
//     }
//   }

//   void on_motor_message(size_t which_motor, iMotion::DataFrame df) {
//     switch (df.command) {
//     case iMotion::Command::SET_TARGET_SPEED_REPLY: {
//       uint16_t sequencer_state = df.dataword0;
//       uint16_t motor_speed_i16 = df.dataword1;
//       motor_speeds[which_motor] = motor_speed_i16;
//     }
//     default:
//       return;
//     }
//   };

//   void set_report_period_ms(uint16_t millis) {
//     report_event.period(millis * 1ms);
//   };

//   void set_motor_targets(std::array<int16_t, 4> target_speeds) {
//     motor_override_timeout_event.cancel();
//     for (auto i = 0; i < 4; ++i) {
//       auto df = iMotion::DataFrame::make_motor_control(target_speeds[i]);
//       MOTOR_BOARDS[i].send(df);
//     }
//     motor_override_timeout_event.delay(MOTOR_OVERRIDE_DURATION);
//     motor_override_timeout_event.post();
//   };
// };
