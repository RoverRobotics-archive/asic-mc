#include "speed_control.hpp"
#include "EventQueue.h"
#include "Span.h"
#include "ThisThread.h"
#include "hardware.h"
#include "imc099.h"
#include "math_types.h"
#include "pid.hpp"
#include "serial_api.h"
#include <algorithm>
#include <array>
#include <mbed.h>

const auto M_PI = 3.1415926535;

const double MOTOR_RPM_PER_DUTY = 1; // todo
const double RADIAN_PER_SECOND_PER_RPM = 60.0 / (2 * M_PI);

Vec<4> LINEAR_METER_PER_SEC_PER_DUTY =
    (TERRAPIN_GEOMETRY.wheel_radius * RADIAN_PER_SECOND_PER_RPM *
     MOTOR_RPM_PER_DUTY) *
    Vec<4>{1, 1, 1, 1} / 4;

Vec<4> ANGULAR_RADIAN_PER_SEC_PER_DUTY =
    (TERRAPIN_GEOMETRY.wheel_base * TERRAPIN_GEOMETRY.wheel_radius *
     RADIAN_PER_SECOND_PER_RPM * MOTOR_RPM_PER_DUTY) *
    Vec<4>{-1, +1, -1, +1} / 4;

Vec<4> duties_from_velocity(Velocity v) {
  return v.linear * pseudoinverse(LINEAR_METER_PER_SEC_PER_DUTY) +
         v.angular * pseudoinverse(ANGULAR_RADIAN_PER_SEC_PER_DUTY);
};

Velocity velocity_from_duties(Vec<4> m) {
  Velocity v;
  v.linear = inner(m, LINEAR_METER_PER_SEC_PER_DUTY);
  v.angular = inner(m, ANGULAR_RADIAN_PER_SEC_PER_DUTY);
  return v;
};

// todo: steering limits
const double max_angular_vel = 10.0, max_linear_vel = 10.0,
             min_turning_radius = 10.0, max_motor_speed = 10.0;

void send_motor_targets(Vec<4> motor_targets) {
  auto safe_motor_targets =
      clamp(motor_targets, Vec<4>(-max_motor_speed), Vec<4>(-max_motor_speed));

  for (auto i = 0; i < MOTOR_CONTROLLERS.size(); ++i) {
    auto d = iMotion::DataFrame::make_motor_control(
        safe_motor_targets[i] / numeric_limits<uint16_t>::max());
    MOTOR_CONTROLLERS[i].send(d);
  }
}

class SpeedController {
  std::array<Thread, 4> motor_communication_threads;

  Vec<4> motor_measured_speed;
  Vec<4> motor_command_duty;             // todo
  PIDFilter<Vec<2>> velocity_filter{{}}; // use default parameters
  ControlMode mode;

  void motor_task(size_t i) {
    while (true) {
      auto rq = iMotion::DataFrame::make_motor_control(motor_command_duty[i]);
      MOTOR_CONTROLLERS[i].send(rq);
      ThisThread::sleep_for(50ms);
    }
  };

  void on_dataframe(size_t i, iMotion::DataFrame d){
      // todo: update measured speed
  };

  void use_open_loop() { mode = ControlMode::OPEN_LOOP; }
  void use_closed_loop() {
    mode = ControlMode::CLOSED_LOOP;
    velocity_filter.reset();
  }

  void request_velocity(Velocity v, ControlMode mode) {
    auto safe_linear = limit_abs(v.linear, max_linear_vel);
    auto safe_angular = limit_abs(v.angular, max_angular_vel);

    if (min_turning_radius) {
      safe_angular =
          limit_abs(safe_angular, abs(safe_linear) / min_turning_radius);
    }

    Velocity safe_velocity{safe_linear, safe_angular};

    Velocity target_velocity;
    switch (mode) {
    case ControlMode::OPEN_LOOP: {
      target_velocity = safe_velocity;
    } break;
    case ControlMode::CLOSED_LOOP: {
      // todo: integrate imu into velocity estimate
      Velocity measured_vel = velocity_from_duties(motor_command_duty);
      Vec<2> target_vel_vec2{safe_velocity.linear, safe_velocity.angular};
      auto result_vec2 = velocity_filter.tick(target_vel_vec2, measured_vel);
      target_velocity.linear = result_vec2[0];
      target_velocity.angular = result_vec2[1];
    } break;
    default:
      __builtin_unreachable();
    }

    auto motor_targets = duties_from_velocity(target_velocity);
    auto safe_motor_targets = clamp(motor_targets, Vec<4>(-max_motor_speed),
                                    Vec<4>(-max_motor_speed));
    for (auto i = 0; i < MOTOR_CONTROLLERS.size(); ++i) {
      auto safe_motor_speed = limit_abs(motor_targets[i], max_motor_speed);
      auto d = iMotion::DataFrame::make_motor_control(
          safe_motor_speed / numeric_limits<uint16_t>::max());
      MOTOR_CONTROLLERS[i].send(d);
    }
  }
};

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
