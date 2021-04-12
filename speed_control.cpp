#include "hardware.h"
#include "EventQueue.h"
#include "Span.h"
#include "hardware.h"
#include "imc099.h"
#include <array>
#include <mbed.h>

struct SpeedControllerState {
  std::array<int16_t, 4> target_speeds;
  std::array<int16_t, 4> actual_speeds;
  std::array<int16_t, 4> motor_currents;
};

const auto MOTOR_OVERRIDE_DURATION = 1s;
const auto MOTOR_REPORT_PERIOD = 100ms;

class SpeedController {
  EventQueue q;

  Event<void()> report_event;

  Event<void()> motor_override_timeout_event;

  void report_status(){};

  std::array<uint16_t, 4> motor_speeds;

  SpeedController()
      : report_event(q.event(this, &SpeedController::report_status))
      , motor_override_timeout_event(
            q.event(this, &SpeedController::on_motor_override_timeout)) {
    report_event = q.event(this, &SpeedController::report_status);
    report_event.period(MOTOR_REPORT_PERIOD);
    report_event.post();

    for (auto i = 0; i < MOTOR_BOARDS.size(); ++i) {
      MOTOR_BOARDS[i].add_listener(
          q.event(this, &SpeedController::on_motor_message, i));
    };
    q.background({});
  }
  void update_linear_acceleration() {
    //
  }
  void update_rotational_velocity() {
    //
  }
  void update_angular_position() {
    //
  }
  void update_motor_velocity(size_t which, float vel) {
    //
  }
  void on_motor_override_timeout() {
    for (auto i = 0; i < MOTOR_BOARDS.size(); ++i) {
      auto df = iMotion::DataFrame::make_motor_control(0);
      MOTOR_BOARDS[i].send(df);
    }
  }

  void on_motor_message(size_t which_motor, iMotion::DataFrame df) {
    switch (df.command) {
    case iMotion::Command::SET_TARGET_SPEED_REPLY: {
      uint16_t sequencer_state = df.dataword0;
      uint16_t motor_speed_i16 = df.dataword1;
      motor_speeds[which_motor] = motor_speed_i16;
    }
    default:
      return;
    }
  };

  void set_report_period_ms(uint16_t millis) {
    report_event.period(millis * 1ms);
  };

  void set_motor_targets(std::array<int16_t, 4> target_speeds) {
    motor_override_timeout_event.cancel();
    for (auto i = 0; i < 4; ++i) {
      auto df = iMotion::DataFrame::make_motor_control(target_speeds[i]);
      MOTOR_BOARDS[i].send(df);
    }
    motor_override_timeout_event.delay(MOTOR_OVERRIDE_DURATION);
    motor_override_timeout_event.post();
  };
};
