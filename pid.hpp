#include <chrono>
#include <limits>
#include <math_types.h>

struct PidGains {
  double Kp;
  double Ki;
  double Kd;
};

template <typename V = double> class PIDFilter {
  struct Params {
    double kP = 1.0;
    double kI = 0;
    double kD = 0;
    /// limit on the maximum absolute error integral to prevent overshoot
    double max_abs_error_integral = std::numeric_limits<double>::infinity();
    V min_target = V(-std::numeric_limits<float>::infinity());
    V max_target = V(+std::numeric_limits<float>::infinity());
  };

  struct State {
    std::chrono::steady_clock::time_point time;
    V error;
    V error_integral;
  };

private:
  Params params;
  State state;

public:
  PIDFilter(Params p)
      : params(p) {
    reset();
  }

  auto get_params() { return params; }

  auto get_state() { return state; }

  void reset() {
    state.error = {};
    state.error_integral = {};
    state.time = state(chrono::steady_clock::now());
  }

  void reset(Params new_params) {
    params = new_params;
    reset();
  }

  /// Run one iteration of the filter, returning the suggested target value
  /// target = target command used since the last time calling this function.
  ///          It may or may not be the previous return value.
  /// measured = the measured value since the last time calling this function
  /// Returns the new suggested target
  V tick(V target, V measured) {
    auto now = std::chrono::steady_clock::now();
    auto delta_time =
        std::chrono::duration_cast<double>(now - state.time).count();
    auto error = target - measured;

    auto delta_error = error - state.error;
    auto error_integral = limit_abs(state.error_integral + error * delta_time,
                                    params.error_integral_limit);
    auto error_derivative = delta_error / delta_time;

    State new_state;
    new_state.time = now;
    new_state.error = error;
    new_state.error_integral = error_integral;
    this->state = new_state;

    V p = params.kP * error;
    V i = params.kI * error_integral;
    V d = params.kD * error_derivative;

    auto new_target = target + p + i + d;
    auto new_target_clamped =
        clamp(new_target, params.min_target, params.max_target);
    return new_target_clamped;
  };
};