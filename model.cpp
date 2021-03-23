#include "Span.h"
#include <array>
#include <cstddef>
#include <initializer_list>
#include <mbed.h>

template <size_t N_TANS> struct MultiTangent {
  // the value
  double primal;
  // the derivatives of the value WRT some set of inputs
  std::array<double, N_TANS> duals;

  MultiTangent(double a_primal, size_t i) {
    primal = a_primal;
    duals = {};
    duals[i] = 1;
  }

  MultiTangent(double a_primal) {
    primal = a_primal;
    duals = {};
  }

  MultiTangent operator*(MultiTangent &other) const {
    MultiTangent result;
    result.primal = primal * other.primal;
    for (auto i = 0; i < duals.size(); ++i) {
      result.duals[i] = primal * other.duals[i] + duals[i] * other.primal;
    }
    return result;
  };

  MultiTangent operator+(MultiTangent &other) const {
    MultiTangent result;
    result.primal = primal + other.primal;
    for (auto i = 0; i < duals.size(); ++i) {
      result.duals[i] = duals[i] + other.duals[i];
    }
    return result;
  };

  MultiTangent operator-(MultiTangent &other) const {
    MultiTangent result;
    result.primal = primal - other.primal;
    for (auto i = 0; i < duals.size(); ++i) {
      result.duals[i] = duals[i] - other.duals[i];
    }
    return result;
  };

  MultiTangent operator/(MultiTangent &other) const {
    MultiTangent result;
    result.primal = primal / other.primal;

    for (auto i = 0; i < duals.size(); ++i) {
      result.duals[i] = (duals[i] * other.primal - primal * other.duals[i]) /
                        (other.primal * other.primal);
    }
    return result;
  }
};

template <typename F = float> struct Point3 {
  F re;
  std::array<F, 3> coords;
};

template <typename F = float> struct Vec3 {
  std::array<F, 3> coords;

  Vec3 operator*(const F &other) const {
    return {coords[0] * other, coords[1] * other, coords[2] * other};
  };

  Vec3 operator+(const Vec3 &other) const {
    return {coords[0] + other.coords[0], coords[1] + other.coords[1],
            coords[2] + other.coords[2]};
  };
};

template <typename F = float> struct Quaternion {
  // r + i + j + k
  float re;
  std::array<float, 3> im;

  Quaternion(F f, std::array<F, 3> imag)
      : re{f}
      , im(imag){};

  Quaternion(F f)
      : Quaternion(f, {0, 0, 0}) {}

  Quaternion conj() const { return {re, {-im[0], -im[1], -im[2]}}; }

  F normsq() {
    return re * re + im[0] * im[0] + im[1] * im[1] + im[2] * im[2];
  };

  Quaternion operator*(const Quaternion &other) const {
    return {re * other.re - im[0] * other.im[0] - im[1] * other.im[1] -
                im[2] * other.im[2], // 1*1 = -ii = -jj = -kk
            {
                re * other.im[0] + im[0] * other.re + im[1] * other.im[2] -
                    im[2] * other.im[1], // i = 1i = i1 = jk = -kj
                re * other.im[1] + im[1] * other.re + im[2] * other.im[0] -
                    im[0] * other.im[2], // j = 1j = j1 = ki = -ik
                re * other.im[2] + im[2] * other.re + im[0] * other.im[1] -
                    im[1] * other.im[0], // k = 1k = k1 = ij = -ji
            }};
  };

  Quaternion operator/(const Quaternion &other) const {
    return (*this) * (other.conj() / other.normsq());
  };

  Quaternion operator+(const Quaternion &other) const {
    return {re + other.re,
            {im[0] + other.im[0], im[1] + other.im[1], im[2] + other.im[2]}};
  };
  Quaternion operator-() const { return {re, {im[0], im[1], im[2]}}; };

  Quaternion operator-(const Quaternion &other) const {
    return (*this + (-other));
  };
};

/// momentum is represented as a combination of: scalar (inertial mass), vector
/// (linear momentum), bivector (rotational momentum)
template <typename F = float> struct Translation { Vec3<F> vector; };
template <typename F = float> struct Rotation { Quaternion<F> quaternion; };

/// Idea: use forward mode automatic differentiation through the kinematic model
/// and perform gradient descent to update the current state

enum class Measurable {
  mc0_rpm,
  mc1_rpm,
  mc2_rpm,
  mc3_rpm,

  imu_rot_pos_r,
  imu_rot_pos_i,
  imu_rot_pos_j,
  imu_rot_pos_k,

  imu_rot_vel_r,
  imu_rot_vel_i,
  imu_rot_vel_j,
  imu_rot_vel_k,

  imu_accel_x,
  imu_accel_y,
  imu_accel_z,

  NUM,
};

struct KinematicObs {
  std::array<float, 4> motor_rpm;
  Quaternion<float> IMU_rotation;
  Quaternion<float> IMU_d_rotation;
  Vec3<float> IMU_ddx;
};

std::array<float, 4> drpm_dfwd;
std::array<float, 4> drpm_drot;

// position x = forward, y = left, z = up
struct KinematicState {
  Vec3<float> x;
  Vec3<float> dx;
  Vec3<float> ddx;
  Quaternion<float> rotation{1};
  Quaternion<float> d_rotation{1};

  void evolve(float delta_t) {
    x = x + dx * delta_t;
    dx = dx + ddx * delta_t;
    rotation = rotation + d_rotation * delta_t;
  }

  KinematicObs predict_observation() {
    KinematicObs obs;
    auto fwd = rotation.conj() * Vec3{1, 0, 0} * rotation;
    auto v_fwd = fwd.dot(dx);
    auto v_clockwise = d_rotation * d_rotation.im[3];
    obs.motor_rpm = { drpm_dfwd * v_fwd + drpm_drot * v_clockwise;
  };
  obs.IMU_rotation = rotation;
  obs.IMU_d_rotation = d_rotation;
  obs.IMU_ddx = ddx;
};
}
;
