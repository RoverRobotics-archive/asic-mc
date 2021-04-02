#pragma once

#include <array>

struct Vec3 {
  std::array<float, 3> coords;

  float normsq() {
    return coords[0] * coords[0] + coords[1] * coords[1] +
           coords[2] * coords[2];
  }
};

inline Vec3 operator+(const Vec3 &a, const Vec3 &b) {
  return Vec3{{a.coords[0] + b.coords[0], a.coords[1] + b.coords[1],
               a.coords[2] + b.coords[2]}};
};
inline Vec3 operator-(const Vec3 &a, const Vec3 &b) {
  return Vec3{{a.coords[0] - b.coords[0], a.coords[1] - b.coords[1],
               a.coords[2] - b.coords[2]}};
};
inline Vec3 operator*(const Vec3 &a, float r) {
  return Vec3{{a.coords[0] * r, a.coords[1] * r, a.coords[2] * r}};
};

inline Vec3 operator*(float r, const Vec3 &a) { return a * r; };

inline Vec3 operator/(const Vec3 &a, float r) {
  return Vec3{{a.coords[0] / r, a.coords[1] / r, a.coords[2] / r}};
};

inline Vec3 operator-(const Vec3 &a) {
  return Vec3{{-a.coords[0], -a.coords[1], -a.coords[2]}};
};

struct Quaternion {
  float re;
  std::array<float, 3> im;
  float normsq() const {
    return re * re + im[0] * im[0] + im[1] * im[1] + im[2] * im[2];
  }
};
