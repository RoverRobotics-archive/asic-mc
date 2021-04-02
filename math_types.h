#pragma once
#include <array>

struct Vec3 {
  std::array<float, 3> coords;

  Vec3 &operator+=(const Vec3 &other) {
    coords[0] += other.coords[0];
    coords[1] += other.coords[1];
    coords[2] += other.coords[2];
    return *this;
  }

  Vec3 &operator-=(const Vec3 &other) {
    for (size_t i = 0; i < 3; ++i) {
      coords[i] -= other.coords[i];
    }
    coords[0] -= other.coords[0];
    coords[1] -= other.coords[1];
    coords[2] -= other.coords[2];
    return *this;
  }

  Vec3 &operator*=(float other) {
    for (size_t i = 0; i < 3; ++i) {
      coords[i] *= other;
    }
    return *this;
  };

  Vec3 &operator/=(float other) {
    for (size_t i = 0; i < 3; ++i) {
      coords[i] /= other;
    }
    return *this;
  };

  float &operator[](size_t i) { return coords[i]; };

  float operator[](size_t i) const { return coords[i]; };
};

inline float inner(const Vec3 &v0, const Vec3 &v1) {
  float result = 0;
  for (size_t i = 0; i < 3; ++i) {
    result += v0[i] * v1[i];
  }
  return result;
};

inline float norm(const Vec3 &v) { return inner(v, v); };
inline Vec3 operator-(const Vec3 &a) { return {-a[0], -a[1], -a[2]}; };
inline Vec3 operator+(const Vec3 &a, const Vec3 &b) { return Vec3{a} += b; };
inline Vec3 operator-(const Vec3 &a, const Vec3 &b) { return Vec3{a} -= b; };
inline Vec3 operator*(const Vec3 &a, float r) { return Vec3{a} *= r; };
inline Vec3 operator*(float r, const Vec3 &a) { return Vec3{a} *= r; };
inline Vec3 operator/(const Vec3 &a, float r) { return Vec3{a} /= r; };

struct Quaternion;
inline float norm(const Quaternion &q);
inline Quaternion operator*(const Quaternion &q0, const Quaternion &q1);

struct Quaternion {
  float real = 0;
  // imaginary part of this quaternion as i,j,k coefficients
  // if this represents a rotation, this coincides with the axis of rotation.
  Vec3 imag = {0, 0, 0};

  Quaternion &operator+=(const Quaternion &other) {
    real += other.real;
    imag += other.imag;
    return *this;
  };
  Quaternion &operator-=(const Quaternion &other) {
    real -= other.real;
    imag -= other.imag;
    return *this;
  };
  Quaternion &operator*=(float other) {
    real *= other;
    imag *= other;
    return *this;
  }
  Quaternion &operator/=(float other) {
    real /= other;
    imag /= other;
    return *this;
  }
  Quaternion &operator*=(const Quaternion &other) {
    Quaternion result = (*this) * other;
    return (*this = result);
  };
  Quaternion &operator/=(const Quaternion &other) {
    Quaternion result{*this};
    result *= other;
    result /= norm(other);
    return (*this = result);
  };
};

inline Quaternion conj(const Quaternion &q) { return {q.real, -q.imag}; };

inline float norm(const Quaternion &q) {
  // Squared magnitude.
  // If 1, this is a rotation.
  // If zero, this is the zero quaternion.
  return q.real * q.real + inner(q.imag, q.imag);
};

inline Quaternion operator*(const Quaternion &a, const Quaternion &b) {
  Quaternion result{
      a.real * b.real - b.imag[0] * b.imag[0] - a.imag[1] * b.imag[1] -
          a.imag[2] * b.imag[2], // 1*1 = -ii = -jj = -kk
      {
          a.real * b.imag[0] + a.imag[0] * b.real + a.imag[1] * b.imag[2] -
              a.imag[2] * b.imag[1], // i = 1i = i1 = jk = -kj
          a.real * b.imag[1] + a.imag[1] * b.real + a.imag[2] * b.imag[0] -
              a.imag[0] * b.imag[2], // j = 1j = j1 = ki = -ik
          a.real * b.imag[2] + a.imag[2] * b.real + a.imag[0] * b.imag[1] -
              a.imag[1] * b.imag[0], // k = 1k = k1 = ij = -ji
      }};
  return result;
}

inline Quaternion operator-(const Quaternion &q) { return {-q.real, -q.imag}; };
inline Quaternion operator+(const Quaternion &q0, const Quaternion &q1) {
  return Quaternion{q0} += q1;
}
inline Quaternion operator-(const Quaternion &a, const Quaternion &b) {
  return Quaternion{a} -= b;
}
inline Quaternion operator*(const Quaternion &q, float r) {
  return Quaternion{q} *= r;
}
inline Quaternion operator*(float r, const Quaternion &q) {
  return Quaternion{q} *= r;
}
inline Quaternion operator/(const Quaternion &q0, const Quaternion &q1) {
  return Quaternion{q0} /= q1;
}

// Quaternions represent a real rotation and scaling in R3
// If a unit quaternion (normsq=1), then it's just a rotation
// If a real quaternion (imag={0,0,0}), thien it's just a scaling
inline Vec3 transform(const Quaternion &q, const Vec3 &v) {
  return (q * Quaternion{0, v} * conj(q)).imag;
};
