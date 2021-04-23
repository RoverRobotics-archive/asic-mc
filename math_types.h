#pragma once
#include <array>
#include <initializer_list>
#include <mbed.h>

template <size_t NDIM_> struct Vec {
  static const size_t NDIM = NDIM_;
  std::array<float, NDIM> coords;

  Vec() = default;

  template <typename... T>
  Vec(T... ts)
      : coords{static_cast<float>(ts)...} {}

  Vec &operator+=(const Vec &other) {
    for (size_t i = 0; i < NDIM; ++i)
      coords[i] += other.coords[i];
    return *this;
  }

  Vec &operator-=(const Vec &other) {
    for (size_t i = 0; i < NDIM; ++i)
      coords[i] -= other.coords[i];
    return *this;
  }

  Vec &operator*=(float other) {
    for (size_t i = 0; i < NDIM; ++i) {
      coords[i] *= other;
    }
    return *this;
  };

  Vec &operator/=(float other) {
    for (size_t i = 0; i < NDIM; ++i) {
      coords[i] /= other;
    }
    return *this;
  };

  float &operator[](size_t i) { return coords[i]; };

  float operator[](size_t i) const { return coords[i]; };
};

template <size_t NDIM>
inline float inner(const Vec<NDIM> &v0, const Vec<NDIM> &v1) {
  float result = 0;
  for (size_t i = 0; i < NDIM; ++i) {
    result += v0[i] * v1[i];
  }
  return result;
};

template <size_t NDIM> inline float norm(const Vec<NDIM> &v) {
  return inner(v, v);
};
template <size_t NDIM> inline Vec<NDIM> operator-(const Vec<NDIM> &a) {
  return Vec<NDIM>{} -= a;
};
template <size_t NDIM>
inline Vec<NDIM> operator+(const Vec<NDIM> &a, const Vec<NDIM> &b) {
  return Vec<NDIM>{a} += b;
};
template <size_t NDIM>
inline Vec<NDIM> operator-(const Vec<NDIM> &a, const Vec<NDIM> &b) {
  return Vec<NDIM>{a} -= b;
};
template <size_t NDIM> inline Vec<NDIM> operator*(const Vec<NDIM> &a, float r) {
  return Vec<NDIM>{a} *= r;
};
template <size_t NDIM> inline Vec<NDIM> operator*(float r, const Vec<NDIM> &a) {
  return Vec<NDIM>{a} *= r;
};
template <size_t NDIM> inline Vec<NDIM> operator/(const Vec<NDIM> &a, float r) {
  return Vec<NDIM>{a} /= r;
};
template <size_t NDIM> inline Vec<NDIM> pseudoinverse(const Vec<NDIM> &a) {
  auto n = norm(a);
  return n ? Vec<NDIM>{a} /= norm(a) : Vec<NDIM>{};
};

using Vec2 = Vec<2>;
using Vec3 = Vec<3>;

struct Complex;
inline float norm(const Complex &q);

struct Complex {
  float real = 0;
  float imag = 0;

  Complex &operator+=(const Complex &other) {
    real += other.real;
    imag += other.imag;
    return *this;
  };
  Complex &operator-=(const Complex &other) {
    real -= other.real;
    imag -= other.imag;
    return *this;
  };
  Complex &operator*=(float other) {
    real *= other;
    imag *= other;
    return *this;
  }
  Complex &operator/=(float other) {
    real /= other;
    imag /= other;
    return *this;
  }
  Complex &operator*=(const Complex &other) {
    *this = Complex{real * other.real - imag * other.imag,
                    real * other.imag + imag * other.real};
    return *this;
  };
  Complex &operator/=(const Complex &other) {
    Complex result{*this};
    result *= other;
    result /= norm(other);
    return (*this = result);
  };
};
inline Complex operator-(const Complex &q) { return {-q.real, -q.imag}; };
inline Complex operator+(const Complex &q0, const Complex &q1) {
  return Complex{q0} += q1;
}
inline Complex operator-(const Complex &a, const Complex &b) {
  return Complex{a} -= b;
}
inline Complex operator*(const Complex &q, float r) { return Complex{q} *= r; }
inline Complex operator*(float r, const Complex &q) { return Complex{q} *= r; }
inline Complex operator/(const Complex &q0, const Complex &q1) {
  return Complex{q0} /= q1;
}
inline Complex operator/(const Complex &q0, float r) {
  return Complex{q0} /= r;
}
inline Complex conj(const Complex &z) { return {z.real, -z.imag}; };
inline float norm(const Complex &z) {
  // Squared magnitude.
  return z.real * z.real + z.imag * z.imag;
};
inline Vec2 transform(const Complex &z, const Vec2 &p) {
  // this is motivated by geometric algebra.
  // x -> (u + v xy)x(u - v xy) = (uu-vv)x - (2uv)y
  // y -> (u + v xy)x(u - v xy) = (2uv)x + (uu-vv)y
  auto a = z.real * z.real - z.imag * z.imag;
  auto b = 2 * z.real * z.imag;
  // you can think of it as (uu-vv) is the cosine and (2uv) is the sine
  // this is the same as z*p*conj(z)
  return Vec2{a * p[0] + b * p[1], a * p[1] - b * p[0]};
};

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
// If a real quaternion (imag={0,0,0}), then it's just a scaling
inline Vec3 transform(const Quaternion &q, const Vec3 &v) {
  return (q * Quaternion{0, v} * conj(q)).imag;
};
