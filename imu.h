#include "BNO055.h"
#include <array>

struct Quaternion {
  float re;
  std::array<float, 3> im;
};

struct Vec3 {
  std::array<float, 3> coords;
};

struct IMUFrame {
  Quaternion angularOrientation;
  Quaternion angularVelocity;
  Vec3 linearAcceleration;
};
