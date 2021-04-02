#pragma once
#include <array>

struct Vec3 {
  std::array<float, 3> coords;
};

struct Quaternion {
  float re;
  std::array<float, 3> im;
  float normsq(){
      return re*re+im[0]*im[0] + im[1]*im[1] + im[2]*im[2];
  }
};