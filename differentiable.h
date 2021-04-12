#include <mbed.h>



template <size_t N_TANS> struct MultiTangent {
    /// Represents a differentiable function of several variables
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


