#pragma once

#include <array>

#include <Eigen/Core>

namespace motion {

struct JointMotion {
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  const Vector7d target;
  const Vector7d target_d;
  double velocity_rel{1.0}, acceleration_rel{1.0}, jerk_rel{1.0};

  explicit JointMotion(const std::array<double, 7> target,
                       const std::array<double, 7> target_d = {0.0})
      : target(target.data()), target_d(target_d.data()) {}
};

} // namespace motion
