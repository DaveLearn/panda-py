#pragma once

#include <array>

#include <Eigen/Core>

namespace motion {

struct JointMotion {
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    const Vector7d target;
    double velocity_rel {1.0}, acceleration_rel {1.0}, jerk_rel {1.0};

    explicit JointMotion(const std::array<double, 7> target): target(target.data()) { }
};

} 