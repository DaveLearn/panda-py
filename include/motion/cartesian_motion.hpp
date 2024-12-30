#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace motion {

enum class ReferenceFrame { GLOBAL, RELATIVE };

struct CartesianMotion {

    double velocity_rel {1.0}, acceleration_rel {1.0}, jerk_rel {1.0};
    Eigen::Isometry3d target;
    ReferenceFrame reference_frame {ReferenceFrame::GLOBAL};

    explicit CartesianMotion(const Eigen::Vector3d &position, const Eigen::Matrix<double, 4, 1> &orientation, ReferenceFrame reference_frame = ReferenceFrame::GLOBAL)
    {
        target.translation() = position;
        target.linear() = Eigen::Quaterniond(orientation).toRotationMatrix();
        this->reference_frame = reference_frame;
    }

    explicit CartesianMotion(const Eigen::Isometry3d &pose, ReferenceFrame reference_frame = ReferenceFrame::GLOBAL)
    {
        target = pose;
        this->reference_frame = reference_frame;
    }

    explicit CartesianMotion(const Eigen::Matrix4d &pose, ReferenceFrame reference_frame = ReferenceFrame::GLOBAL)
    {
        target = Eigen::Isometry3d(pose);
        this->reference_frame = reference_frame;
    }
};

} 