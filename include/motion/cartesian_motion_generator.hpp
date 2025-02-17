#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <ruckig/ruckig.hpp>

#include "motion/cartesian_motion.hpp"
#include "motion/generator.h"
#include "motion/motion_data.hpp"
#include "panda.h"

namespace motion {

struct CartesianMotionGenerator : public CartesianGenerator {

  // run base class
  CartesianMotionGenerator(bool keep_running = true,
                           std::function<void()> done_callback = nullptr)
      : keep_running_(keep_running), CartesianGenerator(done_callback) {}

  void addWaypoint(const CartesianMotion &waypoint) {
    std::scoped_lock lock(mux_);
    waypoints_.push(waypoint);
    reload_ = true;
  }

  void addWaypoints(const std::vector<CartesianMotion> &waypoints) {
    std::scoped_lock lock(mux_);
    for (const auto &waypoint : waypoints) {
      this->waypoints_.push(waypoint);
    }
    reload_ = true;
  }

  void clearWaypoints() {
    std::scoped_lock lock(mux_);
    while (!waypoints_.empty()) {
      waypoints_.pop();
    }
    reload_ = true;
  }

  void start(Panda *robot, const franka::RobotState &robot_state,
             std::shared_ptr<franka::Model> model) override {
    panda_ = robot;
    reload_ = true;
    motion_finished_ = false;
    motion_finishing_ = false;
    setInputCurrent(robot_state);
    Eigen::Isometry3d O_T_EE =
        Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    // std::cout << "current translation: " << O_T_EE.translation() <<
    // std::endl;

    auto translation = O_T_EE.translation();
    auto quat = Eigen::Quaterniond(O_T_EE.rotation());

    input_para_.target_position[0] = translation[0];
    input_para_.target_position[1] = translation[1];
    input_para_.target_position[2] = translation[2];
    input_para_.target_position[3] = quat.x();
    input_para_.target_position[4] = quat.y();
    input_para_.target_position[5] = quat.z();
    input_para_.target_position[6] = quat.w();

    input_para_.target_velocity = toStd(Vector7d::Zero());
    input_para_.target_acceleration = toStd(Vector7d::Zero());

    setProfile(1.0, 1.0, 1.0);
  }

  void stop(const franka::RobotState &robot_state,
            std::shared_ptr<franka::Model> model) override {
    motion_finishing_ = true;
  }

  franka::CartesianPose step(const franka::RobotState &robot_state,
                             franka::Duration period) override {
    panda_->_setState(robot_state);
    setTime(getTime() + period.toSec());
    if (motion_finishing_)
      return cooldown(robot_state, period);

    if (reload_) {
      reload_ = false;
      loadNextWaypoint(robot_state);
    }
    result = trajectory_generator_.update(input_para_, output_para_);
    output_para_.pass_to_input(input_para_);

    if (result == ruckig::Result::Finished) {
      if (!waypoints_.empty())
        reload_ = true;
      else if (!keep_running_)
        motion_finishing_ = true;
    } else if (result != ruckig::Result::Working) {
      std::cout << "[rucking robot] Invalid inputs:" << std::endl;
      motion_finishing_ = true;
    }
    return getPoseFromOutput(robot_state, output_para_);
  }

  bool isRunning() { return !motion_finished_; }

  const std::string name() { return "Joint Motion Generator"; }

private:
  std::mutex mux_;
  std::atomic<bool> motion_finished_;
  std::atomic<bool> motion_finishing_;
  std::queue<CartesianMotion> waypoints_;
  std::optional<CartesianMotion> current_waypoint_;
  ruckig::Ruckig<7> trajectory_generator_{
      Panda::control_rate}; // xyz + quaternion
  ruckig::InputParameter<7> input_para_;
  ruckig::OutputParameter<7> output_para_;
  ruckig::Result result;
  bool reload_ = false;
  bool keep_running_ = true;
  const size_t cooldown_iterations{5};
  size_t current_cooldown_iteration{0};

  void setProfile(double velocity_rel, double acceleration_rel,
                  double jerk_rel) {
    auto translation_factor = 0.4;
    auto derivative_factor = 0.4;
    for (int dof = 0; dof < 3; dof += 1) {
      input_para_.max_velocity[dof] = 0.8 * translation_factor *
                                      Panda::max_translation_velocity *
                                      panda_->velocity_rel * velocity_rel;
      input_para_.max_acceleration[dof] =
          0.3 * translation_factor * derivative_factor *
          Panda::max_translation_acceleration * panda_->acceleration_rel *
          acceleration_rel;
      input_para_.max_jerk[dof] = 0.3 * translation_factor * derivative_factor *
                                  Panda::max_translation_jerk *
                                  panda_->jerk_rel * jerk_rel;
    }
    auto quat_factor =
        0.5; // dq/dt = 0.5*w*q (w: angular velocity, q: quaternion)
    for (int dof = 3; dof < 3 + 4; dof += 1) {
      input_para_.max_velocity[dof] = quat_factor *
                                      Panda::max_rotation_velocity *
                                      panda_->velocity_rel * velocity_rel;
      input_para_.max_acceleration[dof] =
          quat_factor * 0.3 * Panda::max_rotation_acceleration *
          panda_->acceleration_rel * acceleration_rel;
      input_para_.max_jerk[dof] = quat_factor * 0.3 * Panda::max_rotation_jerk *
                                  panda_->jerk_rel * jerk_rel;
    }
  }

  void setInputCurrent(const franka::RobotState &robot_state) {

    auto X_WE =
        Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    auto twist = robot_state.O_dP_EE_c;
    auto translation = X_WE.translation();
    auto rotation = X_WE.rotation();
    Eigen::Quaterniond q(rotation);
    Eigen::Quaterniond omega(0.0, twist[3], twist[4], twist[5]);
    Eigen::Quaterniond qdot = omega * q;

    input_para_.current_position[0] = translation[0];
    input_para_.current_position[1] = translation[1];
    input_para_.current_position[2] = translation[2];
    input_para_.current_position[3] = q.x();
    input_para_.current_position[4] = q.y();
    input_para_.current_position[5] = q.z();
    input_para_.current_position[6] = q.w();

    input_para_.current_velocity[0] = twist[0];
    input_para_.current_velocity[1] = twist[1];
    input_para_.current_velocity[2] = twist[2];
    input_para_.current_velocity[3] = 0.5 * qdot.x();
    input_para_.current_velocity[4] = 0.5 * qdot.y();
    input_para_.current_velocity[5] = 0.5 * qdot.z();
    input_para_.current_velocity[6] = 0.5 * qdot.w();
    // input_para_.current_velocity = toStd(Vector7d::Zero());
    input_para_.current_acceleration = toStd(Vector7d::Zero());
  }

  void setInputTarget(const franka::RobotState &robot_state,
                      const CartesianMotion &waypoint) {
    auto X_WE_target = waypoint.target;

    if (waypoint.reference_frame == ReferenceFrame::RELATIVE) {
      auto X_WE =
          Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      X_WE_target = X_WE * waypoint.target;
    }
    auto translation = X_WE_target.translation();
    auto rotation = X_WE_target.rotation();
    Eigen::Quaterniond q(rotation);
    q.normalize();

    // std::cout << "InputTarget: " << translation << std::endl;

    // input_para_.target_position = toStd3(translation);
    input_para_.target_position[0] = translation[0];
    input_para_.target_position[1] = translation[1];
    input_para_.target_position[2] = translation[2];
    input_para_.target_position[3] = q.x();
    input_para_.target_position[4] = q.y();
    input_para_.target_position[5] = q.z();
    input_para_.target_position[6] = q.w();

    input_para_.target_velocity = toStd(Vector7d::Zero());
    input_para_.target_acceleration = toStd(Vector7d::Zero());

    setProfile(waypoint.velocity_rel, waypoint.acceleration_rel,
               waypoint.jerk_rel);
  }

  void loadNextWaypoint(const franka::RobotState &robot_state) {
    if (waypoints_.empty()) {
      Eigen::Isometry3d X_WE(Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data()));
      auto target = CartesianMotion(X_WE);
      current_waypoint_.emplace(target);
    } else {
      std::scoped_lock lock(mux_);
      current_waypoint_.emplace(waypoints_.front());
      waypoints_.pop();
    }
    setInputTarget(robot_state, *current_waypoint_);
  }

  // franka::CartesianPose getPoseFromOutput(ruckig::OutputParameter<7> &output)
  franka::CartesianPose getPoseFromOutput(const franka::RobotState &robot_state,
                                          ruckig::OutputParameter<7> &output) {
    // Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    // pose.translation() = Eigen::Vector3d(output.new_position[0],
    // output.new_position[1], output.new_position[2]);

    // Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>
    // pose(robot_state.O_T_EE.data()); std::cout<<"New position:
    // "<<output.new_position[0]<<" "<<output.new_position[1]<<"
    // "<<output.new_position[2]<<std::endl;
    auto pose =
        Eigen::Isometry3d(Eigen::Matrix<double, 4, 4, Eigen::ColMajor>::Map(
            robot_state.O_T_EE_c.data()));
    pose.translation() = Eigen::Vector3d(
        output.new_position[0], output.new_position[1], output.new_position[2]);
    Eigen::Quaterniond q(output.new_position[6], output.new_position[3],
                         output.new_position[4], output.new_position[5]);
    q.normalize();
    pose.linear() = q.toRotationMatrix();

    // std::cout << "pose: " << pose.matrix() << std::endl;

    std::array<double, 16> vec;
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(vec.data()) =
        pose.matrix();
    // std::cout<< "vec: "<< vec[0] << " " << vec[1] << " " << vec[2] << " " <<
    // vec[3] << " " << vec[4] << " " << vec[5] << " " << vec[6] << " " <<
    // vec[7] << " " << vec[8] << " " << vec[9] << " " << vec[10] << " " <<
    // vec[11] << " " << vec[12] << " " << vec[13] << " " << vec[14] << " " <<
    // vec[15] << std::endl; std::cout<<"dat: "<< robot_state.O_T_EE[0] << " "
    // << robot_state.O_T_EE[1] << " " << robot_state.O_T_EE[2] << " " <<
    // robot_state.O_T_EE[3] << " " << robot_state.O_T_EE[4] << " " <<
    // robot_state.O_T_EE[5] << " " << robot_state.O_T_EE[6] << " " <<
    // robot_state.O_T_EE[7] << " " << robot_state.O_T_EE[8] << " " <<
    // robot_state.O_T_EE[9] << " " << robot_state.O_T_EE[10] << " " <<
    // robot_state.O_T_EE[11] << " " << robot_state.O_T_EE[12] << " " <<
    // robot_state.O_T_EE[13] << " " << robot_state.O_T_EE[14] << " " <<
    // robot_state.O_T_EE[15] << std::endl;

    // return franka::CartesianPose(vec, robot_state.elbow);
    return franka::CartesianPose(vec);
  }

  franka::CartesianPose cooldown(const franka::RobotState &robot_state,
                                 franka::Duration period) {
    if (current_cooldown_iteration < cooldown_iterations) {
      current_cooldown_iteration++;
      return franka::CartesianPose(robot_state.O_T_EE_c);
    }
    motion_finishing_ = false;
    motion_finished_ = true;
    return franka::MotionFinished(franka::CartesianPose(robot_state.O_T_EE_c));
  }
};
} // namespace motion
