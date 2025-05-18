#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <ruckig/ruckig.hpp>

#include "motion/generator.h"
#include "motion/joint_motion.hpp"
#include "motion/motion_data.hpp"
#include "panda.h"

namespace motion {

struct JointMotionGenerator : public JointGenerator {

  // run base class
  JointMotionGenerator(bool keep_running = true,
                       std::function<void()> done_callback = nullptr)
      : keep_running_(keep_running), JointGenerator(done_callback) {}

  void addWaypoint(const JointMotion &waypoint) {
    std::scoped_lock lock(mux_);
    waypoints_.push(waypoint);
    reload_ = true;
  }

  void addWaypoints(const std::vector<JointMotion> &waypoints) {
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
    reload_ = false;
    motion_finished_ = false;
    motion_finishing_ = false;
    setInputCurrent(robot_state);
    setProfile(1.0, 1.0, 1.0);
    loadNextWaypoint(robot_state);
  }

  void stop(const franka::RobotState &robot_state,
            std::shared_ptr<franka::Model> model) override {
    motion_finishing_ = true;
  }

  franka::JointPositions step(const franka::RobotState &robot_state,
                              franka::Duration period) override {

    panda_->_setState(robot_state);
    setTime(getTime() + period.toSec());
    if (motion_finishing_)
      return cooldown(robot_state, period);

    if (reload_) {
      reload_ = false;
      loadNextWaypoint(robot_state);
    }
    
    const int steps = std::max<int>(period.toMSec(), 1);
    for (int i = 0; i < steps; i++) {
      result = trajectory_generator_.update(input_para_, output_para_);
      output_para_.pass_to_input(input_para_);
      if (result == ruckig::Result::Finished) {
        if (!waypoints_.empty())
          reload_ = true;
        else if (!keep_running_)
          motion_finishing_ = true;
        break;
      } else if (result != ruckig::Result::Working) {
        std::cout << "[rucking robot] Invalid inputs:" << std::endl;
        motion_finishing_ = true;
      }
    }
    return franka::JointPositions(output_para_.new_position);
  }

  bool isRunning() override { return !motion_finished_; }

  const std::string name() { return "Joint Motion Generator"; }

private:
  std::mutex mux_;
  std::atomic<bool> motion_finished_;
  std::atomic<bool> motion_finishing_;
  std::queue<JointMotion> waypoints_;
  std::optional<JointMotion> current_waypoint_;
  ruckig::Ruckig<Panda::degrees_of_freedoms> trajectory_generator_{
      Panda::control_rate};
  ruckig::InputParameter<Panda::degrees_of_freedoms> input_para_;
  ruckig::OutputParameter<Panda::degrees_of_freedoms> output_para_;
  ruckig::Result result;
  bool reload_ = false;
  bool keep_running_ = true;
  const size_t cooldown_iterations{5};
  size_t current_cooldown_iteration{0};

  void setProfile(double velocity_rel, double acceleration_rel,
                  double jerk_rel) {
    for (int dof = 0; dof < Panda::degrees_of_freedoms; dof += 1) {
      input_para_.max_velocity[dof] =
          Panda::max_joint_velocity[dof] * panda_->velocity_rel * velocity_rel;
      input_para_.max_acceleration[dof] =
          0.3 * Panda::max_joint_acceleration[dof] * panda_->acceleration_rel *
          acceleration_rel;
      input_para_.max_jerk[dof] =
          0.3 * Panda::max_joint_jerk[dof] * panda_->jerk_rel * jerk_rel;
    }
  }

  void setInputCurrent(const franka::RobotState &robot_state) {
    input_para_.current_position = robot_state.q;
    input_para_.current_velocity = robot_state.dq;
    input_para_.current_acceleration = toStd(Vector7d::Zero());
  }

  void setInputTarget(const JointMotion &waypoint) {
    input_para_.target_position = toStd(waypoint.target);
    input_para_.target_velocity = toStd(waypoint.target_d);
    input_para_.target_acceleration = toStd(Vector7d::Zero());
    setProfile(waypoint.velocity_rel, waypoint.acceleration_rel,
               waypoint.jerk_rel);
  }

  void loadNextWaypoint(const franka::RobotState &robot_state) {
    if (waypoints_.empty()) {
      auto target = JointMotion(robot_state.q_d);
      current_waypoint_.emplace(target);
    } else {
      std::scoped_lock lock(mux_);
      current_waypoint_.emplace(waypoints_.front());
      waypoints_.pop();
    }
    // setInputCurrent(robot_state);
    setInputTarget(*current_waypoint_);
  }

  franka::JointPositions cooldown(const franka::RobotState &robot_state,
                                  franka::Duration period) {
    if (current_cooldown_iteration < cooldown_iterations) {
      current_cooldown_iteration++;
      return franka::JointPositions(robot_state.q_d);
    }
    motion_finishing_ = false;
    motion_finished_ = true;
    // std::cout << "FINISHED" << std::endl;
    return franka::MotionFinished(franka::JointPositions(robot_state.q_d));
  }
};
} // namespace motion
