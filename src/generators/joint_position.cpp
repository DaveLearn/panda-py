#include "generators/joint_position.h"
#include "constants.h"
#include <iostream>

const double JointPositionGenerator::kDefaultFilterCoeff = 1.0;
const double kDefaultDqdData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d JointPositionGenerator::kDefaultDqd = Vector7d(kDefaultDqdData);

const Vector7d JointPositionGenerator::kMaxJointVelocities = Vector7d({2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610});
const Vector7d JointPositionGenerator::kMaxJointAccelerations = Vector7d({3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0});
const Vector7d JointPositionGenerator::kMaxJointJerk = Vector7d({30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0});

JointPositionGenerator::JointPositionGenerator(const double filter_coeff) {
  filter_coeff_ = filter_coeff;
  acc_rel_ = 1.0;
  vel_rel_ = 1.0;
  jerk_rel_ = 1.0;

  max_accelerations_ = kMaxJointAccelerations;
  max_velocities_ = kMaxJointVelocities;
  max_jerk_ = kMaxJointJerk;
};

void JointPositionGenerator::start(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) {
  motion_finished_ = false; 

  q_d_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  q_d_target_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq_d_.setZero();
  dq_d_target_.setZero();

  input_para.current_position = toStd(q_d_);
  input_para.current_velocity = toStd(Vector7d::Zero());
  input_para.current_acceleration = toStd(Vector7d::Zero());

  input_para.target_position = toStd(q_d_target_);  
  input_para.target_velocity = toStd(Vector7d::Zero());
  input_para.target_acceleration = toStd(Vector7d::Zero());

  input_para.enabled = std::array<bool, 7>{true, true, true, true, true, true, true};
  std::cout << "[ruckig] Started with current position:" << input_para.current_position[0] << "," << input_para.current_position[1] << "," << input_para.current_position[2] << "," << input_para.current_position[3] << "," << input_para.current_position[4] << "," << input_para.current_position[5] << "," << input_para.current_position[6] << std::endl;
  std::cout << "[ruckig] Target position:" << input_para.target_position[0] << "," << input_para.target_position[1] << "," << input_para.target_position[2] << "," << input_para.target_position[3] << "," << input_para.target_position[4] << "," << input_para.target_position[5] << "," << input_para.target_position[6] << std::endl;
}

void JointPositionGenerator::stop(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool JointPositionGenerator::isRunning() {
  return !motion_finished_;
}

const std::string JointPositionGenerator::name() {
  return "Joint Position Generator";
}

franka::JointPositions JointPositionGenerator::step(const franka::RobotState &robot_state, franka::Duration &duration) {
  Vector7d q, q_d, dq_d, dq, tau_d, K_p, K_d;
  q = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  mux_.lock();
  _updateFilter();
  mux_.unlock();


  result = trajectory_generator.update(input_para, output_para);

  if (result == ruckig::Result::Error) {
      std::cout << "[ruckig] Invalid inputs:" << std::endl;
      return franka::MotionFinished(franka::JointPositions(output_para.new_position));
  }
  if (reload || result == ruckig::Result::Finished) {
      input_para.current_position = toStd(q);
      input_para.current_velocity = toStd(dq);
      input_para.current_acceleration = toStd(Vector7d::Zero());

      input_para.target_position = toStd(q_d_);
      input_para.target_velocity = toStd(Vector7d::Zero()); 
      input_para.target_acceleration = toStd(Vector7d::Zero());
      reload = false;
  }
  output_para.pass_to_input(input_para);

  return franka::JointPositions(output_para.new_position);
}

void JointPositionGenerator::_updateFilter() {
  q_d_ = ema_filter(q_d_, q_d_target_, filter_coeff_, true);
  dq_d_ = ema_filter(dq_d_, dq_d_target_, filter_coeff_, true);

  input_para.max_velocity = toStd(max_velocities_ * vel_rel_);
  input_para.max_acceleration = toStd(max_accelerations_ * acc_rel_);
  input_para.max_jerk = toStd(max_jerk_ * jerk_rel_);

}

void JointPositionGenerator::setControl(const Vector7d &position,
                               const Vector7d &velocity) {
  std::lock_guard<std::mutex> lock(mux_);
  q_d_target_ = position;
  dq_d_target_ = velocity;
  reload = true;
}

void JointPositionGenerator::setFilter(const double filter_coeff) {
  std::lock_guard<std::mutex> lock(mux_);
  filter_coeff_ = filter_coeff;
}

void JointPositionGenerator::setRelVelocities(const double vel_rel) {
  std::lock_guard<std::mutex> lock(mux_);
  vel_rel_ = vel_rel;
}

void JointPositionGenerator::setRelAccelerations(const double acc_rel) {
  std::lock_guard<std::mutex> lock(mux_);
  acc_rel_ = acc_rel;
}

void JointPositionGenerator::setRelJerk(const double jerk_rel) {
  std::lock_guard<std::mutex> lock(mux_);
  jerk_rel_ = jerk_rel;
}

void JointPositionGenerator::setMaxVelocities(const Vector7d &max_velocities) {
  std::lock_guard<std::mutex> lock(mux_);
  max_velocities_ = max_velocities;
}

void JointPositionGenerator::setMaxAccelerations(const Vector7d &max_accelerations) {
  std::lock_guard<std::mutex> lock(mux_);
  max_accelerations_ = max_accelerations;
}

void JointPositionGenerator::setMaxJerk(const Vector7d &max_jerk) {
  std::lock_guard<std::mutex> lock(mux_);
  max_jerk_ = max_jerk;
}
