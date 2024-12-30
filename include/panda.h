#pragma once
#include <franka/exception.h>
#include <franka/model.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mutex>
#include <thread>
#include <array>

#include "controllers/controller.h"
#include "controllers/joint_limits/virtual_wall_controller.h"
#include "controllers/joint_trajectory.h"
#include "controllers/cartesian_trajectory.h"
#include "controllers/applied_torque.h"

#include "motion/joint_motion.hpp"
#include "motion/motion_data.hpp"

#include "utils.h"

namespace py = pybind11;

class Panda;
namespace motion {
    class Generator;
    class JointGenerator;
    class CartesianGenerator;
    class JointMotionGenerator;
    class CartesianMotionGenerator;
    
};

class PandaContext {
 public:
  PandaContext(Panda &panda, const double &frequency, const double &t_max = 0,
               const uint64_t &max_ticks = 0);
  const PandaContext &enter();
  bool exit(const py::object &type, const py::object &value,
            const py::object &traceback);
  bool ok();
  uint64_t getNumTicks();
  double getTime();

 private:
  double dt_, t_max_;
  uint64_t num_ticks_, max_ticks_;
  std::chrono::high_resolution_clock::time_point t_start_, t_prev_;
  Panda &panda_;
};

class Panda {
 friend class motion::Generator;
 friend class motion::JointGenerator;
 friend class motion::JointMotionGenerator;
 friend class motion::CartesianGenerator;
 friend class motion::CartesianMotionGenerator;

 public:

  // Cartesian constraints
  static constexpr double max_translation_velocity {1.7}; // [m/s]
  static constexpr double max_rotation_velocity {2.5}; // [rad/s]
  static constexpr double max_elbow_velocity {2.175}; // [rad/s]
  static constexpr double max_translation_acceleration {13.0}; // [m/s²]
  static constexpr double max_rotation_acceleration {25.0}; // [rad/s²]
  static constexpr double max_elbow_acceleration {10.0}; // [rad/s²]
  static constexpr double max_translation_jerk {6500.0}; // [m/s³]
  static constexpr double max_rotation_jerk {12500.0}; // [rad/s³]
  static constexpr double max_elbow_jerk {5000.0}; // [rad/s³]
  
  // Joint constraints
  static constexpr std::array<double, 7> max_joint_velocity {{2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}}; // [rad/s]
  static constexpr std::array<double, 7> max_joint_acceleration {{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0}}; // [rad/s²]
  static constexpr std::array<double, 7> max_joint_jerk {{7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0}}; // [rad/s^3]
  
  static constexpr size_t degrees_of_freedoms {7};
  static constexpr double control_rate {0.001}; // [s]

  static const double kMoveToJointPositionThreshold;
  static const Vector7d kDefaultTeachingDamping;


  double velocity_rel {1.0}, acceleration_rel {1.0}, jerk_rel {1.0};

  Panda(
      std::string hostname, std::string name = "panda",
      franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kIgnore);
  ~Panda();
  const PandaContext createContext(double frequency, double max_runtime = 0.0,
                                   uint64_t max_iter = 0);
  franka::Robot &getRobot();
  franka::Model &getModel();
  franka::RobotState getState();
  void startController(std::shared_ptr<TorqueController> controller);
  void stopController();

  void startGenerator(std::shared_ptr<motion::Generator> generator);
  void stopGenerator();

  void enableLogging(size_t buffer_size);
  void disableLogging();
  std::map<std::string, std::list<Eigen::VectorXd>> getLog();

  void stop();
//   void stopMotion();
  void joinMotionThread();

  Eigen::Vector3d getPosition();
  Eigen::Vector4d getOrientation(bool scalar_first = false);
  Eigen::Vector4d getOrientationScalarLast();
  Eigen::Vector4d getOrientationScalarFirst();
  Vector7d getJointPositions();
  Eigen::Matrix<double, 4, 4> getPose();
  void setDefaultBehavior();
  void raiseError();
  void recover();
  void teaching_mode(bool active, const Vector7d &damping = kDefaultTeachingDamping);

  const std::string name_;

 private:
  void _startController(std::shared_ptr<TorqueController> controller);
  void _runController(TorqueCallback &control);

  
  void _setState(const franka::RobotState &state);
  template <typename... Args>
  void _log(const std::string level, Args &&...args);

  TorqueCallback _createTorqueCallback();

  std::shared_ptr<franka::Robot> robot_;
  std::shared_ptr<franka::Model> model_;
  franka::RobotState state_;
  std::mutex mux_;

  std::shared_ptr<TorqueController> current_controller_;
  std::shared_ptr<motion::Generator> current_generator_;

  std::thread current_thread_;
  std::shared_ptr<controllers::joint_limits::VirtualWallController>
      virtual_walls_;
  py::object logger_;
  std::string hostname_;
  std::shared_ptr<franka::Exception> last_error_;
  std::deque<franka::RobotState> log_;
  bool log_enabled_ = false;
  size_t log_size_;
};