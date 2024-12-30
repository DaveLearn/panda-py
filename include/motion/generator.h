#pragma once

#include <atomic>
#include <functional>

#include <franka/robot.h>

#include <panda.h>
#include <pybind11/pybind11.h>


typedef Callback<franka::JointPositions> JointPositionCallback; 


namespace motion {

struct Generator {

  Generator(std::function<void()> done_callback = nullptr) : done_callback_(done_callback) {}

  virtual void start(
    Panda* robot,
    const franka::RobotState& robot_state, 
    std::shared_ptr<franka::Model> model) = 0;

  virtual void stop(const franka::RobotState &robot_state,
                    std::shared_ptr<franka::Model> model) = 0;
  virtual bool isRunning() = 0;
  virtual const std::string name() = 0;
  void setTime(double time) { time_ = time; }
  double getTime() { return time_; }

  void setDoneCallback(std::function<void()> done_callback) {
    done_callback_ = done_callback;
  }

  virtual void runController() = 0;




  protected:
    std::atomic<double> time_;
    Panda* panda_;
    std::function<void()> done_callback_;

  template<typename T>
  void runController_() {
    try {
      // auto callback = std::bind(&T::step, this, std::placeholders::_1, std::placeholders::_2);
      // cast this to T
      auto my_controller = static_cast<T*>(this);
      auto callback = std::bind(&T::step, my_controller, std::placeholders::_1, std::placeholders::_2);
      (panda_->getRobot()).control(callback);
    } catch (const franka::Exception& e) {
      panda_->_log("error", "Control loop interruped: %s", e.what());
      panda_->last_error_ = std::make_shared<franka::Exception>(e);
    }

    if (done_callback_) {
      try {
        done_callback_();
      } catch (const std::exception& e) {
        std::cout<<"Done callback error: "<<e.what()<<std::endl;
      }
    }
  }

};

struct JointGenerator: Generator {

  JointGenerator(std::function<void()> done_callback = nullptr) : Generator(done_callback) {} 

  virtual franka::JointPositions step(const franka::RobotState& robot_state, franka::Duration period) = 0;
  void runController() override {
    runController_<JointGenerator>();
  }
};


struct CartesianGenerator: Generator {

  CartesianGenerator(std::function<void()> done_callback = nullptr) : Generator(done_callback) {} 

  virtual franka::CartesianPose step(const franka::RobotState& robot_state, franka::Duration period) = 0;
  void runController() override {
    runController_<CartesianGenerator>();
  }
};

}