#pragma once

namespace motion {

struct MotionData {    
    double velocity_rel {1.0}, acceleration_rel {1.0}, jerk_rel {1.0};
    bool max_dynamics {false};

    explicit MotionData(double velocity = 1.0, double acceleration = 1.0, double jerk = 1.0): velocity_rel(velocity), acceleration_rel(acceleration), jerk_rel(jerk) { }

    inline double velocity() const { return velocity_rel; }
    inline double acceleration() const { return acceleration_rel; }
    inline double jerk() const { return jerk_rel; }
    inline bool maxDynamics() const { return max_dynamics; }

    MotionData& withMaxDynamics() {
        this->max_dynamics = true;
        return *this;
    }

};

} 