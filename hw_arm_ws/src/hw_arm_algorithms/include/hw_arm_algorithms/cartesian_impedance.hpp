#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>

namespace hw_arm_algorithms::impedance {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

struct Pose {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // TCP -> world
};

struct Sample {
  Pose pose;
  Mat6 J = Mat6::Zero();  // TCP LOCAL_WORLD_ALIGNED; [linear; angular]
  Vec6 dq = Vec6::Zero();
  Vec6 gravity = Vec6::Zero();  // Joint coordinates; added exactly once.
  // Caller checks ALL joint timestamps, IDs, faults, limits and model validity.
  bool valid = false;
};

struct Config {
  Vec6 stiffness = Vec6::Zero();          // [N/m; Nm/rad]
  Vec6 effective_inertia = Vec6::Zero();  // [kg; kg*m^2], local approximation
  Vec6 damping_ratio = Vec6::Ones();
  Vec6 wrench_limit = Vec6::Zero();       // [N; Nm]
  Vec6 torque_limit = Vec6::Zero();       // Nm, physical limits, NOT MIT tMax
  Vec6 torque_slew = Vec6::Zero();        // Nm/s
  Vec6 joint_speed_limit = Vec6::Zero();  // rad/s
  double filter_time = 0.0;              // s
  double reference_linear_speed = 0.0;   // m/s
  double reference_angular_speed = 0.0;  // rad/s
  double activation_time = 0.0;          // s
  double min_dt = 0.0, max_dt = 0.0;      // s
  double max_position_error = 0.0;       // m, norm, hard fault boundary
  double max_rotation_error = 0.0;       // rad, strictly less than pi
  double max_limited_time = 0.0;         // s, consecutive limiting timeout
};

enum class State { Unconfigured, Inactive, Ramping, Active, Fault };
enum class Error {
  None, InvalidConfig, InvalidSample, GravityLimit, InvalidPreviousTorque,
  InvalidDt, TrackingError, NumericalError, SaturationTimeout
};

struct Command {
  Vec6 torque = Vec6::Zero();
  Vec6 raw_torque = Vec6::Zero();  // After activation blend, before limits
  Vec6 wrench = Vec6::Zero();     // After wrench limits
  double activation_weight = 0.0;
  bool wrench_limited = false, torque_limited = false, rate_limited = false;
};

// Single-thread ownership. No ROS/CAN, model evaluation or motor stop action.
class CartesianImpedance {
 public:
  bool configure(const Config& config);
  bool activate(const Sample& sample, const Vec6& previous_applied_torque);
  bool setTarget(const Pose& target);  // Active only; invalid targets rejected.
  std::optional<Command> update(const Sample& sample,
                               const Vec6& previous_applied_torque, double dt);
  void deactivate();  // Does NOT stop motors or clear a latched fault.
  State state() const { return state_; }
  Error error() const { return error_; }
  const Pose& reference() const { return reference_; }

 private:
  bool validSample(const Sample& sample) const;
  bool validTorque(const Vec6& torque) const;
  std::optional<Command> fail(Error error);
  Config config_;
  Vec6 damping_ = Vec6::Zero(), handover_ = Vec6::Zero();
  Pose reference_, target_;
  double elapsed_ = 0.0, limited_time_ = 0.0;
  State state_ = State::Unconfigured;
  Error error_ = Error::None;
};

}  // namespace hw_arm_algorithms::impedance