#include "hw_arm_algorithms/cartesian_impedance.hpp"

#include <algorithm>
#include <cmath>

namespace hw_arm_algorithms::impedance {
namespace {
constexpr double kPi = 3.14159265358979323846;
bool positive(double x) { return std::isfinite(x) && x > 0.0; }
bool positive(const Vec6& x) {
  return x.allFinite() && (x.array() > 0.0).all();
}
bool validPose(const Pose& p) {
  return p.p.allFinite() && p.q.coeffs().allFinite() &&
         std::abs(p.q.norm() - 1.0) < 1e-6;
}
double angle(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  return a.angularDistance(b);  // Physical angle, invariant under q -> -q.
}
bool within(double x, double limit) { return std::isfinite(x) && x <= limit; }
}  // namespace

bool CartesianImpedance::configure(const Config& c) {
  if (state_ == State::Active || state_ == State::Ramping) return false;
  state_ = State::Unconfigured;
  error_ = Error::InvalidConfig;
  if (!c.stiffness.allFinite() || (c.stiffness.array() < 0.0).any() ||
      !positive(c.effective_inertia) || !positive(c.damping_ratio) ||
      !positive(c.wrench_limit) || !positive(c.torque_limit) ||
      !positive(c.torque_slew) || !positive(c.joint_speed_limit) ||
      !positive(c.filter_time) || !positive(c.reference_linear_speed) ||
      !positive(c.reference_angular_speed) || !positive(c.activation_time) ||
      !positive(c.min_dt) || !positive(c.max_dt) || c.min_dt > c.max_dt ||
      !positive(c.max_position_error) || !positive(c.max_rotation_error) ||
      c.max_rotation_error >= kPi || !positive(c.max_limited_time)) return false;

  const Vec6 damping = (2.0 * c.damping_ratio.array() *
      c.stiffness.array().sqrt() * c.effective_inertia.array().sqrt()).matrix();
  if (!damping.allFinite() || !(c.torque_slew * c.max_dt).allFinite()) return false;
  config_ = c;
  damping_ = damping;
  elapsed_ = limited_time_ = 0.0;
  error_ = Error::None;
  state_ = State::Inactive;
  return true;
}

bool CartesianImpedance::validSample(const Sample& s) const {
  return s.valid && validPose(s.pose) && s.J.allFinite() && s.dq.allFinite() &&
         s.gravity.allFinite() &&
         (s.dq.cwiseAbs().array() <= config_.joint_speed_limit.array()).all();
}

bool CartesianImpedance::validTorque(const Vec6& t) const {
  return t.allFinite() &&
         (t.cwiseAbs().array() <= config_.torque_limit.array()).all();
}

std::optional<Command> CartesianImpedance::fail(Error e) {
  error_ = e;
  state_ = State::Fault;
  return std::nullopt;  // NO valid motor command; caller must handle fault.
}

bool CartesianImpedance::activate(const Sample& s, const Vec6& previous) {
  if (state_ != State::Inactive) return false;
  if (!validSample(s)) { fail(Error::InvalidSample); return false; }
  if (!validTorque(s.gravity)) { fail(Error::GravityLimit); return false; }
  if (!validTorque(previous)) { fail(Error::InvalidPreviousTorque); return false; }
  reference_ = s.pose;
  reference_.q.normalize();
  target_ = reference_;  // Always discard targets from an earlier activation.
  handover_ = previous; // Known TOTAL support torque, not default zero.
  elapsed_ = limited_time_ = 0.0;
  error_ = Error::None;
  state_ = State::Ramping;
  return true;
}

bool CartesianImpedance::setTarget(const Pose& t) {
  if (state_ != State::Active || !validPose(t)) return false;
  if (!within((t.p - reference_.p).norm(), config_.max_position_error) ||
      !within(angle(t.q, reference_.q), config_.max_rotation_error)) return false;
  target_ = t;
  target_.q.normalize();
  return true;
}

void CartesianImpedance::deactivate() {
  if (state_ == State::Active || state_ == State::Ramping) state_ = State::Inactive;
}

std::optional<Command> CartesianImpedance::update(
    const Sample& s, const Vec6& previous, double dt) {
  if (state_ != State::Ramping && state_ != State::Active) return std::nullopt;
  if (!std::isfinite(dt) || dt < config_.min_dt || dt > config_.max_dt)
    return fail(Error::InvalidDt);
  if (!validSample(s)) return fail(Error::InvalidSample);
  if (!validTorque(s.gravity)) return fail(Error::GravityLimit);
  if (!validTorque(previous)) return fail(Error::InvalidPreviousTorque);

  // Exact first-order filter coefficient plus physical reference speed limits.
  const double alpha = -std::expm1(-dt / config_.filter_time);
  const Eigen::Vector3d delta = target_.p - reference_.p;
  const double distance = delta.norm();
  const double theta = angle(reference_.q, target_.q);
  if (!std::isfinite(distance) || !std::isfinite(theta))
    return fail(Error::NumericalError);
  const double ap = distance > 0.0 ?
      std::min(alpha, config_.reference_linear_speed * dt / distance) : 0.0;
  const double ar = theta > 0.0 ?
      std::min(alpha, config_.reference_angular_speed * dt / theta) : 0.0;
  reference_.p += ap * delta;
  reference_.q = reference_.q.slerp(ar, target_.q).normalized();

  const Eigen::Quaterniond q = s.pose.q.normalized();
  Vec6 error;
  error.head<3>() = reference_.p - s.pose.p;
  if (!within(error.head<3>().norm(), config_.max_position_error) ||
      !within(angle(q, reference_.q), config_.max_rotation_error))
    return fail(Error::TrackingError);

  Eigen::Quaterniond qe = q.conjugate() * reference_.q;
  if (qe.w() < 0.0) qe.coeffs() *= -1.0;
  // Franka-style quaternion vector in world axes, scaled by 2.
  // Magnitude = 2*sin(theta/2), approximately theta for small rotations.
  error.tail<3>() = 2.0 * (q * qe.vec());

  Command out;
  const Vec6 velocity = s.J * s.dq;
  const Vec6 wrench = config_.stiffness.cwiseProduct(error) -
                      damping_.cwiseProduct(velocity);
  if (!wrench.allFinite()) return fail(Error::NumericalError);
  out.wrench = wrench.cwiseMax(-config_.wrench_limit).cwiseMin(config_.wrench_limit);
  out.wrench_limited = (wrench.array() != out.wrench.array()).any();
  const Vec6 desired = s.gravity + s.J.transpose() * out.wrench;
  if (!desired.allFinite()) return fail(Error::NumericalError);

  elapsed_ = std::min(config_.activation_time, elapsed_ + dt);
  const double u = elapsed_ / config_.activation_time;
  out.activation_weight = u * u * (3.0 - 2.0 * u);
  out.raw_torque = (1.0 - out.activation_weight) * handover_ +
                   out.activation_weight * desired;
  if (!out.raw_torque.allFinite()) return fail(Error::NumericalError);

  // Intersection of absolute and rate bounds, based on ACTUALLY sent torque.
  for (int i = 0; i < 6; ++i) {
    const double limit = config_.torque_limit[i];
    const double step = config_.torque_slew[i] * dt;
    const double rate_lo = previous[i] - step, rate_hi = previous[i] + step;
    const double lo = std::max(-limit, rate_lo), hi = std::min(limit, rate_hi);
    out.torque[i] = std::clamp(out.raw_torque[i], lo, hi);
    out.torque_limited |= std::abs(out.raw_torque[i]) > limit;
    out.rate_limited |= out.raw_torque[i] < rate_lo || out.raw_torque[i] > rate_hi;
  }
  const bool limited = out.wrench_limited || out.torque_limited || out.rate_limited;
  limited_time_ = limited ? limited_time_ + dt : 0.0;
  if (limited_time_ >= config_.max_limited_time) return fail(Error::SaturationTimeout);
  if (elapsed_ >= config_.activation_time) state_ = State::Active;
  return out;
}

}  // namespace hw_arm_algorithms::impedance