#include "go2_real_localization/localization_state.hpp"

namespace go2_real_localization {
bool LocalizationState::fresh(int64_t stamp_ns, int64_t now_ns) const {
  const double age = (static_cast<long double>(now_ns) - stamp_ns) * 1e-9L;
  return age >= 0 && age <= config_.max_input_age;
}

bool LocalizationState::observe_input(int64_t stamp_ns) {
  const bool backward = last_input_ns_ && stamp_ns < *last_input_ns_;
  if (backward) reset("TIME_RESET_NEEDS_INITIALPOSE");
  if (!last_input_ns_ || stamp_ns != *last_input_ns_) ++sequence_;
  last_input_ns_ = stamp_ns;
  return backward;
}

void LocalizationState::set_initial_pose(const Transform & map_T_body) {
  ++generation_;
  initial_map_T_body_ = map_T_body;
  pending_ = true; initialized_ = false; valid_ = false;
  submitted_sequence_.reset(); accepted_ns_.reset();
  status_ = "INITIAL_POSE_RECEIVED";
}

void LocalizationState::invalidate(const std::string & reason) {
  ++generation_;  // Work already in flight must not restore validity.
  valid_ = false;
  status_ = reason;
}

void LocalizationState::reset(const std::string & reason) {
  invalidate(reason);
  pending_ = false; initialized_ = false;
  last_input_ns_.reset(); accepted_ns_.reset(); submitted_sequence_.reset();
}

std::optional<InputSnapshot> LocalizationState::prepare(
    int64_t stamp_ns, const Transform & odom_T_body, int64_t now_ns) {
  if (!fresh(stamp_ns, now_ns)) { invalidate("STALE_INPUT"); return std::nullopt; }
  if (!pending_ && !initialized_) { status_ = "WAITING_FOR_INITIALPOSE"; return std::nullopt; }
  if (submitted_sequence_ && *submitted_sequence_ == sequence_) return std::nullopt;
  submitted_sequence_ = sequence_;
  InputSnapshot input;
  input.generation = generation_; input.sequence = sequence_; input.stamp_ns = stamp_ns;
  input.initial = pending_ || !initialized_; input.odom_T_body = odom_T_body;
  input.map_T_body_guess = input.initial ? initial_map_T_body_ : (map_T_odom_ * odom_T_body).eval();
  return input;
}

bool LocalizationState::accept(const InputSnapshot & input, const Transform & map_T_body, int64_t now_ns) {
  if (!current(input)) return false;
  const double age = (static_cast<long double>(now_ns) - input.stamp_ns) * 1e-9L;
  if (!fresh(input.stamp_ns, now_ns) || age > config_.transform_timeout) {
    invalidate("STALE_RESULT"); return false;
  }
  map_T_odom_ = map_T_body * input.odom_T_body.inverse();
  pending_ = false; initialized_ = true; valid_ = true;
  accepted_ns_ = input.stamp_ns; status_ = "ACCEPTED";
  return true;
}

bool LocalizationState::expire(int64_t now_ns) {
  if (!valid_) return false;
  if (last_input_ns_ && !fresh(*last_input_ns_, now_ns)) { invalidate("STALE_INPUT"); return true; }
  const double age = (static_cast<long double>(now_ns) - *accepted_ns_) * 1e-9L;
  if (age < 0 || age > config_.transform_timeout) { invalidate("LOCALIZATION_EXPIRED"); return true; }
  return false;
}
}  // namespace go2_real_localization
