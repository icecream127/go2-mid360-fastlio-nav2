#pragma once
#include "go2_real_localization/config.hpp"
#include <cstdint>
#include <optional>

namespace go2_real_localization {
struct InputSnapshot {
  uint64_t generation = 0, sequence = 0;
  int64_t stamp_ns = 0;
  bool initial = false;
  Transform map_T_body_guess = Transform::Identity();
  Transform odom_T_body = Transform::Identity();
};

// Owned only by the ROS executor. No clouds, publishers or ROS clocks here.
class LocalizationState {
public:
  explicit LocalizationState(const Config & config) : config_(config) {}
  bool observe_input(int64_t stamp_ns);
  void set_initial_pose(const Transform & map_T_body);
  void reset(const std::string & reason);
  void invalidate(const std::string & reason);
  std::optional<InputSnapshot> prepare(int64_t stamp_ns, const Transform & odom_T_body, int64_t now_ns);
  bool current(const InputSnapshot & input) const { return input.generation == generation_; }
  bool accept(const InputSnapshot & input, const Transform & map_T_body, int64_t now_ns);
  bool expire(int64_t now_ns);
  bool fresh(int64_t stamp_ns, int64_t now_ns) const;
  bool valid() const { return valid_; }
  const Transform & map_T_odom() const { return map_T_odom_; }
  const std::string & status() const { return status_; }
private:
  Config config_;
  bool pending_ = false, initialized_ = false, valid_ = false;
  uint64_t generation_ = 0, sequence_ = 0;
  std::optional<uint64_t> submitted_sequence_;
  std::optional<int64_t> last_input_ns_, accepted_ns_;
  Transform initial_map_T_body_ = Transform::Identity(), map_T_odom_ = Transform::Identity();
  std::string status_ = "WAITING_FOR_SYNCHRONIZED_INPUT_AND_INITIALPOSE";
};
}  // namespace go2_real_localization
