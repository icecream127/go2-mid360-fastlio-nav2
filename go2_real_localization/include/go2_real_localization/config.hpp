#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_real_localization {
using Transform = Eigen::Matrix4f;

struct Config {
  std::string map_frame = "map", odom_frame = "odom", body_frame = "livox_frame";
  std::string cloud_topic = "/cloud_registered_body", odom_topic = "/Odometry", map_path;
  double voxel_size = 0.15, local_radius = 25.0, registration_period = 1.0;
  double max_input_age = 2.0, transform_timeout = 3.0;
  int max_iterations = 40, num_threads = 4, min_points = 100;
  double correspondence_distance = 1.0, inlier_distance = 0.30;
  double min_inlier_ratio = 0.60, max_rmse = 0.20;
  double max_correction_translation = 0.5, max_correction_rotation = 0.35;
  double initial_max_translation = 2.0, initial_max_rotation = 0.8;
  // Normal-based point-to-plane geometry check, not a global uniqueness test.
  double min_observability_ratio = 1e-3;
  bool initial_pose_enabled = false;
  std::vector<double> initial_pose{0, 0, 0, 0, 0, 0};

  void validate() const {
    for (double v : {voxel_size, local_radius, registration_period, max_input_age,
        transform_timeout, correspondence_distance, inlier_distance, min_inlier_ratio,
        max_rmse, max_correction_translation, max_correction_rotation,
        initial_max_translation, initial_max_rotation}) {
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Parameters must be finite and positive");
    }
    if (registration_period < 1e-6 || registration_period > 86400 ||
        max_iterations < 1 || num_threads < 1 || min_points < 20 || min_inlier_ratio > 1 ||
        !std::isfinite(min_observability_ratio) || min_observability_ratio < 0 ||
        min_observability_ratio > 1) throw std::invalid_argument("Invalid GICP limits");
    if (map_frame.empty() || odom_frame.empty() || body_frame.empty() || map_frame == odom_frame ||
        map_frame == body_frame || odom_frame == body_frame || cloud_topic.empty() || odom_topic.empty())
      throw std::invalid_argument("Topics and distinct frame names are required");
    if (initial_pose.size() != 6 || !std::all_of(initial_pose.begin(), initial_pose.end(),
        [](double v) { return std::isfinite(v); })) throw std::invalid_argument("initial_pose needs six finite values");
  }
};
}  // namespace go2_real_localization
