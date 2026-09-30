#pragma once
#include "go2_real_localization/config.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>

namespace go2_real_localization {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct RegistrationResult {
  bool accepted = false;
  std::string reason;
  Transform map_T_body = Transform::Identity();
  Cloud aligned;
  size_t scan_points = 0, source_points = 0, map_points = 0, target_points = 0, inlier_points = 0;
  size_t observability_points = 0;
  double rmse = std::numeric_limits<double>::infinity(), inlier_ratio = 0;
  double correction_m = 0, correction_rad = 0, registration_ms = 0, observability_ratio = 0;
  std::string summary() const;
};

// Immutable map/config. All per-registration data belong to the calling worker.
class RegistrationPipeline {
public:
  explicit RegistrationPipeline(const Config & config);
  RegistrationPipeline(const Config & config, const Cloud::ConstPtr & map);
  Cloud::ConstPtr map() const { return map_; }
  RegistrationResult run(const Cloud::ConstPtr & scan, const Transform & map_T_body_guess, bool initial) const;
private:
  Cloud::Ptr filtered(const Cloud::ConstPtr & input, double radius = 0) const;
  Config config_;
  Cloud::ConstPtr map_;
};
}  // namespace go2_real_localization
