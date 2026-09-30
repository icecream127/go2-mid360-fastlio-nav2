#include "go2_real_localization/registration_pipeline.hpp"
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <sstream>

namespace go2_real_localization {
std::string RegistrationResult::summary() const {
  std::ostringstream out;
  out << reason << " rmse_m=" << rmse << " inlier_ratio=" << inlier_ratio
      << " correction_m=" << correction_m << " correction_rad=" << correction_rad
      << " scan_points=" << scan_points << " source_points=" << source_points
      << " map_points=" << map_points << " target_points=" << target_points
      << " aligned_points=" << aligned.size() << " inlier_points=" << inlier_points
      << " registration_ms=" << registration_ms << " observability_ratio=" << observability_ratio
      << " observability_points=" << observability_points;
  return out.str();
}

RegistrationPipeline::RegistrationPipeline(const Config & config) : config_(config) {
  config_.validate();
  Cloud::Ptr raw(new Cloud);
  if (config.map_path.empty() || pcl::io::loadPCDFile(config.map_path, *raw) < 0)
    throw std::runtime_error("Cannot load map_path: " + config.map_path);
  map_ = filtered(raw);
  if (map_->size() < static_cast<size_t>(config_.min_points)) throw std::runtime_error("Map too small");
}

RegistrationPipeline::RegistrationPipeline(const Config & config, const Cloud::ConstPtr & map) : config_(config) {
  config_.validate(); map_ = filtered(map);
  if (map_->size() < static_cast<size_t>(config_.min_points)) throw std::runtime_error("Map too small");
}

Cloud::Ptr RegistrationPipeline::filtered(const Cloud::ConstPtr & input, double radius) const {
  Cloud::Ptr clean(new Cloud), output(new Cloud);
  for (const auto & point : *input) {
    if (pcl::isFinite(point) && (radius <= 0 || point.getVector3fMap().squaredNorm() <= radius * radius))
      clean->push_back(point);
  }
  if (clean->empty()) return output;
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
  voxel.setInputCloud(clean); voxel.filter(*output);
  return output;
}

RegistrationResult RegistrationPipeline::run(const Cloud::ConstPtr & scan,
    const Transform & map_T_body_guess, bool initial) const {
  RegistrationResult result;
  result.scan_points = scan->size(); result.map_points = map_->size();
  auto source = filtered(scan, config_.local_radius);
  Cloud::Ptr target(new Cloud);
  const Eigen::Vector3f center = map_T_body_guess.block<3, 1>(0, 3);
  const double allowed_translation = initial ? config_.initial_max_translation : config_.max_correction_translation;
  // Include a halo for all allowed translations and correspondence searches.
  const double target_radius = config_.local_radius + allowed_translation + config_.correspondence_distance;
  for (const auto & point : *map_)
    if ((point.getVector3fMap() - center).squaredNorm() <= target_radius * target_radius) target->push_back(point);
  result.source_points = source->size(); result.target_points = target->size();
  if (source->size() < static_cast<size_t>(config_.min_points) || target->size() < static_cast<size_t>(config_.min_points)) {
    result.reason = "INSUFFICIENT_POINTS"; return result;
  }
  fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setNumThreads(config_.num_threads); gicp.setMaximumIterations(config_.max_iterations);
  gicp.setMaxCorrespondenceDistance(config_.correspondence_distance);
  gicp.setTransformationEpsilon(5e-4); gicp.setCorrespondenceRandomness(20);
  gicp.setInputSource(source); gicp.setInputTarget(target);
  const auto start = std::chrono::steady_clock::now();
  gicp.align(result.aligned, map_T_body_guess);
  result.registration_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  result.map_T_body = gicp.getFinalTransformation();
  if (!gicp.hasConverged() || !result.map_T_body.allFinite()) { result.reason = "NOT_CONVERGED"; return result; }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree; tree.setInputCloud(target);
  std::vector<int> indices(20); std::vector<float> distances(20);
  double sum = 0, lever_squared = 0;
  struct Constraint { Eigen::Vector3d lever, normal; };
  std::vector<Constraint> constraints;
  for (const auto & point : result.aligned) {
    if (!pcl::isFinite(point) || tree.nearestKSearch(point, 20, indices, distances) < 1 ||
        distances[0] > config_.inlier_distance * config_.inlier_distance) continue;
    ++result.inlier_points; sum += distances[0];
    // Local surface normals supply a geometry check independent of GICP's
    // covariance regularization, which can conceal an unconstrained plane.
    if (indices.size() < 6) continue;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int index : indices) mean += (*target)[index].getVector3fMap().cast<double>();
    mean /= indices.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index : indices) {
      const Eigen::Vector3d offset = (*target)[index].getVector3fMap().cast<double>() - mean;
      covariance.noalias() += offset * offset.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues()[1] <= 1e-9 ||
        eigen.eigenvalues()[0] > 0.05 * eigen.eigenvalues()[1]) continue;
    const Eigen::Vector3d lever = point.getVector3fMap().cast<double>() - result.map_T_body.block<3, 1>(0, 3).cast<double>();
    constraints.push_back({lever, eigen.eigenvectors().col(0)}); lever_squared += lever.squaredNorm();
  }
  result.inlier_ratio = double(result.inlier_points) / source->size();
  if (result.inlier_points) result.rmse = std::sqrt(sum / result.inlier_points);
  result.observability_points = constraints.size();
  if (!constraints.empty()) {
    const double scale = std::max(1.0, std::sqrt(lever_squared / constraints.size()));
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    for (const auto & constraint : constraints) {
      Eigen::Matrix<double, 6, 1> jacobian;
      jacobian.head<3>() = constraint.lever.cross(constraint.normal) / scale;
      jacobian.tail<3>() = constraint.normal;
      information.noalias() += jacobian * jacobian.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(information);
    if (eigen.info() == Eigen::Success && eigen.eigenvalues()[5] > 0)
      result.observability_ratio = std::max(0.0, eigen.eigenvalues()[0]) / eigen.eigenvalues()[5];
  }
  const Transform correction = map_T_body_guess.inverse() * result.map_T_body;
  result.correction_m = correction.block<3, 1>(0, 3).norm();
  result.correction_rad = Eigen::AngleAxisf(correction.block<3, 3>(0, 0)).angle();
  if (result.inlier_ratio < config_.min_inlier_ratio || result.rmse > config_.max_rmse ||
      result.correction_m > allowed_translation ||
      result.correction_rad > (initial ? config_.initial_max_rotation : config_.max_correction_rotation)) {
    result.reason = "REJECTED";
  } else if (config_.min_observability_ratio > 0 &&
      (constraints.size() < static_cast<size_t>(config_.min_points) || result.observability_ratio < config_.min_observability_ratio)) {
    result.reason = "DEGENERATE";
  } else { result.accepted = true; result.reason = "ACCEPTED"; }
  return result;
}
}  // namespace go2_real_localization
