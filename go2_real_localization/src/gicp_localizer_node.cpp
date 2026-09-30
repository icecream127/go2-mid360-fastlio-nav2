#include "go2_real_localization/gicp_localizer_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/create_timer.hpp>
#include <Eigen/Geometry>

namespace go2_real_localization {
namespace {
Transform from_pose(const geometry_msgs::msg::Pose & pose) {
  Eigen::Quaternionf rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6) throw std::invalid_argument("Invalid quaternion");
  Transform transform = Transform::Identity();
  transform.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
  if (!transform.allFinite()) throw std::invalid_argument("Non-finite pose");
  return transform;
}
geometry_msgs::msg::Pose to_pose(const Transform & transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3); pose.position.y = transform(1, 3); pose.position.z = transform(2, 3);
  Eigen::Quaternionf rotation(transform.block<3, 3>(0, 0)); rotation.normalize();
  pose.orientation.x = rotation.x(); pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z(); pose.orientation.w = rotation.w(); return pose;
}
}  // namespace

Config GicpLocalizerNode::read_config() {
  Config config;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Startup configuration; edit YAML and restart the node to change it.";
#define READ_CONFIG(name) config.name = declare_parameter(#name, config.name, descriptor)
  READ_CONFIG(map_frame); READ_CONFIG(odom_frame); READ_CONFIG(body_frame);
  READ_CONFIG(cloud_topic); READ_CONFIG(odom_topic); READ_CONFIG(map_path);
  READ_CONFIG(voxel_size); READ_CONFIG(local_radius); READ_CONFIG(registration_period);
  READ_CONFIG(max_input_age); READ_CONFIG(transform_timeout);
  READ_CONFIG(max_iterations); READ_CONFIG(num_threads); READ_CONFIG(min_points);
  READ_CONFIG(correspondence_distance); READ_CONFIG(inlier_distance);
  READ_CONFIG(min_inlier_ratio); READ_CONFIG(max_rmse);
  READ_CONFIG(max_correction_translation); READ_CONFIG(max_correction_rotation);
  READ_CONFIG(initial_max_translation); READ_CONFIG(initial_max_rotation);
  READ_CONFIG(min_observability_ratio); READ_CONFIG(initial_pose_enabled); READ_CONFIG(initial_pose);
#undef READ_CONFIG
  config.validate(); return config;
}

GicpLocalizerNode::GicpLocalizerNode()
    : Node("gicp_localizer"), config_(read_config()), state_(config_), pipeline_(config_) {
  map_pub_ = create_publisher<CloudMsg>("gicp/map", rclcpp::QoS(1).transient_local());
  aligned_pub_ = create_publisher<CloudMsg>("gicp/aligned_cloud", 1);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gicp/pose", 1);
  status_pub_ = create_publisher<std_msgs::msg::String>("gicp/status", rclcpp::QoS(1).transient_local());
  valid_pub_ = create_publisher<std_msgs::msg::Bool>("gicp/valid", rclcpp::QoS(1).transient_local());
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  CloudMsg map_message; pcl::toROSMsg(*pipeline_.map(), map_message);
  map_message.header.frame_id = config_.map_frame; map_message.header.stamp = now(); map_pub_->publish(map_message);
  if (config_.initial_pose_enabled) {
    const auto & p = config_.initial_pose;
    Transform initial_map_T_body = Transform::Identity();
    initial_map_T_body.block<3, 1>(0, 3) = Eigen::Vector3f(p[0], p[1], p[2]);
    initial_map_T_body.block<3, 3>(0, 0) = (Eigen::AngleAxisf(p[5], Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(p[4], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(p[3], Eigen::Vector3f::UnitX())).toRotationMatrix();
    state_.set_initial_pose(initial_map_T_body);
  }
  // The worker never touches ROS publishers, timers or localization state.
  worker_ = std::make_unique<Worker>([this](const Job & job) {
    Result result; result.input = job.input;
    try {
      Cloud::Ptr cloud(new Cloud); pcl::fromROSMsg(*job.cloud, *cloud);
      result.registration = pipeline_.run(cloud, job.input.map_T_body_guess, job.input.initial);
    } catch (const std::exception & error) { result.registration.reason = std::string("GICP_ERROR: ") + error.what(); }
    return result;
  });
  initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
        handle_time_reset();
        if (message->header.frame_id != config_.map_frame) {
          RCLCPP_WARN(get_logger(), "Initial pose must be in map frame"); return;
        }
        try { state_.set_initial_pose(from_pose(message->pose.pose)); }
        catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); return; }
        report(state_.status());
        schedule();
      });
  cloud_sub_.subscribe(this, config_.cloud_topic, rmw_qos_profile_sensor_data);
  odom_sub_.subscribe(this, config_.odom_topic, rmw_qos_profile_sensor_data);
  sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(20), cloud_sub_, odom_sub_);
  sync_->registerCallback(std::bind(&GicpLocalizerNode::on_pair, this, std::placeholders::_1, std::placeholders::_2));
  // All logical scheduling, input age and validity use the same ROS clock.
  registration_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(config_.registration_period),
      [this] { schedule(); });
  tf_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.05), [this] { tick(); });
  rcl_jump_threshold_t threshold{};
  threshold.min_backward.nanoseconds = -1;
  threshold.on_clock_change = true;
  jump_handler_ = get_clock()->create_jump_callback(nullptr,
      [this](const rcl_time_jump_t &) { time_reset_requested_.store(true); }, threshold);
  parameter_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result; result.successful = true;
    for (const auto & parameter : parameters) if (parameter.get_name() == "use_sim_time") {
      result.successful = false; result.reason = "Set use_sim_time at startup; restart to change clock domains.";
    }
    return result;
  });
  report(state_.status());
  RCLCPP_INFO(get_logger(), "Loaded %zu map points; worker uses FastGICP (%d threads)", pipeline_.map()->size(), config_.num_threads);
}

GicpLocalizerNode::~GicpLocalizerNode() { jump_handler_.reset(); worker_.reset(); }

bool GicpLocalizerNode::handle_time_reset() {
  if (!time_reset_requested_.exchange(false)) return false;
  state_.reset("TIME_RESET_NEEDS_INITIALPOSE"); latest_cloud_.reset(); report(state_.status()); return true;
}

void GicpLocalizerNode::on_pair(const CloudMsg::ConstSharedPtr & cloud, const Odom::ConstSharedPtr & odom) {
  handle_time_reset();
  if (cloud->header.frame_id != config_.body_frame || odom->header.frame_id != config_.odom_frame ||
      odom->child_frame_id != config_.body_frame) {
    state_.invalidate("FRAME_MISMATCH"); latest_cloud_.reset(); report(state_.status()); return;
  }
  try { latest_odom_T_body_ = from_pose(odom->pose.pose); }
  catch (const std::exception & e) {
    state_.invalidate("INVALID_ODOMETRY"); latest_cloud_.reset(); report(state_.status()); return;
  }
  if (state_.observe_input(rclcpp::Time(cloud->header.stamp).nanoseconds())) report(state_.status());
  latest_cloud_ = cloud;
}

void GicpLocalizerNode::schedule() {
  handle_time_reset();
  if (worker_->busy()) return;
  if (!latest_cloud_) { report(state_.status()); return; }
  const auto previous_status = state_.status();
  auto input = state_.prepare(rclcpp::Time(latest_cloud_->header.stamp).nanoseconds(), latest_odom_T_body_, now().nanoseconds());
  if (input) worker_->submit(Job{*input, latest_cloud_});
  else if (state_.status() != previous_status) report(state_.status());
}

void GicpLocalizerNode::tick() {
  handle_time_reset();
  const auto current_time = now();
  bool sent_transform = false;
  if (state_.expire(current_time.nanoseconds())) report(state_.status());
  if (auto completed = worker_->take()) {
    if (completed->error) {
      try { std::rethrow_exception(completed->error); }
      catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "Registration failed: %s", e.what()); }
      catch (...) { RCLCPP_ERROR(get_logger(), "Unknown registration error"); }
      state_.invalidate("GICP_ERROR"); report(state_.status());
    } else {
      const auto & result = *completed->value;
      // New seed, clock reset, frame error or expiry cancels old work logically.
      if (state_.current(result.input)) {
        if (!state_.fresh(result.input.stamp_ns, current_time.nanoseconds())) {
          state_.invalidate("STALE_RESULT"); report(state_.status());
        } else if (!result.registration.accepted) {
          state_.invalidate(result.registration.reason); report(result.registration.summary());
        } else if (state_.accept(result.input, result.registration.map_T_body, current_time.nanoseconds())) {
          // Commit state and send its transform before advertising validity.
          broadcast(current_time);
          sent_transform = true;
          CloudMsg cloud; pcl::toROSMsg(result.registration.aligned, cloud);
          cloud.header.frame_id = config_.map_frame;
          cloud.header.stamp = rclcpp::Time(result.input.stamp_ns, get_clock()->get_clock_type());
          aligned_pub_->publish(cloud);
          geometry_msgs::msg::PoseStamped pose; pose.header = cloud.header;
          pose.pose = to_pose(result.registration.map_T_body); pose_pub_->publish(pose);
          report(result.registration.summary());
        } else { report(state_.status()); }
      }
    }
  }
  if (!sent_transform) broadcast(current_time);
}

void GicpLocalizerNode::report(const std::string & message) {
  std_msgs::msg::String status; status.data = message; status_pub_->publish(status);
  std_msgs::msg::Bool valid; valid.data = state_.valid(); valid_pub_->publish(valid);
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

void GicpLocalizerNode::broadcast(const rclcpp::Time & stamp) {
  if (!state_.valid()) return;
  const auto pose = to_pose(state_.map_T_odom());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp; transform.header.frame_id = config_.map_frame; transform.child_frame_id = config_.odom_frame;
  transform.transform.translation.x = pose.position.x; transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z; transform.transform.rotation = pose.orientation;
  broadcaster_->sendTransform(transform);
}
}  // namespace go2_real_localization
