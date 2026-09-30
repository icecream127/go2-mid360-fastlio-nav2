#pragma once
#include "go2_real_localization/localization_state.hpp"
#include "go2_real_localization/registration_pipeline.hpp"
#include "go2_real_localization/registration_worker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <atomic>

namespace go2_real_localization {
class GicpLocalizerNode : public rclcpp::Node {
public:
  GicpLocalizerNode();
  ~GicpLocalizerNode() override;
private:
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using Odom = nav_msgs::msg::Odometry;
  using Policy = message_filters::sync_policies::ExactTime<CloudMsg, Odom>;
  struct Job { InputSnapshot input; CloudMsg::ConstSharedPtr cloud; };
  struct Result { InputSnapshot input; RegistrationResult registration; };
  using Worker = RegistrationWorker<Job, Result>;
  Config read_config();
  void on_pair(const CloudMsg::ConstSharedPtr & cloud, const Odom::ConstSharedPtr & odom);
  void schedule();
  void tick();
  bool handle_time_reset();
  void report(const std::string & message);
  void broadcast(const rclcpp::Time & stamp);
  Config config_;
  LocalizationState state_;
  RegistrationPipeline pipeline_;
  CloudMsg::ConstSharedPtr latest_cloud_;
  Transform latest_odom_T_body_ = Transform::Identity();
  message_filters::Subscriber<CloudMsg> cloud_sub_;
  message_filters::Subscriber<Odom> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr map_pub_, aligned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr registration_timer_, tf_timer_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  std::atomic<bool> time_reset_requested_{false};
  std::unique_ptr<Worker> worker_;
};
}  // namespace go2_real_localization
