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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

using CloudMsg = sensor_msgs::msg::PointCloud2;
using Odom = nav_msgs::msg::Odometry;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Matrix = Eigen::Matrix4f;
using Policy = message_filters::sync_policies::ExactTime<CloudMsg, Odom>;

Matrix poseMatrix(const geometry_msgs::msg::Pose & p) {
  Eigen::Quaternionf q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) throw std::runtime_error("Invalid quaternion");
  Matrix t = Matrix::Identity();
  t.block<3,3>(0,0) = q.normalized().toRotationMatrix();
  t.block<3,1>(0,3) = Eigen::Vector3f(p.position.x,p.position.y,p.position.z);
  if (!t.allFinite()) throw std::runtime_error("Non-finite pose");
  return t;
}
geometry_msgs::msg::Pose toPose(const Matrix & t) {
  geometry_msgs::msg::Pose p;
  p.position.x=t(0,3); p.position.y=t(1,3); p.position.z=t(2,3);
  Eigen::Quaternionf q(t.block<3,3>(0,0)); q.normalize();
  p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
  return p;
}

class Localizer : public rclcpp::Node {
public:
  Localizer() : Node("gicp_localizer") {
    map_frame_=declare_parameter("map_frame", "map");
    odom_frame_=declare_parameter("odom_frame", "odom");
    body_frame_=declare_parameter("body_frame", "livox_frame");
    if (map_frame_==odom_frame_ || body_frame_==odom_frame_ || body_frame_==map_frame_)
      throw std::runtime_error("Frames must be distinct");
    voxel_=declare_parameter("voxel_size",0.15);
    radius_=declare_parameter("local_radius",25.0);
    period_=declare_parameter("registration_period",1.0);
    max_age_=declare_parameter("max_input_age",2.0);
    ttl_=declare_parameter("transform_timeout",3.0);
    iterations_=declare_parameter("max_iterations",40);
    threads_=declare_parameter("num_threads",4);
    distance_=declare_parameter("correspondence_distance",1.0);
    inlier_distance_=declare_parameter("inlier_distance",0.30);
    ratio_=declare_parameter("min_inlier_ratio",0.60);
    rmse_=declare_parameter("max_rmse",0.20);
    min_points_=declare_parameter("min_points",100);
    correction_t_=declare_parameter("max_correction_translation",0.5);
    correction_r_=declare_parameter("max_correction_rotation",0.35);
    initial_t_=declare_parameter("initial_max_translation",2.0);
    initial_r_=declare_parameter("initial_max_rotation",0.8);
    if (voxel_<=0 || radius_<=0 || period_<=0 || max_age_<=0 || ttl_<=0 ||
        iterations_<1 || threads_<1 || min_points_<20 || distance_<=0 || inlier_distance_<=0 ||
        ratio_<=0 || ratio_>1 || rmse_<=0 || correction_t_<=0 || correction_r_<=0 ||
        initial_t_<=0 || initial_r_<=0) throw std::runtime_error("Invalid GICP parameters");
    auto file=declare_parameter<std::string>("map_path", "");
    Cloud::Ptr raw(new Cloud);
    if (file.empty() || pcl::io::loadPCDFile(file,*raw)<0) throw std::runtime_error("Cannot load map_path: "+file);
    map_=filtered(raw);
    if (map_->size()<static_cast<size_t>(min_points_)) throw std::runtime_error("Map too small");
    map_pub_=create_publisher<CloudMsg>("gicp/map",rclcpp::QoS(1).transient_local());
    aligned_pub_=create_publisher<CloudMsg>("gicp/aligned_cloud",1);
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("gicp/pose",1);
    status_pub_=create_publisher<std_msgs::msg::String>("gicp/status",rclcpp::QoS(1).transient_local());
    valid_pub_=create_publisher<std_msgs::msg::Bool>("gicp/valid",rclcpp::QoS(1).transient_local());
    broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    CloudMsg map_msg; pcl::toROSMsg(*map_,map_msg);
    map_msg.header.frame_id=map_frame_; map_msg.header.stamp=now(); map_pub_->publish(map_msg);
    pending_=declare_parameter("initial_pose_enabled",false);
    auto initial=declare_parameter<std::vector<double>>("initial_pose",{0,0,0,0,0,0});
    if (initial.size()!=6 || !std::all_of(initial.begin(),initial.end(),[](double x){return std::isfinite(x);}))
      throw std::runtime_error("initial_pose must contain finite x,y,z,roll,pitch,yaw");
    seed_=Matrix::Identity();
    seed_.block<3,1>(0,3)=Eigen::Vector3f(initial[0],initial[1],initial[2]);
    seed_.block<3,3>(0,0)=(Eigen::AngleAxisf(initial[5],Eigen::Vector3f::UnitZ())*
      Eigen::AngleAxisf(initial[4],Eigen::Vector3f::UnitY())*
      Eigen::AngleAxisf(initial[3],Eigen::Vector3f::UnitX())).toRotationMatrix();
    initial_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
        if(m->header.frame_id!=map_frame_) {RCLCPP_WARN(get_logger(),"Initial pose must be in map frame"); return;}
        try {seed_=poseMatrix(m->pose.pose);} catch(const std::exception &e){RCLCPP_WARN(get_logger(),"%s",e.what());return;}
        pending_=true; initialized_=false; valid_=false; report("INITIAL_POSE_RECEIVED");
      });
    cloud_sub_.subscribe(this,declare_parameter("cloud_topic", "/cloud_registered_body"),rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this,declare_parameter("odom_topic", "/Odometry"),rmw_qos_profile_sensor_data);
    sync_=std::make_shared<message_filters::Synchronizer<Policy>>(Policy(20),cloud_sub_,odom_sub_);
    sync_->registerCallback(std::bind(&Localizer::onPair,this,std::placeholders::_1,std::placeholders::_2));
    registration_timer_=create_wall_timer(std::chrono::duration<double>(period_),[this]{match();});
    tf_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{broadcast();});
    report("WAITING_FOR_SYNCHRONIZED_INPUT_AND_INITIALPOSE");
    RCLCPP_INFO(get_logger(),"Loaded %zu map points; using fast_gicp::FastGICP (%d threads)",map_->size(),threads_);
  }
private:
  void onPair(const CloudMsg::ConstSharedPtr &c,const Odom::ConstSharedPtr &o) {
    if(c->header.frame_id!=body_frame_ || o->header.frame_id!=odom_frame_ || o->child_frame_id!=body_frame_){
      valid_=false; cloud_.reset(); odom_.reset(); report("FRAME_MISMATCH");return;
    }
    const auto stamp=rclcpp::Time(c->header.stamp).nanoseconds();
    if(last_input_ && stamp<last_input_){pending_=false;initialized_=false;valid_=false;report("TIME_RESET_NEEDS_INITIALPOSE");}
    last_input_=stamp; cloud_=c; odom_=o;
  }
  Cloud::Ptr filtered(const Cloud::Ptr & input) {
    Cloud::Ptr clean(new Cloud), out(new Cloud);
    for(const auto &p:*input) if(pcl::isFinite(p)) clean->push_back(p);
    pcl::VoxelGrid<pcl::PointXYZ> vg; vg.setLeafSize(voxel_,voxel_,voxel_);
    vg.setInputCloud(clean); vg.filter(*out); return out;
  }
  void report(const std::string & s) {
    std_msgs::msg::String msg;msg.data=s;status_pub_->publish(msg);
    std_msgs::msg::Bool b;b.data=valid_;valid_pub_->publish(b);
  }
  void match() {
    if(!cloud_ || !odom_) {valid_=false;report("WAITING_FOR_SYNCHRONIZED_INPUT");return;}
    double age=(now()-rclcpp::Time(cloud_->header.stamp)).seconds();
    if(age<0 || age>max_age_){valid_=false;report("STALE_INPUT");return;}
    if(!pending_ && !initialized_){report("WAITING_FOR_INITIALPOSE");return;}
    if(last_processed_==last_input_) return;
    last_processed_=last_input_;
    Matrix odom_body;
    try {odom_body=poseMatrix(odom_->pose.pose);} catch(const std::exception & e){valid_=false;report(e.what());return;}
    Matrix guess=pending_?seed_:map_odom_*odom_body;
    Cloud::Ptr raw(new Cloud); pcl::fromROSMsg(*cloud_,*raw); auto source=filtered(raw);
    Cloud::Ptr target(new Cloud);
    const Eigen::Vector3f center=guess.block<3,1>(0,3);
    for(const auto &p:*map_) if((p.getVector3fMap()-center).squaredNorm()<radius_*radius_) target->push_back(p);
    if(source->size()<static_cast<size_t>(min_points_) || target->size()<static_cast<size_t>(min_points_)){
      valid_=false;report("INSUFFICIENT_POINTS");return;
    }
    fast_gicp::FastGICP<pcl::PointXYZ,pcl::PointXYZ> gicp;
    gicp.setNumThreads(threads_);
    gicp.setMaximumIterations(iterations_); gicp.setMaxCorrespondenceDistance(distance_);
    // Keep the pinned FastGICP translation convergence tolerance (0.5 mm).
    // A 1e-6 m step threshold failed to converge on the 0.10 m voxel profile.
    gicp.setTransformationEpsilon(5e-4);
    gicp.setCorrespondenceRandomness(20);
    gicp.setInputSource(source);gicp.setInputTarget(target);Cloud aligned;
    try {gicp.align(aligned,guess);} catch(const std::exception &e){valid_=false;report(std::string("GICP_ERROR: ")+e.what());return;}
    Matrix result=gicp.getFinalTransformation();
    if(!gicp.hasConverged() || !result.allFinite()){valid_=false;report("NOT_CONVERGED");return;}
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
    size_t inliers=0;double sum=0;std::vector<int> idx(1);std::vector<float> dist(1);
    for(const auto &p:aligned) if(pcl::isFinite(p) && tree.nearestKSearch(p,1,idx,dist)>0 && dist[0]<=inlier_distance_*inlier_distance_){++inliers;sum+=dist[0];}
    double overlap=double(inliers)/source->size();
    double error=inliers?std::sqrt(sum/inliers):INFINITY;
    Matrix delta=guess.inverse()*result;
    double translation=delta.block<3,1>(0,3).norm();
    double rotation=Eigen::AngleAxisf(delta.block<3,3>(0,0)).angle();
    bool first=pending_ || !initialized_;
    bool accepted=overlap>=ratio_ && error<=rmse_ && translation<=(first?initial_t_:correction_t_) &&
      rotation<=(first?initial_r_:correction_r_) && (now()-rclcpp::Time(cloud_->header.stamp)).seconds()<=max_age_;
    valid_=accepted;
    std::ostringstream info; info<<(accepted?"ACCEPTED":"REJECTED")<<" rmse_m="<<error<<" inlier_ratio="<<overlap
      <<" correction_m="<<translation<<" correction_rad="<<rotation;report(info.str());
    if(!accepted) return;
    map_odom_=result*odom_body.inverse();pending_=false;initialized_=true;
    accepted_stamp_=rclcpp::Time(cloud_->header.stamp).nanoseconds();
    CloudMsg out;pcl::toROSMsg(aligned,out);out.header=cloud_->header;out.header.frame_id=map_frame_;aligned_pub_->publish(out);
    geometry_msgs::msg::PoseStamped pose;pose.header=out.header;pose.pose=toPose(result);pose_pub_->publish(pose);
  }
  void broadcast() {
    if(!valid_) return;
    double age=(now().nanoseconds()-accepted_stamp_)*1e-9;
    if(age<0 || age>ttl_){valid_=false;report("LOCALIZATION_EXPIRED");return;}
    auto p=toPose(map_odom_);geometry_msgs::msg::TransformStamped t;
    t.header.stamp=now();t.header.frame_id=map_frame_;t.child_frame_id=odom_frame_;
    t.transform.translation.x=p.position.x;t.transform.translation.y=p.position.y;t.transform.translation.z=p.position.z;
    t.transform.rotation=p.orientation;broadcaster_->sendTransform(t);
  }
  std::string map_frame_,odom_frame_,body_frame_;
  double voxel_,radius_,period_,max_age_,ttl_,distance_,inlier_distance_,ratio_,rmse_,correction_t_,correction_r_,initial_t_,initial_r_;
  int iterations_,threads_,min_points_;bool pending_=false,initialized_=false,valid_=false;
  int64_t last_input_=0,last_processed_=0,accepted_stamp_=0;
  Matrix seed_=Matrix::Identity(),map_odom_=Matrix::Identity();Cloud::Ptr map_;
  CloudMsg::ConstSharedPtr cloud_;Odom::ConstSharedPtr odom_;
  message_filters::Subscriber<CloudMsg> cloud_sub_;message_filters::Subscriber<Odom> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr map_pub_,aligned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr registration_timer_,tf_timer_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Localizer>());}
  catch(const std::exception &e){RCLCPP_FATAL(rclcpp::get_logger("gicp_localizer"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;}
