#include <atomic>
#include <thread>
#include <memory>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gtsam/geometry/Pose3.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>

namespace glim {

class TimeKeeper;
class CloudPreprocessor;
class OdometryEstimationBase;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

/**
 * @brief glim instance for ROS environments
 */
class GlimROS {
public:
  GlimROS(ros::NodeHandle& nh);
  ~GlimROS();

  const std::vector<std::shared_ptr<ExtensionModule>>& extensions();
  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

  void insert_image(const double stamp, const cv::Mat& image);
  void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void insert_frame(const glim::RawPoints::Ptr& raw_points);
  void insert_raw_gkv(const nav_msgs::Odometry & odom_msg);
  void insert_raw_loc(const geometry_msgs::PoseWithCovarianceStamped & pose_msg);
  void insert_raw_tr(const geometry_msgs::PoseWithCovarianceStamped & pose_msg);

  void wait(bool auto_quit);

  void save(const std::string& path);

private:
  void loop();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  // gtsam::Pose3 T_odom_odom_ned;

  double imu_time_offset;
  double acc_scale;
  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::unique_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;


  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

};

}  // namespace glim