#include <deque>
#include <atomic>
#include <thread>
#include <numeric>
#include <Eigen/Core>

// #define GLIM_ROS2

#include <boost/format.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>

#include <glim/util/extension_module_ros.hpp>
// #include <geometry_msgs/PoseWithCovarianceStamped.hpp>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>

using ExtensionModuleBase = glim::ExtensionModuleROS;

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/util/logging.hpp>
#include <glim/util/convert_to_string.hpp>
//#include <glim_ext/util/config_ext.hpp>
#include <gazel_nav_tools/utils.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

/**
 * @brief Naive implementation of GNSS constraints for the global optimization.
 * @note  This implementation is very naive and ignores the IMU-GNSS transformation and GNSS observation covariance.
 *        If you use a precise GNSS (e.g., RTK), consider asking for a closed-source extension module with better GNSS handling.
 */
class GNSSGlobal : public ExtensionModuleBase {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSGlobal() : logger(create_module_logger("gnss_global")), tf_buffer(ros::Duration(100.)), tf_listener(tf_buffer)   {
    logger->info("initializing GNSS global constraints");
    // const std::string config_path = glim::GlobalConfigExt::get_config_path("config_gnss_global");
    // logger->info("gnss_global_config_path={}", config_path);

    // glim::Config config(config_path);
    gnss_topic = "/gkv_global/odom";
    // prior_inf_scale = config.param<Eigen::Vector3d>("gnss", "prior_inf_scale", Eigen::Vector3d(1e3, 1e3, 0.0));
    // min_baseline = config.param<double>("gnss", "min_baseline", 5.0);

    transformation_initialized = false;
    T_world_utm.setIdentity();

    T_odom_odom_ned = gtsam::Pose3(tf2::transformToEigen(tf_buffer.lookupTransform("odom", "odom_ned", ros::Time(0), ros::Duration(5))).matrix());
    T_gkv_imu = gtsam::Pose3(tf2::transformToEigen(tf_buffer.lookupTransform("gkv", "os_imu_top", ros::Time(0), ros::Duration(5))).matrix());


    kill_switch = false;
    thread = std::thread([this] { backend_task(); });


    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GNSSGlobal::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GNSSGlobal::on_smoother_update, this, _1, _2, _3));
  }
  ~GNSSGlobal() {
    kill_switch = true;
    thread.join();
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override {
    const auto sub = std::make_shared<TopicSubscription<nav_msgs::Odometry>>(gnss_topic, [this](const nav_msgs::Odometry::ConstPtr msg) { gnss_callback(msg); });
    return {sub};
  }

  void gnss_callback(const nav_msgs::OdometryConstPtr& odom_msg) {
    Eigen::Vector4d gnss_data;
    const double stamp = odom_msg->header.stamp.toSec();
    // const auto& pos = gnss_msg->pose.pose.position;
    // gnss_data << stamp, pos.x, pos.y, pos.z;
    // input_gnss_queue.push_back(gnss_data);
    auto [pose_odom_ned_gkv, cov_odom_ned_gkv_gkv] =  gazel_nav_tools::ros_pose_with_cov_to_gtsam(odom_msg->pose);



    gtsam::Matrix66 adj_gkv_imu =  T_gkv_imu.AdjointMap();
    auto pose_odom_ned_bl = pose_odom_ned_gkv.compose(T_gkv_imu);
    gtsam::Matrix66 cov_odom_ned_imu_imu = adj_gkv_imu * cov_odom_ned_gkv_gkv * adj_gkv_imu.transpose();
    gtsam::Matrix66 &cov_odom_imu_imu = cov_odom_ned_imu_imu;
    auto pose_odom_bl = T_odom_odom_ned.compose(pose_odom_ned_bl);

    input_gnss_queue.push_back({{pose_odom_bl, cov_odom_imu_imu}, stamp});

  }

  void on_insert_submap(const SubMap::ConstPtr& submap) { input_submap_queue.push_back(submap); }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    const auto factors = output_factors.get_all_and_clear();
    if (!factors.empty()) {
      logger->debug("insert {} GNSS prior factors", factors.size());
      new_factors.add(factors);
    }
  }

  void backend_task() {
    logger->info("starting GNSS global thread");
    std::deque<std::pair<std::pair<gtsam::Pose3, gtsam::Matrix66>, double>> utm_queue;
    std::deque<SubMap::ConstPtr> submap_queue;

    while (!kill_switch) {
      // Convert GeoPoint(lat/lon) to UTM
      const auto gnss_data = input_gnss_queue.get_all_and_clear();
      utm_queue.insert(utm_queue.end(), gnss_data.begin(), gnss_data.end());

      // Add new submaps
      const auto new_submaps = input_submap_queue.get_all_and_clear();
      if (new_submaps.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());

      // Remove submaps that are created earlier than the oldest GNSS data
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front()->frames.front()->stamp < utm_queue.front().second) {
        submap_queue.pop_front();
      }

      // Interpolate UTM coords and associate with submaps
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front()->frames.front()->stamp > utm_queue.front().second &&
             submap_queue.front()->frames.back()->stamp < utm_queue.back().second) {
        const auto& submap = submap_queue.front();
        const double stamp = submap->frames[submap->frames.size() / 2]->stamp;

        const auto right = std::lower_bound(utm_queue.begin(), utm_queue.end(), stamp, [](const auto& utm, const double t) { return utm.second < t; });
        if (right == utm_queue.end() || (right + 1) == utm_queue.end()) {
          logger->warn("invalid condition in GNSS global module!!");
          break;
        }
        const auto left = right - 1;
        logger->debug("submap={:.6f} utm_left={:.6f} utm_right={:.6f}", stamp, (*left).second, (*right).second);

        const double tl = (*left).second;
        const double tr = (*right).second;
        const double p = (stamp - tl) / (tr - tl);
        // const Eigen::Vector4d interpolated = (1.0 - p) * (*left) + p * (*right);

        submaps.push_back(submap);
        submap_coords.push_back({(*left).first.first.interpolateRt(right->first.first, p), (1.0 - p) * (*left).first.second + p * (*right).first.second});

        submap_queue.pop_front();
        utm_queue.erase(utm_queue.begin(), left);
      }


      // Add translation prior factor
      if (transformation_initialized) {
        // const Eigen::Vector3d xyz = T_world_utm * submap_coords.back().tail<3>();

        auto pose = submap_coords.back();
        logger->debug("submap={} gnss={}", convert_to_string(submaps.back()->T_world_origin.translation().eval()), convert_to_string(pose.first.translation().eval()));

        const auto& submap = submaps.back();
        // note: should use a more accurate information matrix
        const auto model = gtsam::noiseModel::Gaussian::Covariance(pose.second);
        gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PriorFactor<gtsam::Pose3>(X(submap->id),  pose.first, model));
        output_factors.push_back(factor);
      }
    }
  }

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  ConcurrentVector<std::pair<std::pair<gtsam::Pose3, gtsam::Matrix66>, double>> input_gnss_queue;
  ConcurrentVector<SubMap::ConstPtr> input_submap_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<SubMap::ConstPtr> submaps;
  std::vector<std::pair<gtsam::Pose3, gtsam::Matrix66>> submap_coords;

  std::string gnss_topic;
  Eigen::Vector3d prior_inf_scale;
  double min_baseline;

  bool transformation_initialized;
  Eigen::Isometry3d T_world_utm;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
private:

  gtsam::Pose3 T_gkv_imu;
  gtsam::Pose3 T_odom_odom_ned;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;


};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSGlobal();
}