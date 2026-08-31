#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/geometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

class PriorMapLocalizer final : public rclcpp::Node
{
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

public:
  PriorMapLocalizer()
  : Node("prior_map_localizer"), tf_broadcaster_(this)
  {
    map_path_ = declare_parameter<std::string>(
      "map_path", "/home/zbx/.config/ranger_nav/maps/real_lab_3d_level_full.pcd");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    lio_topic_ = declare_parameter<std::string>("lio_topic", "/Odometry");
    wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    map_voxel_ = declare_parameter<double>("map_voxel", 0.15);
    scan_voxel_ = declare_parameter<double>("scan_voxel", 0.15);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 0.80);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.10);
    ndt_epsilon_ = declare_parameter<double>("ndt_epsilon", 0.01);
    ndt_iterations_ = declare_parameter<int>("ndt_iterations", 20);
    max_fitness_score_ = declare_parameter<double>("max_fitness_score", 2.0);
    process_every_n_ = std::max(
      1, static_cast<int>(declare_parameter<int>("process_every_n", 10)));
    publish_map_ = declare_parameter<bool>("publish_map", true);

    const auto tx = declare_parameter<double>("base_to_body_x", 0.30);
    const auto ty = declare_parameter<double>("base_to_body_y", 0.0);
    const auto tz = declare_parameter<double>("base_to_body_z", 0.70);
    const auto roll = declare_parameter<double>("base_to_body_roll", 0.0);
    const auto pitch = declare_parameter<double>("base_to_body_pitch", 0.523599);
    const auto yaw = declare_parameter<double>("base_to_body_yaw", 0.0);
    base_to_body_ = make_transform(tx, ty, tz, roll, pitch, yaw);

    const auto initial_x = declare_parameter<double>("initial_x", 0.30);
    const auto initial_y = declare_parameter<double>("initial_y", 0.0);
    const auto initial_z = declare_parameter<double>("initial_z", 0.70);
    const auto initial_roll = declare_parameter<double>("initial_roll", 0.0);
    const auto initial_pitch = declare_parameter<double>("initial_pitch", 0.523599);
    const auto initial_yaw = declare_parameter<double>("initial_yaw", 0.0);
    last_map_camera_ = make_transform(
      initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw);

    target_cloud_.reset(new Cloud);
    if (pcl::io::loadPCDFile<Point>(map_path_, *target_cloud_) != 0 || target_cloud_->empty()) {
      throw std::runtime_error("cannot load prior map: " + map_path_);
    }
    voxelize(target_cloud_, map_voxel_);
    if (target_cloud_->size() < 100) {
      throw std::runtime_error("prior map has too few usable points: " + map_path_);
    }

    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(ndt_epsilon_);
    ndt_.setMaximumIterations(ndt_iterations_);
    ndt_.setInputTarget(target_cloud_);

    prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/prior_map", rclcpp::QoS(1).transient_local().reliable());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/prior_pose", 10);
    ndt_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ndt_options;
    ndt_options.callback_group = ndt_group_;
    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = state_group_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&PriorMapLocalizer::cloud_callback, this, std::placeholders::_1), ndt_options);
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lio_topic_, 20,
      std::bind(&PriorMapLocalizer::lio_callback, this, std::placeholders::_1), state_options);
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, 20,
      std::bind(&PriorMapLocalizer::wheel_callback, this, std::placeholders::_1), state_options);
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&PriorMapLocalizer::initial_pose_callback, this, std::placeholders::_1), state_options);

    if (publish_map_) {
      sensor_msgs::msg::PointCloud2 map_msg;
      pcl::toROSMsg(*target_cloud_, map_msg);
      map_msg.header.frame_id = map_frame_;
      map_msg.header.stamp = now();
      prior_map_pub_->publish(map_msg);
    }

    RCLCPP_INFO(get_logger(),
      "loaded prior 3D map %s (%zu points), input=%s, initial=(%.2f, %.2f, %.2f)",
      map_path_.c_str(), target_cloud_->size(), cloud_topic_.c_str(),
      initial_x, initial_y, initial_z);
    RCLCPP_INFO(get_logger(),
      "publishing the only map->odom TF; FAST-LIO remains local odometry");
  }

private:
  static Eigen::Matrix4f make_transform(
    double x, double y, double z, double roll, double pitch, double yaw)
  {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = Eigen::Vector3f(
      static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    transform *= Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
    transform *= Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
    transform *= Eigen::AngleAxisf(static_cast<float>(roll), Eigen::Vector3f::UnitX());
    return transform.matrix();
  }

  static void voxelize(const Cloud::Ptr & cloud, double leaf)
  {
    if (leaf <= 0.0 || cloud->empty()) {
      return;
    }
    pcl::VoxelGrid<Point> filter;
    filter.setInputCloud(cloud);
    const auto l = static_cast<float>(leaf);
    filter.setLeafSize(l, l, l);
    Cloud filtered;
    filter.filter(filtered);
    cloud->swap(filtered);
  }

  static Eigen::Matrix4f pose_matrix(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    Eigen::Quaternionf rotation(
      static_cast<float>(q.w), static_cast<float>(q.x),
      static_cast<float>(q.y), static_cast<float>(q.z));
    if (rotation.norm() < 1e-6f) {
      rotation = Eigen::Quaternionf::Identity();
    } else {
      rotation.normalize();
    }
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    result(0, 3) = static_cast<float>(p.x);
    result(1, 3) = static_cast<float>(p.y);
    result(2, 3) = static_cast<float>(p.z);
    return result;
  }

  static Eigen::Matrix4f pose_matrix(const geometry_msgs::msg::Pose & pose)
  {
    const auto & p = pose.position;
    const auto & q = pose.orientation;
    Eigen::Quaternionf rotation(
      static_cast<float>(q.w), static_cast<float>(q.x),
      static_cast<float>(q.y), static_cast<float>(q.z));
    if (rotation.norm() < 1e-6f) {
      rotation = Eigen::Quaternionf::Identity();
    } else {
      rotation.normalize();
    }
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    result(0, 3) = static_cast<float>(p.x);
    result(1, 3) = static_cast<float>(p.y);
    result(2, 3) = static_cast<float>(p.z);
    return result;
  }

  static geometry_msgs::msg::Quaternion quaternion_msg(const Eigen::Matrix4f & matrix)
  {
    const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
    geometry_msgs::msg::Quaternion result;
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    result.w = q.w();
    return result;
  }

  void lio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    bool applied_initial_pose = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_lio_ = msg;
      if (pending_initial_pose_) {
        last_map_camera_ =
          pose_matrix(pending_initial_pose_->pose.pose) * base_to_body_ * pose_matrix(*msg).inverse();
        pending_initial_pose_.reset();
        localized_ = false;
        last_score_ = std::numeric_limits<double>::infinity();
        ++pose_generation_;
        applied_initial_pose = true;
      }
    }
    if (applied_initial_pose) {
      RCLCPP_INFO(get_logger(), "applied RViz initial pose immediately; waiting for 3D alignment");
    }
    publish_tf(msg->header.stamp);
  }

  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_wheel_ = msg;
  }

  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    bool queued = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_lio_) {
        pending_initial_pose_ = msg;
        queued = true;
      } else {
        last_map_camera_ =
          pose_matrix(msg->pose.pose) * base_to_body_ * pose_matrix(*latest_lio_).inverse();
        localized_ = false;
        last_score_ = std::numeric_limits<double>::infinity();
        ++pose_generation_;
      }
    }
    if (queued) {
      RCLCPP_INFO(get_logger(), "RViz initial pose queued until FAST-LIO publishes");
    } else {
      RCLCPP_INFO(get_logger(), "RViz initial pose applied immediately; NDT will refine it in background");
      publish_tf(msg->header.stamp);
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++cloud_count_;
    if (cloud_count_ % process_every_n_ != 0) {
      publish_tf(msg->header.stamp);
      return;
    }

    Cloud::Ptr source(new Cloud);
    try {
      pcl::fromROSMsg(*msg, *source);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cannot decode point cloud: %s", error.what());
      return;
    }
    source->erase(std::remove_if(source->begin(), source->end(), [](const Point & p) {
      return !pcl::isFinite(p);
    }), source->end());
    voxelize(source, scan_voxel_);
    if (source->size() < 80) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "skip localization: scan has only %zu points", source->size());
      return;
    }

    Eigen::Matrix4f guess;
    std::uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      guess = last_map_camera_;
      generation = pose_generation_;
    }
    ndt_.setInputSource(source);
    Cloud aligned;
    ndt_.align(aligned, guess);
    const auto score = ndt_.getFitnessScore();
    if (!ndt_.hasConverged() || !std::isfinite(score) || score > max_fitness_score_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "prior-map alignment rejected: converged=%s score=%.3f threshold=%.3f",
        ndt_.hasConverged() ? "true" : "false", score, max_fitness_score_);
      publish_tf(msg->header.stamp);
      return;
    }

    Eigen::Matrix4f accepted_transform;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (generation != pose_generation_) {
        RCLCPP_INFO(get_logger(), "discarded stale NDT result after a newer RViz initial pose");
        return;
      }
      last_map_camera_ = ndt_.getFinalTransformation();
      last_score_ = score;
      localized_ = true;
      accepted_transform = last_map_camera_;
    }
    publish_tf(msg->header.stamp);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "prior-map localization healthy: score=%.3f map-camera=(%.2f, %.2f, %.2f)",
      score, accepted_transform(0, 3), accepted_transform(1, 3), accepted_transform(2, 3));
  }

  void publish_tf(const builtin_interfaces::msg::Time & stamp)
  {
    Eigen::Matrix4f map_camera;
    std::shared_ptr<nav_msgs::msg::Odometry> lio;
    std::shared_ptr<nav_msgs::msg::Odometry> wheel;
    bool localized;
    double score;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map_camera = last_map_camera_;
      lio = latest_lio_;
      wheel = latest_wheel_;
      localized = localized_;
      score = last_score_;
    }

    geometry_msgs::msg::TransformStamped map_camera_tf;
    map_camera_tf.header.stamp = stamp;
    map_camera_tf.header.frame_id = map_frame_;
    map_camera_tf.child_frame_id = "camera_init";
    map_camera_tf.transform.translation.x = map_camera(0, 3);
    map_camera_tf.transform.translation.y = map_camera(1, 3);
    map_camera_tf.transform.translation.z = map_camera(2, 3);
    map_camera_tf.transform.rotation = quaternion_msg(map_camera);
    tf_broadcaster_.sendTransform(map_camera_tf);
    if (!lio || !wheel) {
      return;
    }

    // T_map_body = T_map_camera_init * T_camera_init_body.
    // T_map_base = T_map_body * inverse(T_base_body).
    const Eigen::Matrix4f map_body = map_camera * pose_matrix(*lio);
    const Eigen::Matrix4f map_base = map_body * base_to_body_.inverse();
    const Eigen::Matrix4f map_odom = map_base * pose_matrix(*wheel).inverse();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = map_odom(0, 3);
    tf.transform.translation.y = map_odom(1, 3);
    tf.transform.translation.z = map_odom(2, 3);
    tf.transform.rotation = quaternion_msg(map_odom);
    tf_broadcaster_.sendTransform(tf);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = tf.header;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = map_base(0, 3);
    pose.pose.pose.position.y = map_base(1, 3);
    pose.pose.pose.position.z = map_base(2, 3);
    pose.pose.pose.orientation = quaternion_msg(map_base);
    const double variance = localized ? std::max(0.001, score) : 10.0;
    pose.pose.covariance[0] = variance;
    pose.pose.covariance[7] = variance;
    pose.pose.covariance[35] = variance;
    pose_pub_->publish(pose);
  }

  std::string map_path_;
  std::string cloud_topic_;
  std::string lio_topic_;
  std::string wheel_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double map_voxel_{0.08};
  double scan_voxel_{0.10};
  double ndt_resolution_{0.50};
  double ndt_step_size_{0.10};
  double ndt_epsilon_{0.01};
  double max_fitness_score_{2.0};
  double last_score_{std::numeric_limits<double>::infinity()};
  int ndt_iterations_{30};
  int process_every_n_{3};
  bool publish_map_{true};
  bool localized_{false};
  std::size_t cloud_count_{0};
  std::uint64_t pose_generation_{0};

  Eigen::Matrix4f base_to_body_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_map_camera_{Eigen::Matrix4f::Identity()};
  Cloud::Ptr target_cloud_;
  pcl::NormalDistributionsTransform<Point, Point> ndt_;
  std::mutex data_mutex_;
  std::shared_ptr<nav_msgs::msg::Odometry> latest_lio_;
  std::shared_ptr<nav_msgs::msg::Odometry> latest_wheel_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pending_initial_pose_;
  rclcpp::CallbackGroup::SharedPtr ndt_group_;
  rclcpp::CallbackGroup::SharedPtr state_group_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PriorMapLocalizer>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    fprintf(stderr, "prior_map_localizer: %s\n", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
