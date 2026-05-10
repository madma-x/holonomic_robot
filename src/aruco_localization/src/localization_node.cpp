#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <aruco_interfaces/msg/detected_tag_array.hpp>

#include <map>
#include <cmath>

struct TagWorldPose { double x, y, theta; };

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : Node("aruco_localization_node"), has_correction_(false),
    odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
  {
    // Parameters: one entry per positioning tag as flat lists
    // tag_ids: [20, 21]
    // tag_world_x: [1.0, 2.0]
    // tag_world_y: [0.5, 1.5]
    // tag_world_theta: [0.0, 1.57]
    declare_parameter<std::vector<long int>>("tag_ids",          std::vector<long int>{});
    declare_parameter<std::vector<double>>("tag_world_x",        std::vector<double>{});
    declare_parameter<std::vector<double>>("tag_world_y",        std::vector<double>{});
    declare_parameter<std::vector<double>>("tag_world_theta",    std::vector<double>{});
    declare_parameter<double>("confidence_threshold",            0.3);
    declare_parameter<double>("broadcast_rate_hz",               10.0);
    declare_parameter<double>("transform_tolerance_sec",         0.2);

    // Build tag world pose map
    auto ids    = get_parameter("tag_ids").as_integer_array();
    auto xs     = get_parameter("tag_world_x").as_double_array();
    auto ys     = get_parameter("tag_world_y").as_double_array();
    auto thetas = get_parameter("tag_world_theta").as_double_array();

    for (size_t i = 0; i < ids.size(); ++i) {
      tag_world_poses_[static_cast<uint32_t>(ids[i])] = {
        xs[i], ys[i], thetas[i]
      };
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publish identity map->odom immediately so the map frame exists for Nav2
    // before the first ArUco sighting or /initialpose message.
    last_tf_.header.stamp = now();
    last_tf_.header.frame_id = "map";
    last_tf_.child_frame_id = "odom";
    last_tf_.transform.translation.x = 0.0;
    last_tf_.transform.translation.y = 0.0;
    last_tf_.transform.translation.z = 0.0;
    last_tf_.transform.rotation.x = 0.0;
    last_tf_.transform.rotation.y = 0.0;
    last_tf_.transform.rotation.z = 0.0;
    last_tf_.transform.rotation.w = 1.0;
    has_correction_ = true;

    localization_sub_ = create_subscription<aruco_interfaces::msg::DetectedTagArray>(
      "/localization_tags", 10,
      std::bind(&LocalizationNode::tags_callback, this, std::placeholders::_1));

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&LocalizationNode::initial_pose_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto & ori = msg->pose.pose.orientation;
        double siny = 2.0 * (ori.w * ori.z + ori.x * ori.y);
        double cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z);
        odom_x_   = msg->pose.pose.position.x;
        odom_y_   = msg->pose.pose.position.y;
        odom_yaw_ = std::atan2(siny, cosy);
      });

    double rate = get_parameter("broadcast_rate_hz").as_double();
    broadcast_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&LocalizationNode::broadcast_tf, this));

    RCLCPP_INFO(get_logger(), "aruco_localization_node started");
  }

  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring /initialpose in frame '%s' (expected 'map')",
        msg->header.frame_id.c_str());
      return;
    }

    // Desired robot pose in map frame
    const auto & ori = msg->pose.pose.orientation;
    double siny = 2.0 * (ori.w * ori.z + ori.x * ori.y);
    double cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z);
    double desired_x   = msg->pose.pose.position.x;
    double desired_y   = msg->pose.pose.position.y;
    double desired_yaw = std::atan2(siny, cosy);

    // map->odom = T_map_base_desired * inv(T_odom_base_current)
    // yaw: desired_yaw - odom_yaw_
    // translation: p_map_odom = p_map_base - R(dyaw) * p_odom_base
    double dyaw = desired_yaw - odom_yaw_;
    double cos_dyaw = std::cos(dyaw);
    double sin_dyaw = std::sin(dyaw);
    double map_odom_x = desired_x - cos_dyaw * odom_x_ + sin_dyaw * odom_y_;
    double map_odom_y = desired_y - sin_dyaw * odom_x_ - cos_dyaw * odom_y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, dyaw);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = map_odom_x;
    tf.transform.translation.y = map_odom_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    last_tf_ = tf;
    has_correction_ = true;
    RCLCPP_INFO(
      get_logger(),
      "map->odom set from /initialpose: desired=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f) -> map_odom=(%.3f,%.3f,%.3f)",
      desired_x, desired_y, desired_yaw,
      odom_x_, odom_y_, odom_yaw_,
      map_odom_x, map_odom_y, dyaw);
  }

private:
  void tags_callback(const aruco_interfaces::msg::DetectedTagArray::SharedPtr msg)
  {
    double conf_thr = get_parameter("confidence_threshold").as_double();

    for (const auto& tag : msg->tags) {
      if (tag.confidence < conf_thr) continue;
      auto it = tag_world_poses_.find(tag.tag_id);
      if (it == tag_world_poses_.end()) continue;

      const TagWorldPose& world = it->second;

      // tag_pose is the transform from camera/robot to tag.
      // We need: robot pose in world = world_T_tag * inv(robot_T_tag)
      // Simplified 2D: extract yaw from tag_pose orientation
      const auto& pos = tag.tag_pose.position;
      const auto& ori = tag.tag_pose.orientation;

      // Yaw from quaternion
      double siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y);
      double cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z);
      double robot_tag_yaw = std::atan2(siny_cosp, cosy_cosp);

      // Robot world yaw = tag world theta - relative yaw to tag
      double robot_world_yaw = world.theta - robot_tag_yaw;

      // Robot world position = tag world pos - rotated robot-to-tag vector
      double cos_w = std::cos(world.theta);
      double sin_w = std::sin(world.theta);
      double robot_world_x = world.x - (pos.x * cos_w - pos.y * sin_w);
      double robot_world_y = world.y - (pos.x * sin_w + pos.y * cos_w);

      // Build map→odom correction:
      // map→odom = map→base_link (from aruco)
      // Actual odom→base_link comes from odometry, but we don't have it here.
      // We publish map→odom such that map→base_link is consistent.
      // For simplicity we publish map→base_link directly as map→odom.
      // (Proper implementation would subtract current odom→base_link transform.)

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = msg->header.stamp;
      tf.header.frame_id = "map";
      tf.child_frame_id  = "odom";

      tf.transform.translation.x = robot_world_x;
      tf.transform.translation.y = robot_world_y;
      tf.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, robot_world_yaw);
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();

      last_tf_    = tf;
      has_correction_ = true;
      RCLCPP_DEBUG(get_logger(),
        "map→odom updated from tag %u: x=%.3f y=%.3f yaw=%.3f",
        tag.tag_id, robot_world_x, robot_world_y, robot_world_yaw);
    }
  }

  void broadcast_tf()
  {
    if (!has_correction_) return;
    // Future-date the localization transform slightly so Nav2 can request
    // transforms at "now" without hitting extrapolation into the future.
    double tolerance_sec = get_parameter("transform_tolerance_sec").as_double();
    last_tf_.header.stamp = now() + rclcpp::Duration::from_seconds(tolerance_sec);
    tf_broadcaster_->sendTransform(last_tf_);
  }

  std::map<uint32_t, TagWorldPose>                                   tag_world_poses_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                     tf_broadcaster_;
  rclcpp::Subscription<aruco_interfaces::msg::DetectedTagArray>::SharedPtr localization_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
  rclcpp::TimerBase::SharedPtr                                        broadcast_timer_;
  geometry_msgs::msg::TransformStamped                               last_tf_;
  bool has_correction_;
  double odom_x_, odom_y_, odom_yaw_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
