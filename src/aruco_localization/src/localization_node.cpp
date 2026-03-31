#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
  : Node("aruco_localization_node"), has_correction_(false)
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

    localization_sub_ = create_subscription<aruco_interfaces::msg::DetectedTagArray>(
      "/localization_tags", 10,
      std::bind(&LocalizationNode::tags_callback, this, std::placeholders::_1));

    double rate = get_parameter("broadcast_rate_hz").as_double();
    broadcast_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&LocalizationNode::broadcast_tf, this));

    RCLCPP_INFO(get_logger(), "aruco_localization_node started");
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
    // Always rebroadcast last known correction so nav2 doesn't timeout
    last_tf_.header.stamp = now();
    tf_broadcaster_->sendTransform(last_tf_);
  }

  std::map<uint32_t, TagWorldPose>                                   tag_world_poses_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                     tf_broadcaster_;
  rclcpp::Subscription<aruco_interfaces::msg::DetectedTagArray>::SharedPtr localization_sub_;
  rclcpp::TimerBase::SharedPtr                                        broadcast_timer_;
  geometry_msgs::msg::TransformStamped                               last_tf_;
  bool has_correction_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
