#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
    declare_parameter<double>("speed_threshold_mps",             0.1);
    declare_parameter<double>("correction_smoothing_alpha",      0.1);
    // Camera extrinsics relative to base_link (camera_link-style axes:
    // X-forward, Y-left, Z-up), plus downward pitch in degrees.
    declare_parameter<double>("camera_offset_x",                 0.12);
    declare_parameter<double>("camera_offset_y",                 0.0);
    declare_parameter<double>("camera_offset_z",                 0.30);
    declare_parameter<double>("camera_pitch_deg",                45.0);
    // Reject an ArUco correction if it implies a jump this large from the
    // current best-estimate pose (odom composed with the live map->odom
    // correction) — guards against the upstream detector occasionally
    // locking onto the wrong planar-marker pose-ambiguity solution.
    declare_parameter<double>("max_pose_jump_m",                 0.5);
    declare_parameter<double>("max_yaw_jump_rad",                0.5);

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

    // base_T_camera_optical_: static transform from base_link to the
    // camera's optical frame (X-right, Y-down, Z-forward — the convention
    // cv::aruco/solvePnP publish tag_pose in). Built in two steps:
    //  1) base_link -> camera_link (X-fwd,Y-left,Z-up): mounting offset +
    //     downward pitch.
    //  2) camera_link -> camera_optical_frame: the fixed REP 103 axis
    //     permutation (optical_X=-link_Y, optical_Y=-link_Z, optical_Z=link_X),
    //     i.e. the standard quaternion (-0.5, 0.5, -0.5, 0.5).
    {
      double cam_x = get_parameter("camera_offset_x").as_double();
      double cam_y = get_parameter("camera_offset_y").as_double();
      double cam_z = get_parameter("camera_offset_z").as_double();
      double cam_pitch = get_parameter("camera_pitch_deg").as_double() * M_PI / 180.0;

      tf2::Quaternion q_mount;
      q_mount.setRPY(0, cam_pitch, 0);
      tf2::Transform base_T_camera_link(q_mount, tf2::Vector3(cam_x, cam_y, cam_z));

      tf2::Quaternion q_optical(-0.5, 0.5, -0.5, 0.5);
      tf2::Transform camera_link_T_optical(q_optical, tf2::Vector3(0, 0, 0));

      base_T_camera_optical_ = base_T_camera_link * camera_link_T_optical;
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
        robot_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
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

    // A manual /initialpose reset should take effect immediately, not ease in,
    // so it sets both the published transform and the smoothing target.
    last_tf_ = tf;
    target_tf_ = tf;
    has_target_ = true;
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
    // ArUco corrections are only trustworthy while the robot is essentially
    // stationary (motion blur / detection latency otherwise bias the tag
    // pose), so skip updating the correction target above this speed.
    double speed_threshold = get_parameter("speed_threshold_mps").as_double();
    if (robot_speed_ > speed_threshold) return;

    double conf_thr = get_parameter("confidence_threshold").as_double();

    for (const auto& tag : msg->tags) {
      if (tag.confidence < conf_thr) continue;
      auto it = tag_world_poses_.find(tag.tag_id);
      if (it == tag_world_poses_.end()) continue;

      const TagWorldPose& world = it->second;

      // tag_pose is camera_optical_T_tag: the tag's pose as measured in the
      // camera's optical frame. Chain through the static camera extrinsics
      // to get base_T_tag, then invert and compose with the tag's known
      // world pose to get world_T_base. This is a full 3D composition (the
      // camera's 45° downward pitch means x/y/z all mix), unlike a flat 2D
      // approximation.
      const auto& pos = tag.tag_pose.position;
      const auto& ori = tag.tag_pose.orientation;

      tf2::Transform camera_T_tag(
        tf2::Quaternion(ori.x, ori.y, ori.z, ori.w),
        tf2::Vector3(pos.x, pos.y, pos.z));

      tf2::Transform base_T_tag = base_T_camera_optical_ * camera_T_tag;

      tf2::Transform world_T_tag(
        tf2::Quaternion(tf2::Vector3(0, 0, 1), world.theta),
        tf2::Vector3(world.x, world.y, 0.0));

      tf2::Transform world_T_base = world_T_tag * base_T_tag.inverse();

      tf2::Vector3 origin = world_T_base.getOrigin();
      double robot_world_x = origin.x();
      double robot_world_y = origin.y();

      double roll, pitch, robot_world_yaw;
      tf2::Matrix3x3(world_T_base.getRotation()).getRPY(roll, pitch, robot_world_yaw);

      // Plausibility gate: with a single resolved tag_pose (no second
      // candidate/reprojection-error from the upstream detector), we can't
      // choose between the two classic planar-marker pose-ambiguity
      // solutions directly. Instead, reject this correction if it implies a
      // jump too large from the current best-estimate pose (odom composed
      // with the live map->odom correction) — the practical equivalent of
      // picking the solution closest to the current pose.
      tf2::Transform map_T_odom_current(
        tf2::Quaternion(
          last_tf_.transform.rotation.x, last_tf_.transform.rotation.y,
          last_tf_.transform.rotation.z, last_tf_.transform.rotation.w),
        tf2::Vector3(
          last_tf_.transform.translation.x, last_tf_.transform.translation.y,
          last_tf_.transform.translation.z));
      tf2::Transform odom_T_base_current(
        tf2::Quaternion(tf2::Vector3(0, 0, 1), odom_yaw_),
        tf2::Vector3(odom_x_, odom_y_, 0.0));
      tf2::Transform map_T_base_current = map_T_odom_current * odom_T_base_current;

      tf2::Vector3 cur_origin = map_T_base_current.getOrigin();
      double cur_roll, cur_pitch, cur_yaw;
      tf2::Matrix3x3(map_T_base_current.getRotation()).getRPY(cur_roll, cur_pitch, cur_yaw);

      double pos_jump = std::hypot(robot_world_x - cur_origin.x(), robot_world_y - cur_origin.y());
      double yaw_jump = std::atan2(
        std::sin(robot_world_yaw - cur_yaw), std::cos(robot_world_yaw - cur_yaw));

      double max_pos_jump = get_parameter("max_pose_jump_m").as_double();
      double max_yaw_jump = get_parameter("max_yaw_jump_rad").as_double();
      if (has_target_ && (pos_jump > max_pos_jump || std::fabs(yaw_jump) > max_yaw_jump)) {
        RCLCPP_WARN(get_logger(),
          "Rejecting correction from tag %u: implies %.3fm / %.3frad jump from current pose",
          tag.tag_id, pos_jump, yaw_jump);
        continue;
      }

      // map->odom must account for whatever odom->base_link the robot has
      // already accumulated, not just publish world_T_base straight as the
      // correction (that's only correct if odom->base_link is identity).
      tf2::Transform map_T_odom_new = world_T_base * odom_T_base_current.inverse();
      tf2::Vector3 odom_origin = map_T_odom_new.getOrigin();

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = msg->header.stamp;
      tf.header.frame_id = "map";
      tf.child_frame_id  = "odom";

      tf.transform.translation.x = odom_origin.x();
      tf.transform.translation.y = odom_origin.y();
      tf.transform.translation.z = 0.0;

      const auto & q = map_T_odom_new.getRotation();
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();

      // Don't snap last_tf_ directly: stash as the smoothing target so
      // broadcast_tf() can ease the published transform toward it.
      target_tf_  = tf;
      has_target_ = true;
      has_correction_ = true;
      RCLCPP_DEBUG(get_logger(),
        "map→odom updated from tag %u: x=%.3f y=%.3f yaw=%.3f",
        tag.tag_id, robot_world_x, robot_world_y, robot_world_yaw);
    }
  }

  void broadcast_tf()
  {
    if (!has_correction_) return;

    if (has_target_) {
      // Ease the published transform toward the latest accepted correction
      // instead of snapping, so map->odom doesn't jump when a new tag fix
      // lands. alpha is the fraction of the remaining gap closed per tick.
      double alpha = get_parameter("correction_smoothing_alpha").as_double();

      auto & t = last_tf_.transform.translation;
      const auto & tt = target_tf_.transform.translation;
      t.x += alpha * (tt.x - t.x);
      t.y += alpha * (tt.y - t.y);

      tf2::Quaternion q_last(
        last_tf_.transform.rotation.x, last_tf_.transform.rotation.y,
        last_tf_.transform.rotation.z, last_tf_.transform.rotation.w);
      tf2::Quaternion q_target(
        target_tf_.transform.rotation.x, target_tf_.transform.rotation.y,
        target_tf_.transform.rotation.z, target_tf_.transform.rotation.w);
      tf2::Quaternion q_smoothed = q_last.slerp(q_target, alpha);
      q_smoothed.normalize();

      last_tf_.transform.rotation.x = q_smoothed.x();
      last_tf_.transform.rotation.y = q_smoothed.y();
      last_tf_.transform.rotation.z = q_smoothed.z();
      last_tf_.transform.rotation.w = q_smoothed.w();
    }

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
  geometry_msgs::msg::TransformStamped                               target_tf_;
  bool has_correction_;
  bool has_target_ = false;
  double odom_x_, odom_y_, odom_yaw_;
  double robot_speed_ = 0.0;
  tf2::Transform base_T_camera_optical_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
