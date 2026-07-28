#include "motion_controller/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace motion_controller
{

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::ControllerException("Unable to lock node in PurePursuitController::configure");
  }

  logger_ = node_->get_logger();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  node_->declare_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->declare_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node_->declare_parameter(plugin_name_ + ".min_lookahead", min_lookahead_);
  node_->declare_parameter(plugin_name_ + ".max_lookahead", max_lookahead_);

  node_->declare_parameter(plugin_name_ + ".k_xy", k_xy_);
  node_->declare_parameter(plugin_name_ + ".k_theta", k_theta_);

  node_->declare_parameter(plugin_name_ + ".max_vx", base_max_vx_);
  node_->declare_parameter(plugin_name_ + ".max_vy", base_max_vy_);
  node_->declare_parameter(plugin_name_ + ".max_wz", max_wz_);

  node_->declare_parameter(plugin_name_ + ".max_ax", max_ax_);
  node_->declare_parameter(plugin_name_ + ".max_ay", max_ay_);
  node_->declare_parameter(plugin_name_ + ".max_aw", max_aw_);

  node_->declare_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tolerance_);
  node_->declare_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);

  node_->declare_parameter(
    plugin_name_ + ".linear_speed_for_wz_scale_start",
    linear_speed_for_wz_scale_start_);
  node_->declare_parameter(
    plugin_name_ + ".linear_speed_for_wz_scale_end",
    linear_speed_for_wz_scale_end_);
  node_->declare_parameter(
    plugin_name_ + ".min_wz_scale_at_high_speed",
    min_wz_scale_at_high_speed_);
  node_->declare_parameter(plugin_name_ + ".output_rotate_deg", output_rotate_deg_);
  node_->declare_parameter(plugin_name_ + ".output_invert_x", output_invert_x_);
  node_->declare_parameter(plugin_name_ + ".output_invert_y", output_invert_y_);
  node_->declare_parameter(plugin_name_ + ".output_invert_wz", output_invert_wz_);
  node_->declare_parameter(
    plugin_name_ + ".enable_local_path_validity_check",
    enable_local_path_validity_check_);
  node_->declare_parameter(
    plugin_name_ + ".local_path_check_distance",
    local_path_check_distance_);
  node_->declare_parameter(
    plugin_name_ + ".local_path_invalid_cost_threshold",
    local_path_invalid_cost_threshold_);
  node_->declare_parameter(
    plugin_name_ + ".local_path_unknown_is_invalid",
    local_path_unknown_is_invalid_);
  node_->declare_parameter(plugin_name_ + ".publish_lookahead_point", publish_lookahead_point_);
  node_->declare_parameter(plugin_name_ + ".lookahead_topic", lookahead_topic_);

  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node_->get_parameter(plugin_name_ + ".min_lookahead", min_lookahead_);
  node_->get_parameter(plugin_name_ + ".max_lookahead", max_lookahead_);

  node_->get_parameter(plugin_name_ + ".k_xy", k_xy_);
  node_->get_parameter(plugin_name_ + ".k_theta", k_theta_);

  node_->get_parameter(plugin_name_ + ".max_vx", base_max_vx_);
  node_->get_parameter(plugin_name_ + ".max_vy", base_max_vy_);
  node_->get_parameter(plugin_name_ + ".max_wz", max_wz_);

  node_->get_parameter(plugin_name_ + ".max_ax", max_ax_);
  node_->get_parameter(plugin_name_ + ".max_ay", max_ay_);
  node_->get_parameter(plugin_name_ + ".max_aw", max_aw_);

  node_->get_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tolerance_);
  node_->get_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);

  node_->get_parameter(
    plugin_name_ + ".linear_speed_for_wz_scale_start",
    linear_speed_for_wz_scale_start_);
  node_->get_parameter(
    plugin_name_ + ".linear_speed_for_wz_scale_end",
    linear_speed_for_wz_scale_end_);
  node_->get_parameter(
    plugin_name_ + ".min_wz_scale_at_high_speed",
    min_wz_scale_at_high_speed_);
  node_->get_parameter(plugin_name_ + ".output_rotate_deg", output_rotate_deg_);
  node_->get_parameter(plugin_name_ + ".output_invert_x", output_invert_x_);
  node_->get_parameter(plugin_name_ + ".output_invert_y", output_invert_y_);
  node_->get_parameter(plugin_name_ + ".output_invert_wz", output_invert_wz_);
  node_->get_parameter(
    plugin_name_ + ".enable_local_path_validity_check",
    enable_local_path_validity_check_);
  node_->get_parameter(
    plugin_name_ + ".local_path_check_distance",
    local_path_check_distance_);
  node_->get_parameter(
    plugin_name_ + ".local_path_invalid_cost_threshold",
    local_path_invalid_cost_threshold_);
  node_->get_parameter(
    plugin_name_ + ".local_path_unknown_is_invalid",
    local_path_unknown_is_invalid_);
  node_->get_parameter(plugin_name_ + ".publish_lookahead_point", publish_lookahead_point_);
  node_->get_parameter(plugin_name_ + ".lookahead_topic", lookahead_topic_);

  if (publish_lookahead_point_) {
    lookahead_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(lookahead_topic_, 10);
  }

  speed_limit_scale_ = 1.0;
  last_closest_index_ = 0;
  last_cmd_time_ = node_->now();
  last_cmd_ = geometry_msgs::msg::Twist();

  RCLCPP_INFO(
    logger_,
    "Configured %s (lookahead=%.2f, max_vx=%.2f, max_vy=%.2f, max_wz=%.2f, wz_scale=[%.2f..%.2f]->%.2f)",
    plugin_name_.c_str(),
    lookahead_dist_,
    base_max_vx_,
    base_max_vy_,
    max_wz_,
    linear_speed_for_wz_scale_start_,
    linear_speed_for_wz_scale_end_,
    min_wz_scale_at_high_speed_);
}

void PurePursuitController::cleanup()
{
  global_plan_.poses.clear();
  lookahead_pub_.reset();
}

void PurePursuitController::activate()
{
  was_cancelled_ = false;
  last_cmd_time_ = node_->now();
  if (lookahead_pub_) {
    lookahead_pub_->on_activate();
  }
}

void PurePursuitController::deactivate()
{
  was_cancelled_ = true;
  if (lookahead_pub_) {
    lookahead_pub_->on_deactivate();
  }
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  last_closest_index_ = 0;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("PurePursuitController received an empty plan");
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = pose.header.frame_id;

  if (was_cancelled_) {
    cmd.twist = geometry_msgs::msg::Twist();
    return cmd;
  }

  const auto & goal_pose = global_plan_.poses.back();
  if (goal_checker != nullptr && goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity)) {
    cmd.twist = geometry_msgs::msg::Twist();
    last_cmd_ = cmd.twist;
    return cmd;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);

  geometry_msgs::msg::PoseStamped pose_in_plan = pose;
  const std::string plan_frame =
    global_plan_.header.frame_id.empty() ? pose.header.frame_id : global_plan_.header.frame_id;
  if (!plan_frame.empty() && pose.header.frame_id != plan_frame) {
    try {
      pose_in_plan = tf_->transform(pose, plan_frame);
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::ControllerTFError(
              std::string("Failed to transform pose to plan frame: ") + ex.what());
    }
  }

  const double current_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  double lookahead_distance = std::max(lookahead_dist_, current_speed * lookahead_time_);
  lookahead_distance = clamp(lookahead_distance, min_lookahead_, max_lookahead_);

  const size_t closest_index = findClosestPathIndex(pose_in_plan);
  if (enable_local_path_validity_check_ && !isPathLocallyValid(closest_index)) {
    throw nav2_core::NoValidControl("Path is invalid in local costmap");
  }
  const LookaheadPoint target = findLookaheadPoint(closest_index, lookahead_distance);

  geometry_msgs::msg::PointStamped target_point_plan;
  target_point_plan.header.stamp = cmd.header.stamp;
  target_point_plan.header.frame_id = plan_frame;
  target_point_plan.point.x = target.x;
  target_point_plan.point.y = target.y;
  target_point_plan.point.z = 0.0;

  geometry_msgs::msg::PointStamped target_point_pose = target_point_plan;
  if (!target_point_plan.header.frame_id.empty() &&
    target_point_plan.header.frame_id != pose.header.frame_id)
  {
    try {
      target_point_pose = tf_->transform(target_point_plan, pose.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::ControllerTFError(
              std::string("Failed to transform lookahead point to pose frame: ") + ex.what());
    }
  }

  if (lookahead_pub_ && lookahead_pub_->is_activated()) {
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.stamp = cmd.header.stamp;
    point_msg.header.frame_id = pose.header.frame_id;
    point_msg.point.x = target_point_pose.point.x;
    point_msg.point.y = target_point_pose.point.y;
    point_msg.point.z = 0.0;
    lookahead_pub_->publish(point_msg);
  }

  const double dx = target_point_pose.point.x - pose.pose.position.x;
  const double dy = target_point_pose.point.y - pose.pose.position.y;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double ex_robot = cos_yaw * dx + sin_yaw * dy;
  const double ey_robot = -sin_yaw * dx + cos_yaw * dy;

  geometry_msgs::msg::Twist target_cmd;

  const double max_vx = base_max_vx_ * speed_limit_scale_;
  const double max_vy = base_max_vy_ * speed_limit_scale_;

  target_cmd.linear.x = clamp(k_xy_ * ex_robot, -max_vx, max_vx);
  target_cmd.linear.y = clamp(k_xy_ * ey_robot, -max_vy, max_vy);

  const double heading_to_target = std::atan2(dy, dx);
  const double heading_error = normalizeAngle(heading_to_target - yaw);
  const double linear_cmd_mag = std::hypot(target_cmd.linear.x, target_cmd.linear.y);
  const double wz_scale = computeWzScaleFromLinearSpeed(linear_cmd_mag);
  const double wz_unclamped = k_theta_ * heading_error * wz_scale;
  target_cmd.angular.z = clamp(wz_unclamped, -max_wz_, max_wz_);

  target_cmd = applyOutputFrameTransform(target_cmd);

  target_cmd.linear.x = clamp(target_cmd.linear.x, -max_vx, max_vx);
  target_cmd.linear.y = clamp(target_cmd.linear.y, -max_vy, max_vy);
  target_cmd.angular.z = clamp(target_cmd.angular.z, -max_wz_, max_wz_);

  const rclcpp::Time now = node_->now();
  const double dt = std::max((now - last_cmd_time_).seconds(), 1e-3);
  cmd.twist = applyAccelerationLimits(target_cmd, dt);

  last_cmd_ = cmd.twist;
  last_cmd_time_ = now;
  return cmd;
}

bool PurePursuitController::cancel()
{
  was_cancelled_ = true;
  return true;
}

void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_scale_ = clamp(speed_limit / 100.0, 0.0, 1.0);
    return;
  }

  const double nominal_linear_limit = std::max(base_max_vx_, base_max_vy_);
  if (nominal_linear_limit <= 1e-6) {
    speed_limit_scale_ = 1.0;
    return;
  }

  speed_limit_scale_ = clamp(speed_limit / nominal_linear_limit, 0.0, 1.0);
}

double PurePursuitController::clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double PurePursuitController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double PurePursuitController::euclideanDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

size_t PurePursuitController::findClosestPathIndex(const geometry_msgs::msg::PoseStamped & pose) const
{
  size_t best_index = last_closest_index_;
  double best_distance = std::numeric_limits<double>::max();

  for (size_t i = last_closest_index_; i < global_plan_.poses.size(); ++i) {
    const double dist = euclideanDistance(pose, global_plan_.poses[i]);
    if (dist < best_distance) {
      best_distance = dist;
      best_index = i;
    }
  }

  last_closest_index_ = best_index;
  return best_index;
}

PurePursuitController::LookaheadPoint PurePursuitController::findLookaheadPoint(
  size_t closest_index,
  double lookahead_distance) const
{
  LookaheadPoint point;

  if (closest_index >= global_plan_.poses.size()) {
    const auto & back = global_plan_.poses.back().pose.position;
    point.x = back.x;
    point.y = back.y;
    point.heading = tf2::getYaw(global_plan_.poses.back().pose.orientation);
    return point;
  }

  double traveled = 0.0;
  for (size_t i = closest_index; i + 1 < global_plan_.poses.size(); ++i) {
    const auto & p0 = global_plan_.poses[i].pose.position;
    const auto & p1 = global_plan_.poses[i + 1].pose.position;

    const double seg_dx = p1.x - p0.x;
    const double seg_dy = p1.y - p0.y;
    const double seg_len = std::hypot(seg_dx, seg_dy);

    if (traveled + seg_len >= lookahead_distance && seg_len > 1e-6) {
      const double ratio = (lookahead_distance - traveled) / seg_len;
      point.x = p0.x + ratio * seg_dx;
      point.y = p0.y + ratio * seg_dy;
      point.heading = std::atan2(seg_dy, seg_dx);
      return point;
    }

    traveled += seg_len;
  }

  const auto & back = global_plan_.poses.back().pose.position;
  point.x = back.x;
  point.y = back.y;
  point.heading = tf2::getYaw(global_plan_.poses.back().pose.orientation);
  return point;
}

geometry_msgs::msg::Twist PurePursuitController::applyAccelerationLimits(
  const geometry_msgs::msg::Twist & target,
  double dt) const
{
  geometry_msgs::msg::Twist limited = target;

  const double max_dvx = max_ax_ * dt;
  const double max_dvy = max_ay_ * dt;
  const double max_dwz = max_aw_ * dt;

  limited.linear.x = clamp(
    target.linear.x,
    last_cmd_.linear.x - max_dvx,
    last_cmd_.linear.x + max_dvx);

  limited.linear.y = clamp(
    target.linear.y,
    last_cmd_.linear.y - max_dvy,
    last_cmd_.linear.y + max_dvy);

  limited.angular.z = clamp(
    target.angular.z,
    last_cmd_.angular.z - max_dwz,
    last_cmd_.angular.z + max_dwz);

  return limited;
}

geometry_msgs::msg::Twist PurePursuitController::applyOutputFrameTransform(
  const geometry_msgs::msg::Twist & input_cmd) const
{
  geometry_msgs::msg::Twist output = input_cmd;

  const double rad = output_rotate_deg_ * M_PI / 180.0;
  const double c = std::cos(rad);
  const double s = std::sin(rad);

  const double x_rot = c * input_cmd.linear.x - s * input_cmd.linear.y;
  const double y_rot = s * input_cmd.linear.x + c * input_cmd.linear.y;

  output.linear.x = output_invert_x_ ? -x_rot : x_rot;
  output.linear.y = output_invert_y_ ? -y_rot : y_rot;
  output.angular.z = output_invert_wz_ ? -input_cmd.angular.z : input_cmd.angular.z;

  return output;
}

double PurePursuitController::computeWzScaleFromLinearSpeed(double linear_speed) const
{
  if (linear_speed <= linear_speed_for_wz_scale_start_) {
    return 1.0;
  }

  if (linear_speed >= linear_speed_for_wz_scale_end_) {
    return clamp(min_wz_scale_at_high_speed_, 0.0, 1.0);
  }

  const double denom = std::max(
    linear_speed_for_wz_scale_end_ - linear_speed_for_wz_scale_start_,
    1e-6);
  const double alpha = (linear_speed - linear_speed_for_wz_scale_start_) / denom;
  const double scale = 1.0 - alpha * (1.0 - min_wz_scale_at_high_speed_);
  return clamp(scale, 0.0, 1.0);
}

bool PurePursuitController::isPathLocallyValid(size_t closest_index) const
{
  if (!costmap_ros_ || !tf_) {
    return true;
  }

  auto * costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return true;
  }

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  double traveled = 0.0;

  for (size_t i = closest_index; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose_in_costmap = global_plan_.poses[i];

    if (pose_in_costmap.header.frame_id != costmap_frame) {
      try {
        pose_in_costmap = tf_->transform(pose_in_costmap, costmap_frame);
      } catch (const tf2::TransformException &) {
        return false;
      }
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(pose_in_costmap.pose.position.x, pose_in_costmap.pose.position.y, mx, my)) {
      return false;
    }

    const unsigned char cost = costmap->getCost(mx, my);
    if (local_path_unknown_is_invalid_ && cost == nav2_costmap_2d::NO_INFORMATION) {
      return false;
    }
    if (static_cast<int>(cost) >= local_path_invalid_cost_threshold_) {
      return false;
    }

    if (i + 1 < global_plan_.poses.size()) {
      const auto & p0 = global_plan_.poses[i].pose.position;
      const auto & p1 = global_plan_.poses[i + 1].pose.position;
      traveled += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (traveled >= local_path_check_distance_) {
        break;
      }
    }
  }

  return true;
}

}  // namespace motion_controller

PLUGINLIB_EXPORT_CLASS(motion_controller::PurePursuitController, nav2_core::Controller)
