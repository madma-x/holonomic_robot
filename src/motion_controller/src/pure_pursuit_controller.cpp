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

namespace
{
// Minimal 2D polygon geometry for footprint-vs-costmap checks - no existing
// dependency does exactly "distance from point to polygon, signed negative
// inside" (nav2_costmap_2d::FootprintCollisionChecker only walks the
// boundary, which misses an obstacle fully enclosed by a coarse footprint
// and doesn't give the clearance-margin ring this needs).
double pointSegmentDistance(double px, double py, double ax, double ay, double bx, double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double len2 = abx * abx + aby * aby;
  double t = len2 > 1e-12 ? ((px - ax) * abx + (py - ay) * aby) / len2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  const double cx = ax + t * abx;
  const double cy = ay + t * aby;
  return std::hypot(px - cx, py - cy);
}

bool pointInPolygon(double px, double py, const std::vector<std::pair<double, double>> & poly)
{
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const double xi = poly[i].first, yi = poly[i].second;
    const double xj = poly[j].first, yj = poly[j].second;
    if (((yi > py) != (yj > py)) &&
      (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

// Unsigned distance from (px,py) to the polygon boundary; 0.0 if inside.
double distanceToPolygon(double px, double py, const std::vector<std::pair<double, double>> & poly)
{
  if (pointInPolygon(px, py, poly)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    best = std::min(best, pointSegmentDistance(px, py, poly[j].first, poly[j].second, poly[i].first, poly[i].second));
  }
  return best;
}
// Offsets a polygon outward by `d` along each edge's outward normal, used
// only for debug visualization (footprintClearance checks real distance
// against the exact polygon, not this). Robust to either winding order via
// the shoelace sign. Degenerate (near-zero-length) edges fall back to the
// unoffset vertex rather than producing NaNs.
std::vector<std::pair<double, double>> offsetPolygon(
  const std::vector<std::pair<double, double>> & poly, double d)
{
  const size_t n = poly.size();
  if (n < 3) {
    return poly;
  }
  double shoelace = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const auto & a = poly[i];
    const auto & b = poly[(i + 1) % n];
    shoelace += a.first * b.second - b.first * a.second;
  }
  const double wind_sign = shoelace >= 0.0 ? 1.0 : -1.0;

  std::vector<std::pair<double, double>> out(n);
  for (size_t i = 0; i < n; ++i) {
    const auto & prev = poly[(i + n - 1) % n];
    const auto & cur = poly[i];
    const auto & next = poly[(i + 1) % n];
    const double e1x = cur.first - prev.first, e1y = cur.second - prev.second;
    const double e2x = next.first - cur.first, e2y = next.second - cur.second;
    const double len1 = std::hypot(e1x, e1y), len2 = std::hypot(e2x, e2y);
    if (len1 < 1e-9 || len2 < 1e-9) {
      out[i] = cur;
      continue;
    }
    const double n1x = wind_sign * e1y / len1, n1y = -wind_sign * e1x / len1;
    const double n2x = wind_sign * e2y / len2, n2y = -wind_sign * e2x / len2;
    const double p1x = prev.first + n1x * d, p1y = prev.second + n1y * d;
    const double p2x = cur.first + n2x * d, p2y = cur.second + n2y * d;
    const double denom = e1x * e2y - e1y * e2x;
    if (std::abs(denom) < 1e-9) {
      out[i] = {cur.first + n1x * d, cur.second + n1y * d};
    } else {
      const double t = ((p2x - p1x) * e2y - (p2y - p1y) * e2x) / denom;
      out[i] = {p1x + t * e1x, p1y + t * e1y};
    }
  }
  return out;
}

// Mechanism 2's opponent disk only needs to be bigger than mechanism 1's own
// (much smaller) footprint-clearance margins when the opponent is actually
// closing fast enough that a static costmap snapshot would miss it - see
// opponent_speed_deadband_m_s_'s comment in the header. Ramps the disk's
// extra margins linearly from 0 (stationary - mechanism 1 alone governs) to
// the full configured values (fast-moving - mechanism 2 governs).
void opponentDiskMargins(
  double obstacle_vx, double obstacle_vy,
  double opponent_safety_margin_m, double avoidance_caution_margin_m,
  double deadband_m_s, double full_effect_m_s,
  double & safety_margin_out, double & caution_margin_out)
{
  const double speed = std::hypot(obstacle_vx, obstacle_vy);
  const double span = std::max(1e-6, full_effect_m_s - deadband_m_s);
  const double scale = std::max(0.0, std::min(1.0, (speed - deadband_m_s) / span));
  safety_margin_out = opponent_safety_margin_m * scale;
  caution_margin_out = avoidance_caution_margin_m * scale;
}
}  // namespace

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
  node_->declare_parameter(plugin_name_ + ".max_v", max_v_);
  node_->declare_parameter(plugin_name_ + ".max_wz", max_wz_);

  node_->declare_parameter(plugin_name_ + ".max_a", max_a_);
  node_->declare_parameter(plugin_name_ + ".max_ax", max_ax_);
  node_->declare_parameter(plugin_name_ + ".max_ay", max_ay_);
  node_->declare_parameter(plugin_name_ + ".max_aw", max_aw_);

  node_->declare_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tolerance_);
  node_->declare_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);

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
  node_->declare_parameter(plugin_name_ + ".publish_opponent_debug", publish_opponent_debug_);
  node_->declare_parameter(plugin_name_ + ".opponent_debug_topic", opponent_debug_topic_);
  node_->declare_parameter(plugin_name_ + ".publish_footprint_debug", publish_footprint_debug_);
  node_->declare_parameter(plugin_name_ + ".footprint_debug_topic", footprint_debug_topic_);
  node_->declare_parameter(plugin_name_ + ".local_avoidance_enabled", local_avoidance_enabled_);
  node_->declare_parameter(
    plugin_name_ + ".avoidance_prediction_horizon_s", avoidance_prediction_horizon_s_);
  node_->declare_parameter(
    plugin_name_ + ".avoidance_simulation_dt_s", avoidance_simulation_dt_s_);
  node_->declare_parameter(plugin_name_ + ".avoidance_vx_samples", avoidance_vx_samples_);
  node_->declare_parameter(plugin_name_ + ".avoidance_vy_samples", avoidance_vy_samples_);
  node_->declare_parameter(plugin_name_ + ".avoidance_wz_samples", avoidance_wz_samples_);
  node_->declare_parameter(
    plugin_name_ + ".avoidance_obstacle_cost_threshold", avoidance_obstacle_cost_threshold_);
  node_->declare_parameter(
    plugin_name_ + ".avoidance_unknown_is_obstacle", avoidance_unknown_is_obstacle_);
  node_->declare_parameter(plugin_name_ + ".avoidance_min_clearance_m", avoidance_min_clearance_m_);
  node_->declare_parameter(
    plugin_name_ + ".avoidance_caution_margin_m", avoidance_caution_margin_m_);
  node_->declare_parameter(plugin_name_ + ".avoidance_weight_goal", avoidance_weight_goal_);
  node_->declare_parameter(plugin_name_ + ".avoidance_weight_smooth", avoidance_weight_smooth_);
  node_->declare_parameter(plugin_name_ + ".avoidance_weight_obstacle", avoidance_weight_obstacle_);
  node_->declare_parameter(plugin_name_ + ".opponent_prediction_enabled", opponent_prediction_enabled_);
  node_->declare_parameter(plugin_name_ + ".opponent_tracking_radius_m", opponent_tracking_radius_m_);
  node_->declare_parameter(plugin_name_ + ".opponent_radius_m", opponent_radius_m_);
  node_->declare_parameter(plugin_name_ + ".opponent_safety_margin_m", opponent_safety_margin_m_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_speed_deadband_m_s", opponent_speed_deadband_m_s_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_speed_full_effect_m_s", opponent_speed_full_effect_m_s_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_speed_limit_enabled", opponent_speed_limit_enabled_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_speed_limit_range_m", opponent_speed_limit_range_m_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_speed_limit_min_scale", opponent_speed_limit_min_scale_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_estimate_timeout_s", opponent_estimate_timeout_s_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_velocity_deadband_m", opponent_velocity_deadband_m_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_velocity_smoothing", opponent_velocity_smoothing_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_velocity_max_m_s", opponent_velocity_max_m_s_);
  node_->declare_parameter(
    plugin_name_ + ".opponent_tracking_rate_hz", opponent_tracking_rate_hz_);

  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node_->get_parameter(plugin_name_ + ".min_lookahead", min_lookahead_);
  node_->get_parameter(plugin_name_ + ".max_lookahead", max_lookahead_);

  node_->get_parameter(plugin_name_ + ".k_xy", k_xy_);
  node_->get_parameter(plugin_name_ + ".k_theta", k_theta_);

  node_->get_parameter(plugin_name_ + ".max_vx", base_max_vx_);
  node_->get_parameter(plugin_name_ + ".max_vy", base_max_vy_);
  node_->get_parameter(plugin_name_ + ".max_v", max_v_);
  node_->get_parameter(plugin_name_ + ".max_wz", max_wz_);
  if (max_v_ <= 1e-9) {
    max_v_ = std::min(base_max_vx_, base_max_vy_);
  }

  node_->get_parameter(plugin_name_ + ".max_a", max_a_);
  node_->get_parameter(plugin_name_ + ".max_ax", max_ax_);
  node_->get_parameter(plugin_name_ + ".max_ay", max_ay_);
  if (max_a_ <= 1e-9) {
    max_a_ = std::min(max_ax_, max_ay_);
  }
  node_->get_parameter(plugin_name_ + ".max_aw", max_aw_);

  node_->get_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tolerance_);
  node_->get_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);

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
  node_->get_parameter(plugin_name_ + ".publish_opponent_debug", publish_opponent_debug_);
  node_->get_parameter(plugin_name_ + ".opponent_debug_topic", opponent_debug_topic_);
  node_->get_parameter(plugin_name_ + ".publish_footprint_debug", publish_footprint_debug_);
  node_->get_parameter(plugin_name_ + ".footprint_debug_topic", footprint_debug_topic_);
  node_->get_parameter(plugin_name_ + ".local_avoidance_enabled", local_avoidance_enabled_);
  node_->get_parameter(
    plugin_name_ + ".avoidance_prediction_horizon_s", avoidance_prediction_horizon_s_);
  node_->get_parameter(plugin_name_ + ".avoidance_simulation_dt_s", avoidance_simulation_dt_s_);
  node_->get_parameter(plugin_name_ + ".avoidance_vx_samples", avoidance_vx_samples_);
  node_->get_parameter(plugin_name_ + ".avoidance_vy_samples", avoidance_vy_samples_);
  node_->get_parameter(plugin_name_ + ".avoidance_wz_samples", avoidance_wz_samples_);
  node_->get_parameter(
    plugin_name_ + ".avoidance_obstacle_cost_threshold", avoidance_obstacle_cost_threshold_);
  node_->get_parameter(
    plugin_name_ + ".avoidance_unknown_is_obstacle", avoidance_unknown_is_obstacle_);
  node_->get_parameter(plugin_name_ + ".avoidance_min_clearance_m", avoidance_min_clearance_m_);
  node_->get_parameter(plugin_name_ + ".avoidance_caution_margin_m", avoidance_caution_margin_m_);
  node_->get_parameter(plugin_name_ + ".avoidance_weight_goal", avoidance_weight_goal_);
  node_->get_parameter(plugin_name_ + ".avoidance_weight_smooth", avoidance_weight_smooth_);
  node_->get_parameter(plugin_name_ + ".avoidance_weight_obstacle", avoidance_weight_obstacle_);
  node_->get_parameter(plugin_name_ + ".opponent_prediction_enabled", opponent_prediction_enabled_);
  node_->get_parameter(plugin_name_ + ".opponent_tracking_radius_m", opponent_tracking_radius_m_);
  node_->get_parameter(plugin_name_ + ".opponent_radius_m", opponent_radius_m_);
  node_->get_parameter(plugin_name_ + ".opponent_safety_margin_m", opponent_safety_margin_m_);
  node_->get_parameter(
    plugin_name_ + ".opponent_speed_deadband_m_s", opponent_speed_deadband_m_s_);
  node_->get_parameter(
    plugin_name_ + ".opponent_speed_full_effect_m_s", opponent_speed_full_effect_m_s_);
  node_->get_parameter(
    plugin_name_ + ".opponent_speed_limit_enabled", opponent_speed_limit_enabled_);
  node_->get_parameter(
    plugin_name_ + ".opponent_speed_limit_range_m", opponent_speed_limit_range_m_);
  node_->get_parameter(
    plugin_name_ + ".opponent_speed_limit_min_scale", opponent_speed_limit_min_scale_);
  node_->get_parameter(
    plugin_name_ + ".opponent_estimate_timeout_s", opponent_estimate_timeout_s_);
  node_->get_parameter(
    plugin_name_ + ".opponent_velocity_deadband_m", opponent_velocity_deadband_m_);
  node_->get_parameter(
    plugin_name_ + ".opponent_velocity_smoothing", opponent_velocity_smoothing_);
  node_->get_parameter(
    plugin_name_ + ".opponent_velocity_max_m_s", opponent_velocity_max_m_s_);
  node_->get_parameter(
    plugin_name_ + ".opponent_tracking_rate_hz", opponent_tracking_rate_hz_);
  has_obstacle_estimate_ = false;
  obstacle_vx_ = obstacle_vy_ = 0.0;

  if (publish_lookahead_point_) {
    lookahead_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(lookahead_topic_, 10);
  }
  if (publish_opponent_debug_) {
    opponent_debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      opponent_debug_topic_, 10);
  }
  if (publish_footprint_debug_) {
    footprint_debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      footprint_debug_topic_, 10);
  }

  speed_limit_scale_ = 1.0;
  last_closest_index_ = 0;
  last_cmd_time_ = node_->now();
  last_cmd_ = geometry_msgs::msg::Twist();
  last_vx_world_ = last_vy_world_ = last_wz_ = 0.0;

  RCLCPP_INFO(
    logger_,
    "Configured %s (lookahead=%.2f, max_v=%.2f [vx %.2f vy %.2f], max_wz=%.2f, "
    "goal_tol=%.3fm/%.1fdeg)",
    plugin_name_.c_str(),
    lookahead_dist_,
    max_v_,
    base_max_vx_,
    base_max_vy_,
    max_wz_,
    goal_dist_tolerance_,
    goal_yaw_tolerance_ * 180.0 / M_PI);

  on_set_parameters_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&PurePursuitController::onParameterUpdate, this, std::placeholders::_1));
}

void PurePursuitController::cleanup()
{
  global_plan_.poses.clear();
  lookahead_pub_.reset();
  opponent_debug_pub_.reset();
  footprint_debug_pub_.reset();
  if (obstacle_tracking_timer_) {
    obstacle_tracking_timer_->cancel();
    obstacle_tracking_timer_.reset();
  }
  on_set_parameters_handle_.reset();
}

void PurePursuitController::activate()
{
  was_cancelled_ = false;
  last_cmd_time_ = node_->now();
  last_vx_world_ = last_vy_world_ = last_wz_ = 0.0;
  {
    std::lock_guard<std::mutex> lk(obstacle_mutex_);
    has_obstacle_estimate_ = false;
    obstacle_vx_ = obstacle_vy_ = 0.0;
  }
  if (lookahead_pub_) {
    lookahead_pub_->on_activate();
  }
  if (opponent_debug_pub_) {
    opponent_debug_pub_->on_activate();
  }
  if (footprint_debug_pub_) {
    footprint_debug_pub_->on_activate();
  }
  if (opponent_tracking_rate_hz_ > 1e-6) {
    const auto period = std::chrono::duration<double>(1.0 / opponent_tracking_rate_hz_);
    obstacle_tracking_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PurePursuitController::obstacleTrackingTimerCallback, this));
  }
}

void PurePursuitController::deactivate()
{
  was_cancelled_ = true;
  if (lookahead_pub_) {
    lookahead_pub_->on_deactivate();
  }
  if (opponent_debug_pub_) {
    opponent_debug_pub_->on_deactivate();
  }
  if (footprint_debug_pub_) {
    footprint_debug_pub_->on_deactivate();
  }
  if (obstacle_tracking_timer_) {
    obstacle_tracking_timer_->cancel();
    obstacle_tracking_timer_.reset();
  }
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  last_closest_index_ = 0;
  // A new plan means a new goal to drive, even if a previous one was
  // cancelled - without this, was_cancelled_ stays latched true from the
  // first-ever cancel() for the rest of this controller's lifetime (it's
  // otherwise only cleared in activate(), a one-time lifecycle transition),
  // silently zeroing computeVelocityCommands() forever after any goal
  // preemption, sim or real.
  was_cancelled_ = false;
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
    last_vx_world_ = last_vy_world_ = last_wz_ = 0.0;
    return cmd;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
  const double dist_to_goal = std::hypot(
    goal_pose.pose.position.x - pose.pose.position.x,
    goal_pose.pose.position.y - pose.pose.position.y);

  // Terminal yaw stage: position is close enough, only heading is left.
  // Nav2's own goal_checker (configured separately, see nav2_params_*.yaml)
  // is what actually keeps computeVelocityCommands being called until this
  // is also satisfied - its yaw tolerance must be <= goal_yaw_tolerance_ or
  // the BT considers the goal reached before this stage ever runs. Holds
  // position and rotates only, at the same k_theta_ gain and max_wz_ cap as
  // normal driving - never a sudden, separately-tuned spin.
  if (dist_to_goal <= goal_dist_tolerance_ &&
    std::fabs(normalizeAngle(goal_yaw - yaw)) > goal_yaw_tolerance_)
  {
    const double heading_error = normalizeAngle(goal_yaw - yaw);
    const double target_wz = clamp(k_theta_ * heading_error, -max_wz_, max_wz_);
    const rclcpp::Time now = node_->now();
    const double dt = std::max((now - last_cmd_time_).seconds(), 1e-3);
    double safe_vx = 0.0, safe_vy = 0.0, safe_wz = target_wz;
    selectSafeVelocity(
      0.0, 0.0, target_wz, pose.pose.position.x, pose.pose.position.y, yaw, dt,
      safe_vx, safe_vy, safe_wz);
    cmd.twist = applyAccelerationLimits(safe_vx, safe_vy, safe_wz, yaw, dt);
    cmd.twist = applyOutputFrameTransform(cmd.twist);
    last_cmd_ = cmd.twist;
    last_cmd_time_ = now;
    return cmd;
  }

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
  // local_avoidance_enabled_ supersedes this: a per-cycle candidate
  // re-selection already stops/detours gracefully, so the old all-or-
  // nothing "throw and let Nav2's recovery BT take over" check (which a
  // live test showed collides against a fast-closing obstacle, since
  // the abort->patience->recovery latency outlasts the closing distance)
  // would otherwise fight it for no benefit.
  if (!local_avoidance_enabled_ && enable_local_path_validity_check_ &&
    !isPathLocallyValid(closest_index))
  {
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

  // World-frame position P-controller: for a holonomic base, translation
  // never needs heading at all, so compute and clamp it directly in world
  // frame instead of rotating into body frame and back (the old body-frame
  // route also clamped x/y separately - a SQUARE limit, not a disk, so the
  // achievable speed toward a fixed world direction swung by up to sqrt(2)
  // as the robot turned, and clipping one axis but not the other bent the
  // commanded direction away from the lookahead bearing).
  double target_vx_world = k_xy_ * dx;
  double target_vy_world = k_xy_ * dy;
  const double max_v = max_v_ * speed_limit_scale_;
  const double v_mag = std::hypot(target_vx_world, target_vy_world);
  if (v_mag > max_v && v_mag > 1e-9) {
    target_vx_world *= max_v / v_mag;
    target_vy_world *= max_v / v_mag;
  }

  // Holonomic heading control: rotate toward the final goal orientation
  // throughout translation, independent of the lookahead point and travel
  // direction (translation above is already computed purely in world
  // frame). Rotation depends only on heading error - never on translation
  // speed (the old computeWzScaleFromLinearSpeed de-rated turn gain as
  // commanded speed rose; with this robot's real k_theta/wz-scale config
  // that cut turn authority to ~3% above 0.3 m/s, which is why it could
  // only meaningfully rotate once stopped).
  const double heading_error = normalizeAngle(goal_yaw - yaw);
  const double target_wz = clamp(k_theta_ * heading_error, -max_wz_, max_wz_);

  const rclcpp::Time now = node_->now();
  const double dt = std::max((now - last_cmd_time_).seconds(), 1e-3);
  double safe_vx_world = target_vx_world, safe_vy_world = target_vy_world, safe_wz = target_wz;
  selectSafeVelocity(
    target_vx_world, target_vy_world, target_wz,
    pose.pose.position.x, pose.pose.position.y, yaw, dt,
    safe_vx_world, safe_vy_world, safe_wz);
  cmd.twist = applyAccelerationLimits(safe_vx_world, safe_vy_world, safe_wz, yaw, dt);
  cmd.twist = applyOutputFrameTransform(cmd.twist);

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

rcl_interfaces::msg::SetParametersResult PurePursuitController::onParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  const std::string prefix = plugin_name_ + ".";
  for (const auto & p : parameters) {
    const std::string & name = p.get_name();
    if (name.rfind(prefix, 0) != 0) {
      continue;  // not one of ours
    }
    const std::string key = name.substr(prefix.size());

    if (key == "lookahead_dist") {lookahead_dist_ = p.as_double();}
    else if (key == "lookahead_time") {lookahead_time_ = p.as_double();}
    else if (key == "min_lookahead") {min_lookahead_ = p.as_double();}
    else if (key == "max_lookahead") {max_lookahead_ = p.as_double();}
    else if (key == "k_xy") {k_xy_ = p.as_double();}
    else if (key == "k_theta") {k_theta_ = p.as_double();}
    else if (key == "max_vx") {base_max_vx_ = p.as_double();}
    else if (key == "max_vy") {base_max_vy_ = p.as_double();}
    else if (key == "max_v") {max_v_ = p.as_double();}
    else if (key == "max_wz") {max_wz_ = p.as_double();}
    else if (key == "max_a") {max_a_ = p.as_double();}
    else if (key == "max_ax") {max_ax_ = p.as_double();}
    else if (key == "max_ay") {max_ay_ = p.as_double();}
    else if (key == "max_aw") {max_aw_ = p.as_double();}
    else if (key == "goal_dist_tolerance") {goal_dist_tolerance_ = p.as_double();}
    else if (key == "goal_yaw_tolerance") {goal_yaw_tolerance_ = p.as_double();}
    else if (key == "output_rotate_deg") {output_rotate_deg_ = p.as_double();}
    else if (key == "output_invert_x") {output_invert_x_ = p.as_bool();}
    else if (key == "output_invert_y") {output_invert_y_ = p.as_bool();}
    else if (key == "output_invert_wz") {output_invert_wz_ = p.as_bool();}
    else if (key == "enable_local_path_validity_check") {enable_local_path_validity_check_ = p.as_bool();}
    else if (key == "local_path_check_distance") {local_path_check_distance_ = p.as_double();}
    else if (key == "local_path_invalid_cost_threshold") {
      local_path_invalid_cost_threshold_ = static_cast<int>(p.as_int());
    } else if (key == "local_path_unknown_is_invalid") {local_path_unknown_is_invalid_ = p.as_bool();}
    else if (key == "local_avoidance_enabled") {local_avoidance_enabled_ = p.as_bool();}
    else if (key == "avoidance_prediction_horizon_s") {avoidance_prediction_horizon_s_ = p.as_double();}
    else if (key == "avoidance_simulation_dt_s") {avoidance_simulation_dt_s_ = p.as_double();}
    else if (key == "avoidance_vx_samples") {avoidance_vx_samples_ = static_cast<int>(p.as_int());}
    else if (key == "avoidance_vy_samples") {avoidance_vy_samples_ = static_cast<int>(p.as_int());}
    else if (key == "avoidance_wz_samples") {avoidance_wz_samples_ = static_cast<int>(p.as_int());}
    else if (key == "avoidance_obstacle_cost_threshold") {
      avoidance_obstacle_cost_threshold_ = static_cast<int>(p.as_int());
    } else if (key == "avoidance_unknown_is_obstacle") {avoidance_unknown_is_obstacle_ = p.as_bool();}
    else if (key == "avoidance_min_clearance_m") {avoidance_min_clearance_m_ = p.as_double();}
    else if (key == "avoidance_caution_margin_m") {avoidance_caution_margin_m_ = p.as_double();}
    else if (key == "avoidance_weight_goal") {avoidance_weight_goal_ = p.as_double();}
    else if (key == "avoidance_weight_smooth") {avoidance_weight_smooth_ = p.as_double();}
    else if (key == "avoidance_weight_obstacle") {avoidance_weight_obstacle_ = p.as_double();}
    else if (key == "opponent_prediction_enabled") {opponent_prediction_enabled_ = p.as_bool();}
    else if (key == "opponent_tracking_radius_m") {opponent_tracking_radius_m_ = p.as_double();}
    else if (key == "opponent_radius_m") {opponent_radius_m_ = p.as_double();}
    else if (key == "opponent_safety_margin_m") {opponent_safety_margin_m_ = p.as_double();}
    else if (key == "opponent_speed_deadband_m_s") {opponent_speed_deadband_m_s_ = p.as_double();}
    else if (key == "opponent_speed_full_effect_m_s") {
      opponent_speed_full_effect_m_s_ = p.as_double();
    }
    else if (key == "opponent_speed_limit_enabled") {opponent_speed_limit_enabled_ = p.as_bool();}
    else if (key == "opponent_speed_limit_range_m") {opponent_speed_limit_range_m_ = p.as_double();}
    else if (key == "opponent_speed_limit_min_scale") {
      opponent_speed_limit_min_scale_ = p.as_double();
    }
    else if (key == "opponent_estimate_timeout_s") {opponent_estimate_timeout_s_ = p.as_double();}
    else if (key == "opponent_velocity_deadband_m") {opponent_velocity_deadband_m_ = p.as_double();}
    else if (key == "opponent_velocity_smoothing") {opponent_velocity_smoothing_ = p.as_double();}
    else if (key == "opponent_velocity_max_m_s") {opponent_velocity_max_m_s_ = p.as_double();}
    // opponent_tracking_rate_hz intentionally not live-reconfigurable - it
    // only takes effect by recreating the timer, done in activate().
    // lookahead_topic/publish_lookahead_point intentionally not live-
    // reconfigurable: changing them would mean creating/destroying the
    // publisher outside configure(), not worth the complexity here.
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
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
  double target_vx_world, double target_vy_world, double target_wz,
  double yaw, double dt)
{
  // Isotropic too, same reasoning as the speed clamp above: ramping x/y
  // separately in the OLD body frame meant a previous tick's command
  // (captured at a different yaw, since the robot was rotating) fought the
  // current rotation every tick - the faster it turned, the more this
  // throttled translation. Ramping the magnitude in world frame removes the
  // stale-yaw dependency entirely.
  double dvx = target_vx_world - last_vx_world_;
  double dvy = target_vy_world - last_vy_world_;
  const double max_dv = max_a_ * dt;
  const double dv_mag = std::hypot(dvx, dvy);
  if (dv_mag > max_dv && dv_mag > 1e-9) {
    dvx *= max_dv / dv_mag;
    dvy *= max_dv / dv_mag;
  }
  last_vx_world_ += dvx;
  last_vy_world_ += dvy;

  const double max_dwz = max_aw_ * dt;
  last_wz_ = clamp(target_wz, last_wz_ - max_dwz, last_wz_ + max_dwz);

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  geometry_msgs::msg::Twist out;
  out.linear.x = c * last_vx_world_ + s * last_vy_world_;
  out.linear.y = -s * last_vx_world_ + c * last_vy_world_;
  out.angular.z = last_wz_;
  return out;
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

void PurePursuitController::footprintClearance(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  nav2_costmap_2d::Costmap2D * costmap,
  double hard_margin, double caution_margin,
  bool & collision, bool & caution) const
{
  collision = false;
  caution = false;

  const double c = std::cos(theta);
  const double s = std::sin(theta);
  std::vector<std::pair<double, double>> poly;
  poly.reserve(footprint.size());
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & p : footprint) {
    const double wx = x + c * p.x - s * p.y;
    const double wy = y + s * p.x + c * p.y;
    poly.emplace_back(wx, wy);
    min_x = std::min(min_x, wx);
    max_x = std::max(max_x, wx);
    min_y = std::min(min_y, wy);
    max_y = std::max(max_y, wy);
  }
  if (poly.size() < 3) {
    return;
  }

  const double res = costmap->getResolution();
  const double outer = hard_margin + caution_margin;
  min_x -= outer;
  max_x += outer;
  min_y -= outer;
  max_y += outer;

  for (double gy = min_y; gy <= max_y; gy += res) {
    for (double gx = min_x; gx <= max_x; gx += res) {
      const double d = distanceToPolygon(gx, gy, poly);
      if (d > outer) {
        continue;
      }
      unsigned int mx = 0, my = 0;
      if (!costmap->worldToMap(gx, gy, mx, my)) {
        continue;  // outside the local costmap window - nothing to check against here
      }
      const unsigned char cost = costmap->getCost(mx, my);
      bool is_obstacle;
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        is_obstacle = avoidance_unknown_is_obstacle_;
      } else {
        is_obstacle = static_cast<int>(cost) >= avoidance_obstacle_cost_threshold_;
      }
      if (!is_obstacle) {
        continue;
      }
      if (d <= hard_margin) {
        collision = true;
        caution = true;
        return;  // can't learn anything finer than "collides" from more cells
      }
      caution = true;
    }
  }
}

void PurePursuitController::updateObstacleEstimate(
  double x, double y, nav2_costmap_2d::Costmap2D * costmap)
{
  const double res = costmap->getResolution();
  double sum_x = 0.0, sum_y = 0.0;
  int count = 0;
  for (double gy = y - opponent_tracking_radius_m_; gy <= y + opponent_tracking_radius_m_; gy += res) {
    for (double gx = x - opponent_tracking_radius_m_; gx <= x + opponent_tracking_radius_m_; gx += res) {
      if (std::hypot(gx - x, gy - y) > opponent_tracking_radius_m_) {
        continue;
      }
      unsigned int mx = 0, my = 0;
      if (!costmap->worldToMap(gx, gy, mx, my)) {
        continue;
      }
      if (costmap->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        sum_x += gx;
        sum_y += gy;
        ++count;
      }
    }
  }

  const rclcpp::Time now = node_->now();
  std::lock_guard<std::mutex> lk(obstacle_mutex_);
  if (count == 0) {
    // No sighting this tick: keep the last known position (so a momentarily
    // occluded obstacle doesn't just vanish from the prediction) but let
    // the timeout below zero the velocity and eventually the whole estimate
    // if it stays unseen.
  } else {
    const double ox = sum_x / count;
    const double oy = sum_y / count;
    if (has_obstacle_estimate_) {
      const double est_dt = (now - obstacle_stamp_).seconds();
      if (est_dt > 1e-3 && est_dt < opponent_estimate_timeout_s_) {
        const double dx = ox - obstacle_x_, dy = oy - obstacle_y_;
        // Below the deadband: almost certainly rolling-window rasterization
        // jitter on a stationary object, not real motion (see member
        // comment) - treat as "didn't move" rather than feeding a spurious
        // ~(deadband/dt) m/s spike into the raw estimate below.
        double raw_vx = std::hypot(dx, dy) > opponent_velocity_deadband_m_ ? dx / est_dt : 0.0;
        double raw_vy = std::hypot(dx, dy) > opponent_velocity_deadband_m_ ? dy / est_dt : 0.0;
        const double raw_speed = std::hypot(raw_vx, raw_vy);
        if (raw_speed > opponent_velocity_max_m_s_ && raw_speed > 1e-9) {
          raw_vx *= opponent_velocity_max_m_s_ / raw_speed;
          raw_vy *= opponent_velocity_max_m_s_ / raw_speed;
        }
        const double a = clamp(opponent_velocity_smoothing_, 0.0, 1.0);
        obstacle_vx_ = a * raw_vx + (1.0 - a) * obstacle_vx_;
        obstacle_vy_ = a * raw_vy + (1.0 - a) * obstacle_vy_;
        // Same filter applied to the tracked POSITION, not just velocity -
        // the raw per-tick centroid (rolling-window rasterization jitter,
        // see opponent_velocity_deadband_m_'s comment above) was being
        // stored verbatim below and used as-is for the disk's own center
        // and the predicted point, so smoothing only the velocity still
        // left the prediction visibly jerky on screen - confirmed live.
        obstacle_x_ = a * ox + (1.0 - a) * obstacle_x_;
        obstacle_y_ = a * oy + (1.0 - a) * obstacle_y_;
      } else {
        obstacle_vx_ = obstacle_vy_ = 0.0;
        obstacle_x_ = ox;
        obstacle_y_ = oy;
      }
    } else {
      obstacle_vx_ = obstacle_vy_ = 0.0;
      obstacle_x_ = ox;
      obstacle_y_ = oy;
    }
    obstacle_stamp_ = now;
    has_obstacle_estimate_ = true;
  }

  if (has_obstacle_estimate_ && (now - obstacle_stamp_).seconds() > opponent_estimate_timeout_s_) {
    has_obstacle_estimate_ = false;
    obstacle_vx_ = obstacle_vy_ = 0.0;
  }
}

void PurePursuitController::publishOpponentDebug(double robot_circumradius)
{
  if (!opponent_debug_pub_ || !opponent_debug_pub_->is_activated()) {
    return;
  }
  bool snap_has_estimate;
  double snap_x, snap_y, snap_vx, snap_vy;
  {
    std::lock_guard<std::mutex> lk(obstacle_mutex_);
    snap_has_estimate = has_obstacle_estimate_;
    snap_x = obstacle_x_; snap_y = obstacle_y_;
    snap_vx = obstacle_vx_; snap_vy = obstacle_vy_;
  }

  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  if (opponent_prediction_enabled_ && snap_has_estimate && costmap_ros_) {
    const std::string frame = costmap_ros_->getGlobalFrameID();
    const rclcpp::Time stamp = node_->now();
    double scaled_safety_margin, scaled_caution_margin;
    opponentDiskMargins(
      snap_vx, snap_vy, opponent_safety_margin_m_, avoidance_caution_margin_m_,
      opponent_speed_deadband_m_s_, opponent_speed_full_effect_m_s_,
      scaled_safety_margin, scaled_caution_margin);
    const double stop_at = opponent_radius_m_ + scaled_safety_margin + robot_circumradius;
    const double pred_x = snap_x + snap_vx * avoidance_prediction_horizon_s_;
    const double pred_y = snap_y + snap_vy * avoidance_prediction_horizon_s_;

    auto disk = [&](int id, double x, double y, double radius, float r, float g, float b, float a) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = stamp;
      m.ns = "opponent_prediction";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = 0.02;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = 2.0 * radius;
      m.scale.z = 0.01;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
      arr.markers.push_back(m);
    };
    // Hard-stop circle (candidates rejected inside this) and the looser
    // caution ring (scored worse, not rejected) - both centered on the
    // TRACKED (current) position, so what's drawn matches what this tick's
    // candidates were actually checked against.
    disk(0, snap_x, snap_y, stop_at, 0.85f, 0.1f, 0.1f, 0.35f);
    disk(1, snap_x, snap_y, stop_at + scaled_caution_margin, 0.9f, 0.6f, 0.1f, 0.2f);
    // Same two disks projected out to the predicted position (where the
    // yellow sphere/white arrow point) - the avoidance zone moves WITH the
    // prediction over the horizon (selectSafeVelocity checks candidates
    // against the predicted position at every future step, not just
    // "now"), so the zone that's actually relevant by the time a slower
    // candidate gets there is this one, not the one sitting on the
    // opponent's current spot. Dimmer, to read as "projected" vs "now".
    disk(4, pred_x, pred_y, stop_at, 0.85f, 0.1f, 0.1f, 0.15f);
    disk(5, pred_x, pred_y, stop_at + scaled_caution_margin, 0.9f, 0.6f, 0.1f, 0.1f);

    visualization_msgs::msg::Marker predicted;
    predicted.header.frame_id = frame;
    predicted.header.stamp = stamp;
    predicted.ns = "opponent_prediction";
    predicted.id = 2;
    predicted.type = visualization_msgs::msg::Marker::SPHERE;
    predicted.action = visualization_msgs::msg::Marker::ADD;
    predicted.pose.position.x = pred_x;
    predicted.pose.position.y = pred_y;
    predicted.pose.position.z = 0.05;
    predicted.pose.orientation.w = 1.0;
    predicted.scale.x = predicted.scale.y = predicted.scale.z = 0.12;
    predicted.color.r = 1.0f; predicted.color.g = 1.0f; predicted.color.b = 0.0f; predicted.color.a = 0.9f;
    arr.markers.push_back(predicted);

    visualization_msgs::msg::Marker velocity;
    velocity.header.frame_id = frame;
    velocity.header.stamp = stamp;
    velocity.ns = "opponent_prediction";
    velocity.id = 3;
    velocity.type = visualization_msgs::msg::Marker::ARROW;
    velocity.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start, end;
    start.x = snap_x; start.y = snap_y; start.z = 0.05;
    end.x = pred_x; end.y = pred_y; end.z = 0.05;
    velocity.points = {start, end};
    velocity.scale.x = 0.03;  // shaft diameter
    velocity.scale.y = 0.06;  // head diameter
    velocity.color.r = 1.0f; velocity.color.g = 1.0f; velocity.color.b = 1.0f; velocity.color.a = 0.9f;
    arr.markers.push_back(velocity);
  }

  opponent_debug_pub_->publish(arr);
}

void PurePursuitController::publishFootprintDebug(
  double x, double y, double theta,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (!footprint_debug_pub_ || !footprint_debug_pub_->is_activated() || !costmap_ros_) {
    return;
  }
  if (footprint.size() < 3) {
    return;
  }

  const double c = std::cos(theta), s = std::sin(theta);
  std::vector<std::pair<double, double>> world_fp;
  world_fp.reserve(footprint.size());
  for (const auto & p : footprint) {
    world_fp.emplace_back(x + c * p.x - s * p.y, y + s * p.x + c * p.y);
  }

  const std::string frame = costmap_ros_->getGlobalFrameID();
  const rclcpp::Time stamp = node_->now();

  auto ring = [&](int id, const std::vector<std::pair<double, double>> & poly,
    float r, float g, float b, float a)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = "avoidance_footprint";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.02;  // line width
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    for (const auto & p : poly) {
      geometry_msgs::msg::Point pt;
      pt.x = p.first; pt.y = p.second; pt.z = 0.02;
      m.points.push_back(pt);
    }
    if (!poly.empty()) {
      geometry_msgs::msg::Point pt;
      pt.x = poly.front().first; pt.y = poly.front().second; pt.z = 0.02;
      m.points.push_back(pt);  // close the loop
    }
    return m;
  };

  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  // Raw footprint (white), hard-reject ring (red, avoidance_min_clearance_m),
  // caution ring (orange, + avoidance_caution_margin_m) - same two distances
  // footprintClearance checks against the costmap, drawn around the actual
  // footprint shape rather than local_costmap's inflation gradient, so
  // mechanism 1's real extent is visible next to mechanism 2's disks
  // (/debug/opponent_prediction).
  arr.markers.push_back(ring(0, world_fp, 1.0f, 1.0f, 1.0f, 0.6f));
  arr.markers.push_back(
    ring(1, offsetPolygon(world_fp, avoidance_min_clearance_m_), 0.85f, 0.1f, 0.1f, 0.6f));
  arr.markers.push_back(
    ring(
      2, offsetPolygon(world_fp, avoidance_min_clearance_m_ + avoidance_caution_margin_m_),
      0.9f, 0.6f, 0.1f, 0.5f));

  // Mechanism 1's look-ahead: selectSafeVelocity actually simulates a whole
  // GRID of candidate (vx,vy,wz) forward over avoidance_prediction_horizon_s_
  // (see its loop), not one single trajectory, so there's no one "the"
  // future footprint to draw. This projects along the robot's own last
  // ramped command (last_vx_world_/last_vy_world_/last_wz_, same velocity
  // applyAccelerationLimits is about to output) instead - a representative
  // look-ahead silhouette, dimmer to read as "projected" (matches
  // publishOpponentDebug's dimmer pred_x/pred_y disks at ids 4/5), so
  // mechanism 1's horizon is visible at a glance next to mechanism 2's.
  const double t = avoidance_prediction_horizon_s_;
  const double theta2 = theta + last_wz_ * t;
  const double x2 = x + last_vx_world_ * t;
  const double y2 = y + last_vy_world_ * t;
  const double c2 = std::cos(theta2), s2 = std::sin(theta2);
  std::vector<std::pair<double, double>> world_fp_pred;
  world_fp_pred.reserve(footprint.size());
  for (const auto & p : footprint) {
    world_fp_pred.emplace_back(x2 + c2 * p.x - s2 * p.y, y2 + s2 * p.x + c2 * p.y);
  }
  arr.markers.push_back(ring(3, world_fp_pred, 1.0f, 1.0f, 1.0f, 0.25f));
  arr.markers.push_back(
    ring(4, offsetPolygon(world_fp_pred, avoidance_min_clearance_m_), 0.85f, 0.1f, 0.1f, 0.25f));
  arr.markers.push_back(
    ring(
      5, offsetPolygon(world_fp_pred, avoidance_min_clearance_m_ + avoidance_caution_margin_m_),
      0.9f, 0.6f, 0.1f, 0.2f));

  footprint_debug_pub_->publish(arr);
}

void PurePursuitController::obstacleTrackingTimerCallback()
{
  if (!local_avoidance_enabled_ || !costmap_ros_ || !tf_) {
    return;
  }
  auto * costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return;
  }

  geometry_msgs::msg::TransformStamped robot_tf;
  try {
    robot_tf = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), costmap_ros_->getBaseFrameID(), tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }
  const double x = robot_tf.transform.translation.x;
  const double y = robot_tf.transform.translation.y;
  const double theta = tf2::getYaw(robot_tf.transform.rotation);

  updateObstacleEstimate(x, y, costmap);

  // Unpadded, not getRobotFootprint() - that one returns Nav2's own
  // footprint_padding_-grown polygon, a bigger triangle than the game
  // marker (sim_node's _robot_prism, same model.local_poly as the bare
  // footprint sim_node publishes) - confirmed as the mismatch Max saw live
  // between the robot's RViz triangle and the debug footprint ring. Using
  // the true (unpadded) shape here makes avoidance_min_clearance_m_/
  // avoidance_caution_margin_m_ the only margins mechanism 1 applies on
  // top of the real robot outline, instead of silently stacking on Nav2's
  // own hidden padding too.
  const auto footprint = costmap_ros_->getUnpaddedRobotFootprint();
  if (footprint.size() < 3) {
    return;
  }
  double robot_circumradius = 0.0;
  for (const auto & p : footprint) {
    robot_circumradius = std::max(robot_circumradius, std::hypot(p.x, p.y));
  }
  publishOpponentDebug(robot_circumradius);
  publishFootprintDebug(x, y, theta, footprint);
}

bool PurePursuitController::selectSafeVelocity(
  double desired_vx_world, double desired_vy_world, double desired_wz,
  double x, double y, double yaw, double dt,
  double & safe_vx_world, double & safe_vy_world, double & safe_wz)
{
  safe_vx_world = desired_vx_world;
  safe_vy_world = desired_vy_world;
  safe_wz = desired_wz;

  if (!local_avoidance_enabled_ || !costmap_ros_) {
    return true;
  }
  auto * costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return true;
  }
  // Unpadded - see the obstacleTrackingTimerCallback's comment above.
  const auto footprint = costmap_ros_->getUnpaddedRobotFootprint();
  if (footprint.size() < 3) {
    return true;
  }

  updateObstacleEstimate(x, y, costmap);
  bool snap_has_obstacle_estimate;
  double snap_obstacle_x, snap_obstacle_y, snap_obstacle_vx, snap_obstacle_vy;
  {
    std::lock_guard<std::mutex> lk(obstacle_mutex_);
    snap_has_obstacle_estimate = has_obstacle_estimate_;
    snap_obstacle_x = obstacle_x_; snap_obstacle_y = obstacle_y_;
    snap_obstacle_vx = obstacle_vx_; snap_obstacle_vy = obstacle_vy_;
  }

  // Conservative circumradius of the (possibly box-laden) footprint, used
  // below as a cheap stand-in for a full footprint-vs-predicted-disk check.
  double robot_circumradius = 0.0;
  for (const auto & p : footprint) {
    robot_circumradius = std::max(robot_circumradius, std::hypot(p.x, p.y));
  }
  publishOpponentDebug(robot_circumradius);
  publishFootprintDebug(x, y, yaw, footprint);

  // Reachable window this tick (same limits applyAccelerationLimits enforces
  // afterwards) - candidates are generated inside it so that clamp is a
  // no-op, and the explicit max-braking candidate below is always feasible.
  const double max_dv = max_a_ * dt;
  const double max_dw = max_aw_ * dt;
  // Simple adaptive speed limit (see opponent_speed_limit_enabled_'s header
  // comment): caps the ceiling every candidate below is sampled/clamped
  // against, based on distance to the opponent's CURRENT position - unlike
  // mechanism 2's disk, this reacts to proximity alone, not opponent speed,
  // so it stays in effect against a stationary opponent too (more reaction
  // room for both mechanisms as we close in, regardless of who's closing).
  double proximity_speed_scale = 1.0;
  if (opponent_speed_limit_enabled_ && snap_has_obstacle_estimate) {
    const double dist = std::hypot(snap_obstacle_x - x, snap_obstacle_y - y);
    const double contact = robot_circumradius + opponent_radius_m_;
    const double t = clamp(
      (dist - contact) / std::max(1e-6, opponent_speed_limit_range_m_), 0.0, 1.0);
    proximity_speed_scale = opponent_speed_limit_min_scale_ +
      (1.0 - opponent_speed_limit_min_scale_) * t;
  }
  const double max_v = max_v_ * speed_limit_scale_ * proximity_speed_scale;

  auto sample_around = [](double center, double spread, int count) {
    std::vector<double> out;
    if (count <= 1 || spread <= 1e-9) {
      out.push_back(center);
      return out;
    }
    for (int i = 0; i < count; ++i) {
      const double t = static_cast<double>(i) / (count - 1) * 2.0 - 1.0;
      out.push_back(center + t * spread);
    }
    return out;
  };

  const auto vx_opts = sample_around(desired_vx_world, max_dv, avoidance_vx_samples_);
  const auto vy_opts = sample_around(desired_vy_world, max_dv, avoidance_vy_samples_);
  const auto wz_opts = sample_around(desired_wz, max_dw, avoidance_wz_samples_);

  struct Candidate { double vx; double vy; double wz; };
  std::vector<Candidate> candidates;
  candidates.reserve(vx_opts.size() * vy_opts.size() * wz_opts.size() + 2);
  for (double vx : vx_opts) {
    for (double vy : vy_opts) {
      for (double wz : wz_opts) {
        candidates.push_back({vx, vy, wz});
      }
    }
  }
  // Explicit max-braking candidate: the planner must always be able to
  // choose "decelerate at the configured limit" even if that point isn't
  // hit by the regular grid (e.g. an even sample count straddling it).
  const double current_speed = std::hypot(last_vx_world_, last_vy_world_);
  double brake_vx = 0.0, brake_vy = 0.0;
  if (current_speed > 1e-6) {
    const double scale = std::max(0.0, current_speed - max_dv) / current_speed;
    brake_vx = last_vx_world_ * scale;
    brake_vy = last_vy_world_ * scale;
  }
  candidates.push_back({brake_vx, brake_vy, 0.0});

  const int steps = std::max(
    1, static_cast<int>(std::round(avoidance_prediction_horizon_s_ / avoidance_simulation_dt_s_)));

  // Constant for this tick (depends only on the opponent's current tracked
  // velocity, not on any candidate) - see opponentDiskMargins' comment.
  double scaled_opponent_safety_margin = 0.0, scaled_opponent_caution_margin = 0.0;
  opponentDiskMargins(
    snap_obstacle_vx, snap_obstacle_vy, opponent_safety_margin_m_, avoidance_caution_margin_m_,
    opponent_speed_deadband_m_s_, opponent_speed_full_effect_m_s_,
    scaled_opponent_safety_margin, scaled_opponent_caution_margin);

  double best_cost = std::numeric_limits<double>::infinity();
  bool found = false;

  for (const auto & raw : candidates) {
    double vx = clamp(raw.vx, last_vx_world_ - max_dv, last_vx_world_ + max_dv);
    double vy = clamp(raw.vy, last_vy_world_ - max_dv, last_vy_world_ + max_dv);
    double wz = clamp(raw.wz, last_wz_ - max_dw, last_wz_ + max_dw);
    wz = clamp(wz, -max_wz_, max_wz_);
    const double speed = std::hypot(vx, vy);
    if (speed > max_v && speed > 1e-9) {
      vx *= max_v / speed;
      vy *= max_v / speed;
    }

    bool collision = false;
    bool caution = false;
    // Continuous, not binary: a flat "touched the caution ring or not" flag
    // scores a full stop identically to a sideways dodge as long as both
    // clear the hard threshold, so the optimizer never has a reason to
    // prefer the dodge - confirmed by a live test where the robot
    // correctly braked to a stop directly on the opponent's closing line
    // and then just sat there (safe in isolation) instead of stepping
    // aside, and a blind, non-avoiding opponent then drove into it. This
    // margin shrinks toward 0 as the predicted opponent gets closer to the
    // caution boundary, so a candidate that actually opens more distance
    // scores strictly better than one that merely holds position.
    double min_opponent_margin = std::numeric_limits<double>::infinity();
    for (int step = 1; step <= steps; ++step) {
      const double t = step * avoidance_simulation_dt_s_;
      const double sx = x + vx * t;
      const double sy = y + vy * t;
      const double syaw = yaw + wz * t;
      // Single combined scan (footprintClearance), not two footprintCollides
      // calls at different margins - caution_margin here is additive on top
      // of the hard margin (matches the opponent-disk check below, and the
      // debug rings in /debug/avoidance_footprint), not
      // avoidance_caution_margin_m_ measured from the object boundary,
      // which would misbehave if ever tuned below avoidance_min_clearance_m_.
      bool step_collision = false, step_caution = false;
      footprintClearance(
        sx, sy, syaw, footprint, costmap,
        avoidance_min_clearance_m_, avoidance_caution_margin_m_,
        step_collision, step_caution);
      if (step_collision) {
        collision = true;
        break;
      }
      caution = caution || step_caution;
      // The costmap snapshot above only has the obstacle's PAST position -
      // against a closing obstacle that's stale by the time this step's t
      // has elapsed. Separately check the predicted obstacle position
      // (current position + estimated velocity * t) as a disk, so a
      // fast-closing opponent is rejected here even though it hasn't
      // reached that costmap cell yet.
      if (opponent_prediction_enabled_ && snap_has_obstacle_estimate) {
        const double pred_ox = snap_obstacle_x + snap_obstacle_vx * t;
        const double pred_oy = snap_obstacle_y + snap_obstacle_vy * t;
        const double center_dist = std::hypot(pred_ox - sx, pred_oy - sy);
        const double stop_at = opponent_radius_m_ + scaled_opponent_safety_margin +
          robot_circumradius;
        if (center_dist <= stop_at) {
          collision = true;
          break;
        }
        if (!caution && center_dist <= stop_at + scaled_opponent_caution_margin) {
          caution = true;
        }
        min_opponent_margin = std::min(min_opponent_margin, center_dist - stop_at);
      }
    }
    if (collision) {
      continue;
    }
    found = true;

    const double j_goal = std::hypot(vx - desired_vx_world, vy - desired_vy_world) +
      0.3 * std::fabs(wz - desired_wz);
    const double j_smooth = std::hypot(vx - last_vx_world_, vy - last_vy_world_) +
      0.3 * std::fabs(wz - last_wz_);
    double j_obstacle = caution ? 0.5 : 0.0;
    // scaled_opponent_caution_margin, not the raw param - when it's ~0
    // (stationary opponent) the disk check above degenerates to the hard
    // stop_at threshold alone, so there's no soft margin left to grade.
    if (std::isfinite(min_opponent_margin) && scaled_opponent_caution_margin > 1e-9) {
      j_obstacle += clamp(1.0 - min_opponent_margin / scaled_opponent_caution_margin, 0.0, 1.0);
    }
    const double cost = avoidance_weight_goal_ * j_goal +
      avoidance_weight_smooth_ * j_smooth +
      avoidance_weight_obstacle_ * j_obstacle;
    if (cost < best_cost) {
      best_cost = cost;
      safe_vx_world = vx;
      safe_vy_world = vy;
      safe_wz = wz;
    }
  }

  if (!found) {
    // No candidate at all was collision-free, including max-braking -
    // the robot is already too close to something. Controlled stop:
    // brake translation at the acceleration limit, and don't trust
    // rotation either (an unverified spin could swing the footprint into
    // whatever is already this close).
    safe_vx_world = brake_vx;
    safe_vy_world = brake_vy;
    safe_wz = 0.0;
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 1000,
      "%s: no collision-free candidate in local avoidance - commanding a stop",
      plugin_name_.c_str());
    return false;
  }
  return true;
}

}  // namespace motion_controller

PLUGINLIB_EXPORT_CLASS(motion_controller::PurePursuitController, nav2_core::Controller)
