#ifndef MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace motion_controller
{

class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  bool cancel() override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct LookaheadPoint
  {
    double x {0.0};
    double y {0.0};
    double heading {0.0};
  };

  static double clamp(double value, double min_value, double max_value);
  static double normalizeAngle(double angle);
  static double euclideanDistance(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b);

  size_t findClosestPathIndex(const geometry_msgs::msg::PoseStamped & pose) const;
  LookaheadPoint findLookaheadPoint(size_t closest_index, double lookahead_distance) const;
  // Fixed docking line for the final approach, derived from the PLAN's own
  // geometry (not the robot's current, possibly already-drifted pose) - the
  // direction from the point docking_lookback_dist_ back along the path to
  // the goal. Cached once per setPlan() so the line doesn't wobble tick to
  // tick as the robot corrects onto it. Falls back to the goal orientation
  // if the plan is shorter than docking_lookback_dist_ (nothing to measure).
  void updateApproachAxis();
  // Ramps (vx_world, vy_world, wz) toward the target at max_a_/max_aw_,
  // holding the ramped state in last_vx_world_/last_vy_world_/last_wz_, then
  // converts to a body-frame Twist using `yaw` for the output. World-frame
  // so a previous tick's command (captured at a different yaw) doesn't fight
  // the current rotation the way ramping in body frame did.
  geometry_msgs::msg::Twist applyAccelerationLimits(
    double target_vx_world, double target_vy_world, double target_wz,
    double yaw, double dt);
  geometry_msgs::msg::Twist applyOutputFrameTransform(
    const geometry_msgs::msg::Twist & input_cmd) const;
  bool isPathLocallyValid(size_t closest_index) const;

  // Holonomic local avoidance (DWA-style): samples candidate world-frame
  // (vx, vy, wz) around Pure Pursuit's desired command, forward-simulates
  // each over a short horizon, rejects any whose swept footprint collides
  // with the live local costmap, and returns the lowest-cost survivor via
  // the safe_* out-params. Falls through to a controlled deceleration (not
  // the raw desired command) when no candidate is collision-free - never a
  // silent unprotected pass-through. Returns false exactly in that
  // no-safe-candidate case (safe_* is still a valid, feasible command).
  bool selectSafeVelocity(
    double desired_vx_world, double desired_vy_world, double desired_wz,
    double x, double y, double yaw, double dt,
    double & safe_vx_world, double & safe_vy_world, double & safe_wz);
  // True if the footprint (base-frame polygon from costmap_ros_, transformed
  // by x/y/theta) is within hard_margin of an unknown-and-treated-as-
  // obstacle or at/above avoidance_obstacle_cost_threshold_ cell
  // (`collision`), and separately whether it's within hard_margin +
  // caution_margin of one (`caution`) - a single grid scan at the larger
  // margin's bounding box, each cell's distance-to-polygon computed once
  // and tested against both thresholds, instead of two full independent
  // scans at different margins (what this replaced: selectSafeVelocity was
  // calling that twice per step per candidate - ~1800 full bbox scans per
  // control tick, the single biggest cost in a loop that was missing
  // controller_server's 20Hz deadline). Early-exits on an actual collision
  // (nothing finer to learn after that); still scans the rest of the cells
  // for `caution` when no collision is found.
  void footprintClearance(
    double x, double y, double theta,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    nav2_costmap_2d::Costmap2D * costmap,
    double hard_margin, double caution_margin,
    bool & collision, bool & caution) const;
  // A static local-costmap snapshot under-reacts to a FAST-CLOSING obstacle:
  // by the time a candidate's forward-simulated footprint reaches a point,
  // the real obstacle (tracked only via its last known cell) has already
  // moved past where the costmap still shows it - confirmed by a live test
  // (opponent + robot closing head-on at ~0.9 m/s combined still collided
  // with the static check alone). Tracks the centroid of nearby LETHAL
  // cells tick-to-tick as a crude velocity estimate (no dependency on any
  // sim-only ground truth or strategy-specific tracker - this is the only
  // obstacle-motion signal a generic Nav2 controller plugin has), so
  // selectSafeVelocity can check candidates against where the obstacle is
  // predicted to BE at each future step, not just where it last was.
  void updateObstacleEstimate(double x, double y, nav2_costmap_2d::Costmap2D * costmap);
  // RViz view of mechanism 2 (the predicted-opponent disk, independent of
  // the costmap's own inflation gradient - see chat): tracked position,
  // predicted position at the prediction horizon, and the hard-stop /
  // caution circles actually used to reject candidates. Lets the two
  // mechanisms' extents be visually compared instead of only inferred from
  // parameter values.
  void publishOpponentDebug(double robot_circumradius);
  // RViz view of mechanism 1's margins: the robot's actual footprint
  // outline, dilated by avoidance_min_clearance_m (hard-reject) and by
  // + avoidance_caution_margin_m (caution, scored not rejected) - the same
  // two distances footprintCollides checks against the costmap, drawn
  // around the real footprint shape instead of a fixed circle, so this and
  // publishOpponentDebug's disks can be visually compared side by side.
  void publishFootprintDebug(
    double x, double y, double theta,
    const std::vector<geometry_msgs::msg::Point> & footprint);
  // Mechanism 2 otherwise only ever runs inside computeVelocityCommands,
  // which Nav2 only calls while a FollowPath goal is active - so there's
  // no way to watch the opponent tracker/prediction settle (and check it's
  // not jerky) without also having our own robot actively driving, which
  // confounds exactly the rolling-window-jitter question that matters.
  // This timer runs updateObstacleEstimate + the debug publishers on their
  // own schedule regardless of whether Nav2 is currently commanding
  // anything, using a direct TF lookup for the robot pose (the same
  // approach table_sim's goal_driver.py uses) instead of the pose
  // computeVelocityCommands is normally handed.
  void obstacleTrackingTimerCallback();
  rclcpp::TimerBase::SharedPtr obstacle_tracking_timer_;
  double opponent_tracking_rate_hz_ {10.0};
  // Guards obstacle_x_/y_/vx_/vy_/has_obstacle_estimate_/obstacle_stamp_,
  // now written from two places (computeVelocityCommands' thread and this
  // timer's callback) instead of one.
  std::mutex obstacle_mutex_;
  // declare_parameter only seeds the initial value - without this, `ros2
  // param set` updates the parameter server but every tunable in this file
  // keeps running on whatever was read once in configure(), which is no
  // good for tuning the avoidance layer live against a running sim.
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuitController")};
  std::string plugin_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr opponent_debug_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_debug_pub_;

  nav_msgs::msg::Path global_plan_;
  mutable size_t last_closest_index_ {0};

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  bool was_cancelled_ {false};

  // Acceleration-ramp state, in the WORLD frame (see applyAccelerationLimits).
  double last_vx_world_ {0.0};
  double last_vy_world_ {0.0};
  double last_wz_ {0.0};

  double base_max_vx_ {0.3};
  double base_max_vy_ {0.3};
  // Isotropic speed ceiling. max_vx/max_vy alone describe a SQUARE in body
  // frame, so the achievable speed along a fixed world direction swung by up
  // to sqrt(2) as the robot rotated, and clipping one component but not the
  // other bent the commanded direction away from the lookahead bearing -
  // i.e. rotating changed where the robot actually went. 0 = derive from
  // min(max_vx, max_vy).
  double max_v_ {0.0};
  double max_wz_ {1.5};
  // Isotropic accel ceiling too - max_ax_/max_ay_ had the identical per-axis
  // problem the clamp did (worse: ramping a stale-yaw body-frame last
  // command against separate x/y limits actively fought rotation, throttling
  // translation the faster the robot turned). 0 = derive from min(max_ax,
  // max_ay) for a one-release migration path from the old two params.
  double max_a_ {0.0};
  double max_ax_ {1.5};
  double max_ay_ {1.5};
  double max_aw_ {3.0};

  double lookahead_dist_ {0.35};
  double lookahead_time_ {0.6};
  double min_lookahead_ {0.20};
  double max_lookahead_ {0.80};

  double k_xy_ {1.4};
  double k_theta_ {1.2};

  // Terminal yaw stage: once within goal_dist_tolerance_ of the goal but
  // still outside goal_yaw_tolerance_, hold position and turn. Nav2's own
  // SimpleGoalChecker is configured with a loose yaw tolerance on purpose
  // (it would otherwise block the "arrived" transition while this finishes),
  // so without this the requested goal orientation is never actually
  // delivered - which is exactly what used to happen: both tolerances were
  // declared, read, and then never used anywhere.
  double goal_dist_tolerance_ {0.05};
  double goal_yaw_tolerance_ {0.175};   // 10 deg

  // Terminal docking stage: inside docking_distance_ of the goal, steer in
  // the approach-axis frame (along-track toward the goal, cross-track back
  // onto the fixed docking line from updateApproachAxis()) instead of
  // chasing the pure-pursuit lookahead point. Pure pursuit's lookahead point
  // sits ON the path, so once the robot carries side velocity INTO the dock
  // (e.g. handed off from a curved upstream segment) the lookahead-derived
  // command only ever asks for whatever residual position error remains -
  // small near the goal - while the real lateral momentum is governed solely
  // by the general accel ramp, which isn't tied to how far off the docking
  // line the robot actually is. Separating cross-track from along-track lets
  // k_lateral_ correct that offset directly instead of waiting on it to show
  // up as position error against a moving lookahead target.
  bool docking_enabled_ {true};
  double docking_distance_ {0.3};
  double docking_lookback_dist_ {0.4};
  // 0 = derive from k_xy_ (same sentinel convention as max_v_/max_a_ above).
  double k_lateral_ {0.0};

  bool has_approach_axis_ {false};
  double approach_axis_x_ {1.0};
  double approach_axis_y_ {0.0};

  double output_rotate_deg_ {0.0};
  bool output_invert_x_ {false};
  bool output_invert_y_ {false};
  bool output_invert_wz_ {false};
  bool enable_local_path_validity_check_ {true};
  double local_path_check_distance_ {0.8};
  int local_path_invalid_cost_threshold_ {253};
  bool local_path_unknown_is_invalid_ {true};
  bool publish_lookahead_point_ {true};
  std::string lookahead_topic_ {"/debug/lookahead_point"};
  bool publish_opponent_debug_ {true};
  std::string opponent_debug_topic_ {"/debug/opponent_prediction"};
  bool publish_footprint_debug_ {true};
  std::string footprint_debug_topic_ {"/debug/avoidance_footprint"};

  // Opt-in (default off): other configs (e.g. the real robot's) keep today's
  // behaviour - enable_local_path_validity_check_'s throw-and-let-Nav2-
  // recover - until this is explicitly turned on for them. When on, it
  // supersedes that check entirely (see computeVelocityCommands): a single
  // whole-path-ahead-or-nothing validity test can't react gracefully to a
  // fast-closing obstacle the way per-cycle candidate re-selection can.
  bool local_avoidance_enabled_ {false};
  double avoidance_prediction_horizon_s_ {0.6};
  double avoidance_simulation_dt_s_ {0.1};
  // Halved from 5/5/3 (Max's request - the single biggest cost in a loop
  // that was missing controller_server's 20Hz deadline, see
  // footprintClearance's comment): candidate count drops from 5*5*3+2=77 to
  // 3*3*3+2=29, keeping a centre + 2 edge samples per axis (still covers
  // "brake harder/softer" and "left/right/straight" without the full 5-wide
  // grid's finer gradations).
  int avoidance_vx_samples_ {3};
  int avoidance_vy_samples_ {3};
  int avoidance_wz_samples_ {3};
  int avoidance_obstacle_cost_threshold_ {253};
  // Real robot default true (its lidar clears free space, so "unknown"
  // really does mean unexplored). Sim-only override false, same reasoning
  // as local_path_unknown_is_invalid_ - table_sim's mast lidar never clears
  // anything, so permanently-unknown cells must stay traversable there.
  bool avoidance_unknown_is_obstacle_ {true};
  double avoidance_min_clearance_m_ {0.03};
  double avoidance_caution_margin_m_ {0.15};
  double avoidance_weight_goal_ {1.0};
  double avoidance_weight_smooth_ {0.3};
  double avoidance_weight_obstacle_ {0.5};
  // Toggle for mechanism 2 (the predicted-opponent disk) independent of
  // local_avoidance_enabled_ (mechanism 1, the costmap-cost check) - lets
  // the two be tuned one at a time instead of always seeing their combined
  // effect. Off: the per-step disk collision/caution check is skipped
  // entirely and /debug/opponent_prediction publishes DELETEALL only.
  bool opponent_prediction_enabled_ {true};
  double opponent_tracking_radius_m_ {2.0};
  // Halved from 0.25/0.1 (Max's request - mechanism 2 was too big even at
  // full speed-scaled effect).
  double opponent_radius_m_ {0.125};
  double opponent_safety_margin_m_ {0.05};
  // Mechanism 2 (this disk) exists only to catch a FAST-CLOSING opponent
  // before a static costmap snapshot would (see updateObstacleEstimate) - a
  // STATIONARY opponent is already handled by mechanism 1 (footprint vs.
  // costmap clearance, avoidance_min_clearance_m_/avoidance_caution_margin_m_),
  // which uses much smaller margins. Without this, mechanism 2's disk (radius
  // opponent_radius_m_+opponent_safety_margin_m_, well past mechanism 1's
  // reach) dominated even against a parked opponent - confirmed live: the
  // robot gave a stationary opponent a wide, unnecessary berth that a real
  // match's tighter stacks/zones can't afford. opponent_safety_margin_m_ and
  // avoidance_caution_margin_m_ (as applied to the opponent disk only) are
  // scaled by how fast the opponent is currently moving: zero extra margin
  // at/below opponent_speed_deadband_m_s_ (so the disk collapses to bare
  // contact radius, smaller than mechanism 1's own margins, and mechanism 1
  // alone governs), ramping linearly to the full configured margins at/above
  // opponent_speed_full_effect_m_s_.
  double opponent_speed_deadband_m_s_ {0.1};
  double opponent_speed_full_effect_m_s_ {0.6};
  // Simple adaptive speed limit: caps max_v as the robot's distance to the
  // opponent's CURRENT (not predicted) position shrinks, independent of
  // whether the opponent is moving - approaching an obstacle slowly gives
  // both mechanisms more reaction room regardless of its own speed. Full
  // speed at/beyond contact-radius + opponent_speed_limit_range_m_, ramping
  // down to opponent_speed_limit_min_scale_ * max_v at contact radius.
  bool opponent_speed_limit_enabled_ {true};
  double opponent_speed_limit_range_m_ {0.6};
  double opponent_speed_limit_min_scale_ {0.35};
  // No sighting for longer than this -> prediction falls back to the last
  // known position with zero velocity (section 5: "do not assume constant
  // velocity indefinitely", and low/no confidence should widen margin, not
  // extrapolate forever). Also resets the estimate on too large a jump.
  double opponent_estimate_timeout_s_ {1.0};
  // The tracked centroid comes from rasterizing a small (mast-sized)
  // object into costmap cells - as the local costmap's rolling window
  // re-centers on our own moving robot, that handful of cells can snap to
  // a different grid alignment tick to tick even though the real object
  // hasn't moved, producing a tick-to-tick centroid jump below this
  // threshold that's quantization noise, not real motion. Confirmed live:
  // visibly jerky predicted motion for a STATIONARY opponent, only while
  // our own robot was moving (which is what drives the rolling-window
  // re-centering - a stationary robot never saw this).
  double opponent_velocity_deadband_m_ {0.04};
  // Exponential smoothing factor for the velocity estimate (0..1): 1.0 =
  // no smoothing (raw finite difference, the old/jerky behaviour), lower =
  // more damping but slower to react to genuine opponent acceleration.
  double opponent_velocity_smoothing_ {0.3};
  // Hard clamp on the RAW instantaneous estimate before smoothing - the
  // deadband alone didn't fully kill spikes (a single bad quantization
  // jump still gets through to the filter input and takes several ticks
  // to decay out under smoothing alone). Set well above any real robot's
  // top speed so it never clips genuine motion, only quantization noise.
  double opponent_velocity_max_m_s_ {3.0};

  bool has_obstacle_estimate_ {false};
  double obstacle_x_ {0.0};
  double obstacle_y_ {0.0};
  double obstacle_vx_ {0.0};
  double obstacle_vy_ {0.0};
  rclcpp::Time obstacle_stamp_;

  double speed_limit_scale_ {1.0};
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
