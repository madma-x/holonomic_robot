#ifndef MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

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
  geometry_msgs::msg::Twist applyAccelerationLimits(
    const geometry_msgs::msg::Twist & target,
    double dt) const;
  geometry_msgs::msg::Twist applyOutputFrameTransform(
    const geometry_msgs::msg::Twist & input_cmd) const;
  double computeWzScaleFromLinearSpeed(double linear_speed) const;
  bool isPathLocallyValid(size_t closest_index) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuitController")};
  std::string plugin_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;

  nav_msgs::msg::Path global_plan_;
  mutable size_t last_closest_index_ {0};

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  bool was_cancelled_ {false};

  double base_max_vx_ {0.3};
  double base_max_vy_ {0.3};
  double max_wz_ {1.5};
  double max_ax_ {1.5};
  double max_ay_ {1.5};
  double max_aw_ {3.0};

  double lookahead_dist_ {0.35};
  double lookahead_time_ {0.6};
  double min_lookahead_ {0.20};
  double max_lookahead_ {0.80};

  double k_xy_ {1.4};
  double k_theta_ {1.2};

  double goal_dist_tolerance_ {0.05};
  double goal_yaw_tolerance_ {0.15};

  double linear_speed_for_wz_scale_start_ {0.12};
  double linear_speed_for_wz_scale_end_ {0.30};
  double min_wz_scale_at_high_speed_ {0.30};
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

  double speed_limit_scale_ {1.0};
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
