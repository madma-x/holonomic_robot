#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>

using std::placeholders::_1;

class ScanFilterNode : public rclcpp::Node
{
public:
  ScanFilterNode() : Node("scan_filter")
  {
    scan_in_ = this->declare_parameter<std::string>("scan_in_topic", "/scan");
    scan_out_ = this->declare_parameter<std::string>("scan_out_topic", "/scan_filtered");
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/odom");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");

    have_pose_ = false;
    have_map_ = false;

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_in_, 10, std::bind(&ScanFilterNode::scan_cb, this, _1));
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, 10, std::bind(&ScanFilterNode::pose_cb, this, _1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, 1, std::bind(&ScanFilterNode::map_cb, this, _1));

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_out_, 10);

    RCLCPP_INFO(this->get_logger(), "scan_filter started (in='%s' out='%s')", scan_in_.c_str(), scan_out_.c_str());
  }

private:
  std::string scan_in_, scan_out_, pose_topic_, map_topic_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  double robot_x_{0.0}, robot_y_{0.0}, robot_theta_{0.0};
  bool have_pose_;
  bool have_map_;
  double xmin_{0.0}, xmax_{0.0}, ymin_{0.0}, ymax_{0.0};

  void pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    robot_theta_ = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
    have_pose_ = true;
  }

  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    double ox = msg->info.origin.position.x;
    double oy = msg->info.origin.position.y;
    double res = msg->info.resolution;
    double w = static_cast<double>(msg->info.width);
    double h = static_cast<double>(msg->info.height);

    xmin_ = ox;
    ymin_ = oy;
    xmax_ = ox + w * res;
    ymax_ = oy + h * res;
    have_map_ = true;
    RCLCPP_INFO(this->get_logger(), "map bounds: x[%.3f,%.3f] y[%.3f,%.3f]", xmin_, xmax_, ymin_, ymax_);
  }

  // slab method
  bool ray_rect_intersection(double px, double py, double dx, double dy, double &out_t)
  {
    double tmin = -INFINITY;
    double tmax = INFINITY;

    auto slab = [&](double p, double d, double mn, double mx) -> bool {
      if (std::fabs(d) < 1e-9) {
        return (p >= mn && p <= mx);
      }
      double t1 = (mn - p) / d;
      double t2 = (mx - p) / d;
      double ta = std::fmin(t1, t2);
      double tb = std::fmax(t1, t2);
      if (ta > tmin) tmin = ta;
      if (tb < tmax) tmax = tb;
      return tmax >= tmin;
    };

    if (!slab(px, dx, xmin_, xmax_)) return false;
    if (!slab(py, dy, ymin_, ymax_)) return false;

    if (tmax >= std::fmax(tmin, 0.0)) {
      out_t = (tmin >= 0.0) ? tmin : tmax;
      return true;
    }
    return false;
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (pub_->get_subscription_count() == 0) return;

    // if missing pose/map just forward raw scan to avoid interfering
    if (!have_pose_ || !have_map_) {
      pub_->publish(*msg);
      return;
    }

    // copy header/meta once
    sensor_msgs::msg::LaserScan out = *msg;
    double angle = msg->angle_min;
    const size_t N = msg->ranges.size();

    for (size_t i = 0; i < N; ++i) {
      double r = static_cast<double>(msg->ranges[i]);
      if (!std::isfinite(r) || r <= msg->range_min) {
        out.ranges[i] = msg->range_max;
        angle += msg->angle_increment;
        continue;
      }

      double beam_angle = robot_theta_ + angle;
      double dx = std::cos(beam_angle);
      double dy = std::sin(beam_angle);

      double allowed_t = 0.0;
      if (!ray_rect_intersection(robot_x_, robot_y_, dx, dy, allowed_t)) {
        out.ranges[i] = msg->range_max; // outside field
      } else {
        if (r <= allowed_t + 1e-3) out.ranges[i] = static_cast<float>(r);
        else out.ranges[i] = msg->range_max; // mask beyond field
      }
      angle += msg->angle_increment;
    }

    pub_->publish(out);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
