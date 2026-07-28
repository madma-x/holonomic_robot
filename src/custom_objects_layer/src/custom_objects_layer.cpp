#include "custom_objects_layer/custom_objects_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace custom_objects_layer
{

void CustomObjectsLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in CustomObjectsLayer::onInitialize");
  }

  enabled_ = true;
  current_ = true;

  declareParameter("topic", rclcpp::ParameterValue(std::string("/custom_objects")));
  declareParameter("lethal_cost", rclcpp::ParameterValue(254));

  node->get_parameter(name_ + "." + "topic", topic_);
  int lethal_cost_int;
  node->get_parameter(name_ + "." + "lethal_cost", lethal_cost_int);
  lethal_cost_ = lethal_cost_int;

  sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&CustomObjectsLayer::markersCallback, this, std::placeholders::_1));
}

void CustomObjectsLayer::reset()
{
  std::lock_guard<std::mutex> lk(mtx_);
  objects_.clear();
  prev_objects_.clear();
  current_ = false;
}

std::string CustomObjectsLayer::key(std::size_t ns_hash, int32_t id) const
{
  return std::to_string(ns_hash) + ":" + std::to_string(id);
}

void CustomObjectsLayer::markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);

  for (const auto & marker : msg->markers) {
    const auto hash = std::hash<std::string>{}(marker.ns);
    const auto object_key = key(hash, marker.id);

    if (marker.action == visualization_msgs::msg::Marker::DELETE) {
      objects_.erase(object_key);
      continue;
    }

    if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
      objects_.clear();
      continue;
    }

    Obj object;
    object.x = marker.pose.position.x;
    object.y = marker.pose.position.y;
    object.size_x = marker.scale.x;
    object.size_y = marker.scale.y;
    // Extract yaw from quaternion (qz, qw)
    object.theta = 2.0 * std::atan2(marker.pose.orientation.z, marker.pose.orientation.w);

    objects_[object_key] = object;
  }

  current_ = false;
}

void CustomObjectsLayer::updateBounds(
  double /* robot_x */, double /* robot_y */, double /* robot_yaw */,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lk(mtx_);

  // Include both current and previous objects so deleted cells fall inside the update window
  auto expand = [&](const Obj & obj) {
    const double half_diag = 0.5 * std::hypot(obj.size_x, obj.size_y);
    *min_x = std::min(*min_x, obj.x - half_diag);
    *min_y = std::min(*min_y, obj.y - half_diag);
    *max_x = std::max(*max_x, obj.x + half_diag);
    *max_y = std::max(*max_y, obj.y + half_diag);
  };
  for (const auto & [_, obj] : objects_) { expand(obj); }
  for (const auto & [_, obj] : prev_objects_) { expand(obj); }
}

void CustomObjectsLayer::drawRectangle(
  nav2_costmap_2d::Costmap2D & grid,
  double center_x, double center_y,
  double size_x, double size_y,
  double theta, unsigned char cost)
{
  const double half_x = size_x * 0.5;
  const double half_y = size_y * 0.5;
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);

  // Four corners of the oriented rectangle in world frame
  const double corners[4][2] = {
    {center_x + cos_t * half_x - sin_t * half_y, center_y + sin_t * half_x + cos_t * half_y},
    {center_x - cos_t * half_x - sin_t * half_y, center_y - sin_t * half_x + cos_t * half_y},
    {center_x + cos_t * half_x + sin_t * half_y, center_y + sin_t * half_x - cos_t * half_y},
    {center_x - cos_t * half_x + sin_t * half_y, center_y - sin_t * half_x - cos_t * half_y},
  };

  double wx_min = corners[0][0], wx_max = corners[0][0];
  double wy_min = corners[0][1], wy_max = corners[0][1];
  for (int i = 1; i < 4; ++i) {
    wx_min = std::min(wx_min, corners[i][0]);
    wx_max = std::max(wx_max, corners[i][0]);
    wy_min = std::min(wy_min, corners[i][1]);
    wy_max = std::max(wy_max, corners[i][1]);
  }

  int min_mx_i, min_my_i, max_mx_i, max_my_i;
  grid.worldToMapEnforceBounds(wx_min, wy_min, min_mx_i, min_my_i);
  grid.worldToMapEnforceBounds(wx_max, wy_max, max_mx_i, max_my_i);
  if (min_mx_i < 0 || min_my_i < 0) {
    return;
  }
  unsigned int min_mx = static_cast<unsigned int>(min_mx_i);
  unsigned int min_my = static_cast<unsigned int>(min_my_i);
  unsigned int max_mx = static_cast<unsigned int>(max_mx_i);
  unsigned int max_my = static_cast<unsigned int>(max_my_i);

  for (unsigned int mx = min_mx; mx <= max_mx; ++mx) {
    for (unsigned int my = min_my; my <= max_my; ++my) {
      // Test if the cell centre is inside the oriented rectangle
      double wx, wy;
      grid.mapToWorld(mx, my, wx, wy);
      const double dx = wx - center_x;
      const double dy = wy - center_y;
      const double local_x = cos_t * dx + sin_t * dy;
      const double local_y = -sin_t * dx + cos_t * dy;
      if (std::abs(local_x) <= half_x && std::abs(local_y) <= half_y) {
        grid.setCost(mx, my, cost);
      }
    }
  }
}

void CustomObjectsLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /* min_i */, int /* min_j */, int /* max_i */, int /* max_j */)
{
  std::lock_guard<std::mutex> lk(mtx_);

  // Erase only cells that we painted in the previous cycle (don't touch other layers).
  for (const auto & [_, obj] : prev_objects_) {
    drawRectangle(master_grid, obj.x, obj.y, obj.size_x, obj.size_y, obj.theta,
      nav2_costmap_2d::FREE_SPACE);
  }

  // Paint current objects.
  for (const auto & [_, obj] : objects_) {
    drawRectangle(master_grid, obj.x, obj.y, obj.size_x, obj.size_y, obj.theta,
      static_cast<unsigned char>(lethal_cost_));
  }

  prev_objects_ = objects_;
  current_ = true;
}

}  // namespace custom_objects_layer

PLUGINLIB_EXPORT_CLASS(custom_objects_layer::CustomObjectsLayer, nav2_costmap_2d::Layer)
