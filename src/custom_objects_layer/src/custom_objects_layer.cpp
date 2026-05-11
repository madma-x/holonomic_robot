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
  declareParameter("object_size_x", rclcpp::ParameterValue(0.15));
  declareParameter("object_size_y", rclcpp::ParameterValue(0.20));

  node->get_parameter(name_ + "." + "topic", topic_);
  int lethal_cost_int;
  node->get_parameter(name_ + "." + "lethal_cost", lethal_cost_int);
  lethal_cost_ = lethal_cost_int;
  node->get_parameter(name_ + "." + "object_size_x", object_size_x_);
  node->get_parameter(name_ + "." + "object_size_y", object_size_y_);

  sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&CustomObjectsLayer::markersCallback, this, std::placeholders::_1));
}

void CustomObjectsLayer::reset()
{
  std::lock_guard<std::mutex> lk(mtx_);
  objects_.clear();
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

    objects_[object_key] = object;
  }

  current_ = false;
}

void CustomObjectsLayer::updateBounds(
  double /* robot_x */, double /* robot_y */, double /* robot_yaw */,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lk(mtx_);

  const double half_x = object_size_x_ * 0.5;
  const double half_y = object_size_y_ * 0.5;

  for (const auto & [_, object] : objects_) {
    *min_x = std::min(*min_x, object.x - half_x);
    *min_y = std::min(*min_y, object.y - half_y);
    *max_x = std::max(*max_x, object.x + half_x);
    *max_y = std::max(*max_y, object.y + half_y);
  }
}

void CustomObjectsLayer::drawRectangle(
  nav2_costmap_2d::Costmap2D & grid, double center_x, double center_y, unsigned char cost)
{
  const double min_x = center_x - (object_size_x_ * 0.5);
  const double max_x = center_x + (object_size_x_ * 0.5);
  const double min_y = center_y - (object_size_y_ * 0.5);
  const double max_y = center_y + (object_size_y_ * 0.5);

  int min_mx_i, min_my_i, max_mx_i, max_my_i;
  grid.worldToMapEnforceBounds(min_x, min_y, min_mx_i, min_my_i);
  grid.worldToMapEnforceBounds(max_x, max_y, max_mx_i, max_my_i);
  if (min_mx_i < 0 || min_my_i < 0) {
    return;
  }
  unsigned int min_mx = static_cast<unsigned int>(min_mx_i);
  unsigned int min_my = static_cast<unsigned int>(min_my_i);
  unsigned int max_mx = static_cast<unsigned int>(max_mx_i);
  unsigned int max_my = static_cast<unsigned int>(max_my_i);

  for (unsigned int mx = min_mx; mx <= max_mx; ++mx) {
    for (unsigned int my = min_my; my <= max_my; ++my) {
      const auto old_cost = grid.getCost(mx, my);
      grid.setCost(mx, my, std::max(old_cost, cost));
    }
  }
}

void CustomObjectsLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /* min_i */, int /* min_j */, int /* max_i */, int /* max_j */)
{
  std::lock_guard<std::mutex> lk(mtx_);

  for (const auto & [_, object] : objects_) {
    drawRectangle(master_grid, object.x, object.y, static_cast<unsigned char>(lethal_cost_));
  }

  current_ = true;
}

}  // namespace custom_objects_layer

PLUGINLIB_EXPORT_CLASS(custom_objects_layer::CustomObjectsLayer, nav2_costmap_2d::Layer)
