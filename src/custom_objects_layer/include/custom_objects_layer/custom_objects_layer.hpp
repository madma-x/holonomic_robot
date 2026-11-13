#pragma once

#include <mutex>
#include <string>
#include <unordered_map>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace custom_objects_layer
{

class CustomObjectsLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  struct Obj
  {
    double x{0.0};
    double y{0.0};
    double size_x{0.15};
    double size_y{0.20};
    double theta{0.0};
    bool operator==(const Obj & o) const
    {
      return x == o.x && y == o.y && size_x == o.size_x && size_y == o.size_y && theta == o.theta;
    }
  };

  void markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  std::string key(std::size_t ns_hash, int32_t id) const;
  void drawRectangle(
    nav2_costmap_2d::Costmap2D & grid,
    double center_x, double center_y,
    double size_x, double size_y,
    double theta, unsigned char cost);

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  std::mutex mtx_;
  std::unordered_map<std::string, Obj> objects_;
  std::unordered_map<std::string, Obj> prev_objects_;  // objects painted in the last updateCosts
  // updateBounds covers every object only when this is set (start-up, after a costmap clear); otherwise
  // only objects added / removed / moved since the last updateCosts - static ones (keep-out zones,
  // built piles) then stop forcing a near-full-map reinflation every cycle
  bool full_redraw_{true};
  std::string topic_;
  int lethal_cost_{254};
};

}  // namespace custom_objects_layer
