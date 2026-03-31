#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <aruco_interfaces/msg/detected_tag_array.hpp>
#include <aruco_interfaces/msg/cluster_pickability.hpp>
#include <aruco_interfaces/msg/arm_assignment.hpp>

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>

// ─── Grab frame: 4 arms on Y axis, 5 cm spacing, X = approach_offset ─────────
static constexpr int NUM_ARMS = 4;
static constexpr double ARM_SPACING = 0.05;  // meters

struct ArmPose { double x, y; };

class TagManagerNode : public rclcpp::Node
{
public:
  TagManagerNode()
  : Node("tag_manager_node")
  {
    // Parameters
    declare_parameter<std::vector<long int>>("object_tag_ids",    std::vector<long int>{});
    declare_parameter<std::vector<long int>>("localization_tag_ids", std::vector<long int>{});
    declare_parameter<double>("arm_y_origin",           0.0);   // y of arm[0] in robot frame
    declare_parameter<double>("arm_x_offset",           0.3);   // x approach distance
    declare_parameter<double>("max_assignment_distance", 0.15); // m — ignore tag if farther
    declare_parameter<double>("consistency_threshold",   0.03); // m — max residual to be pickable

    // Build arm poses
    double y0  = get_parameter("arm_y_origin").as_double();
    double x0  = get_parameter("arm_x_offset").as_double();
    for (int i = 0; i < NUM_ARMS; ++i) {
      arm_poses_[i] = { x0, y0 + i * ARM_SPACING };
    }

    // Build ID sets
    auto object_ids = get_parameter("object_tag_ids").as_integer_array();
    for (auto id : object_ids)
      object_ids_.insert(static_cast<uint32_t>(id));
    auto localization_ids = get_parameter("localization_tag_ids").as_integer_array();
    for (auto id : localization_ids)
      localization_ids_.insert(static_cast<uint32_t>(id));

    // Publishers
    pickability_pub_ = create_publisher<aruco_interfaces::msg::ClusterPickability>(
      "/cluster_pickability", 10);
    localization_pub_ = create_publisher<aruco_interfaces::msg::DetectedTagArray>(
      "/localization_tags", 10);

    // Subscriber
    tags_sub_ = create_subscription<aruco_interfaces::msg::DetectedTagArray>(
      "/detected_tags", 10,
      std::bind(&TagManagerNode::tags_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "tag_manager_node started");
  }

private:
  // ── helpers ────────────────────────────────────────────────────────────────
  double dist2d(double dx, double dy) { return std::sqrt(dx*dx + dy*dy); }

  // Least-squares 2-DOF rigid correction (tx, ty) — ignores rotation for now.
  // Returns mean (tx, ty) over all assigned pairs and per-pair residuals.
  void fit_rigid_correction(
    const std::vector<std::pair<int,int>>& pairs,   // (arm_index, tag_index)
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    double& out_tx, double& out_ty,
    std::vector<double>& out_residuals)
  {
    // Mean correction
    double sum_tx = 0, sum_ty = 0;
    for (auto& [arm_idx, tag_idx] : pairs) {
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      sum_tx += tp.x - arm_poses_[arm_idx].x;
      sum_ty += tp.y - arm_poses_[arm_idx].y;
    }
    out_tx = sum_tx / pairs.size();
    out_ty = sum_ty / pairs.size();

    // Residual per pair after applying mean correction
    out_residuals.clear();
    for (auto& [arm_idx, tag_idx] : pairs) {
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      double rx = (tp.x - arm_poses_[arm_idx].x) - out_tx;
      double ry = (tp.y - arm_poses_[arm_idx].y) - out_ty;
      out_residuals.push_back(dist2d(rx, ry));
    }
  }

  // ── main callback ──────────────────────────────────────────────────────────
  void tags_callback(const aruco_interfaces::msg::DetectedTagArray::SharedPtr msg)
  {
    double max_assign  = get_parameter("max_assignment_distance").as_double();
    double consist_thr = get_parameter("consistency_threshold").as_double();

    // Split tags
    aruco_interfaces::msg::DetectedTagArray loc_array;
    loc_array.header = msg->header;

    // Collect object tags grouped by cluster_id
    std::map<int32_t, std::vector<size_t>> clusters; // cluster_id → tag indices
    for (size_t i = 0; i < msg->tags.size(); ++i) {
      const auto& tag = msg->tags[i];
      if (object_ids_.count(tag.tag_id)) {
        if (tag.cluster_id >= 0)
          clusters[tag.cluster_id].push_back(i);
      } else if (localization_ids_.count(tag.tag_id)) {
        loc_array.tags.push_back(tag);
      }
    }

    // Forward localization tags
    if (!loc_array.tags.empty())
      localization_pub_->publish(loc_array);

    // Process each cluster
    for (auto& [cluster_id, tag_indices] : clusters) {
      aruco_interfaces::msg::ClusterPickability result;
      result.header    = msg->header;
      result.cluster_id = cluster_id;

      // Initialize all arms as unassigned
      for (int a = 0; a < NUM_ARMS; ++a) {
        aruco_interfaces::msg::ArmAssignment aa;
        aa.arm_index = a;
        aa.assigned  = false;
        result.arms[a] = aa;
      }

      // --- Assign each tag to nearest arm ---
      // arm_best[arm] = {distance, tag_index}
      std::array<std::pair<double,int>, NUM_ARMS> arm_best;
      for (auto& ab : arm_best) ab = { 1e9, -1 };

      for (size_t ti : tag_indices) {
        const auto& tp = msg->tags[ti].tag_pose.position;
        for (int a = 0; a < NUM_ARMS; ++a) {
          double d = dist2d(tp.x - arm_poses_[a].x, tp.y - arm_poses_[a].y);
          if (d < max_assign && d < arm_best[a].first) {
            arm_best[a] = { d, static_cast<int>(ti) };
          }
        }
      }

      // Build valid pairs and fill raw errors
      std::vector<std::pair<int,int>> valid_pairs; // (arm_idx, tag_idx)
      for (int a = 0; a < NUM_ARMS; ++a) {
        if (arm_best[a].second < 0) continue;
        int ti = arm_best[a].second;
        valid_pairs.push_back({a, ti});

        auto& aa     = result.arms[a];
        aa.arm_index = a;
        aa.tag_id    = msg->tags[ti].tag_id;
        aa.assigned  = true;
      }

      result.assigned_count = static_cast<uint8_t>(valid_pairs.size());

      // --- Rigid correction fit + consistency check ---
      if (!valid_pairs.empty()) {
        double tx, ty;
        std::vector<double> residuals;
        fit_rigid_correction(valid_pairs, msg, tx, ty, residuals);

        result.correction.x = tx;
        result.correction.y = ty;
        result.correction.z = 0.0;
        result.correction_magnitude = static_cast<float>(dist2d(tx, ty));

        // Check consistency across all arm assignments
        bool all_consistent = true;
        for (double r : residuals) {
          if (r > consist_thr) { all_consistent = false; break; }
        }

        result.is_pickable = all_consistent;
      } else {
        result.is_pickable = false;
        result.correction_magnitude = 0.0f;
      }

      pickability_pub_->publish(result);
    }
  }

  // ── members ────────────────────────────────────────────────────────────────
  std::array<ArmPose, NUM_ARMS> arm_poses_;
  std::unordered_set<uint32_t>  object_ids_;
  std::unordered_set<uint32_t>  localization_ids_;

  rclcpp::Subscription<aruco_interfaces::msg::DetectedTagArray>::SharedPtr tags_sub_;
  rclcpp::Publisher<aruco_interfaces::msg::ClusterPickability>::SharedPtr  pickability_pub_;
  rclcpp::Publisher<aruco_interfaces::msg::DetectedTagArray>::SharedPtr    localization_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagManagerNode>());
  rclcpp::shutdown();
  return 0;
}
