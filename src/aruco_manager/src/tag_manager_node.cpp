#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <aruco_interfaces/msg/detected_tag_array.hpp>
#include <aruco_interfaces/msg/cluster_pickability.hpp>
#include <aruco_interfaces/msg/arm_assignment.hpp>

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>

// ─── Grab frame: 4 arms on Y axis, 5 cm spacing, X = approach_offset ─────────
static constexpr int NUM_ARMS = 4;
static constexpr double ARM_SPACING = 0.05;  // meters

struct ArmPose { double x, y; };

struct AssignmentCandidate
{
  int matched_count = 0;
  double total_cost = 0.0;
  std::vector<std::pair<int, int>> pairs;
};

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
    declare_parameter<double>("arm_x_offset",           0.0);   // x approach distance
    declare_parameter<double>("max_assignment_distance", 0.5); // m — ignore tag if farther
    declare_parameter<double>("consistency_threshold",   0.03); // m — max residual to be pickable
    declare_parameter<bool>("sticky_assignment",         false); // handler enables this only during pick window

    // Build arm poses
    double y0  = get_parameter("arm_y_origin").as_double();
    double x0  = get_parameter("arm_x_offset").as_double();
    for (int i = 0; i < NUM_ARMS; ++i) {
      arm_poses_[i] = { x0+ i * ARM_SPACING, y0};
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

    // Subscriber — use BEST_EFFORT to match sensor publishers
    tags_sub_ = create_subscription<aruco_interfaces::msg::DetectedTagArray>(
      "aruco_picker/detected_tags",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&TagManagerNode::tags_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "tag_manager_node started");

    latched_tag_ids_.fill(-1);
  }

private:
  // ── helpers ────────────────────────────────────────────────────────────────
  double dist2d(double dx, double dy) { return std::sqrt(dx*dx + dy*dy); }

  void update_best_candidate(
    const std::vector<std::pair<int, int>>& current_pairs,
    double current_cost,
    AssignmentCandidate& best)
  {
    const int current_count = static_cast<int>(current_pairs.size());
    if (current_count > best.matched_count ||
        (current_count == best.matched_count && current_cost < best.total_cost)) {
      best.matched_count = current_count;
      best.total_cost = current_cost;
      best.pairs = current_pairs;
    }
  }

  void search_best_assignment(
    int arm_idx,
    const std::vector<size_t>& object_tag_indices,
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    double max_assign,
    std::vector<bool>& used_tags,
    std::vector<std::pair<int, int>>& current_pairs,
    double current_cost,
    AssignmentCandidate& best)
  {
    if (arm_idx >= NUM_ARMS) {
      update_best_candidate(current_pairs, current_cost, best);
      return;
    }

    // Option 1: leave this arm unused.
    search_best_assignment(
      arm_idx + 1,
      object_tag_indices,
      msg,
      max_assign,
      used_tags,
      current_pairs,
      current_cost,
      best);

    // Option 2: assign one currently unused visible object tag to this arm.
    for (size_t i = 0; i < object_tag_indices.size(); ++i) {
      if (used_tags[i]) {
        continue;
      }

      const size_t tag_idx = object_tag_indices[i];
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      const double distance = dist2d(tp.x - arm_poses_[arm_idx].x, tp.y - arm_poses_[arm_idx].y);
      if (distance >= max_assign) {
        continue;
      }

      used_tags[i] = true;
      current_pairs.push_back({arm_idx, static_cast<int>(tag_idx)});
      search_best_assignment(
        arm_idx + 1,
        object_tag_indices,
        msg,
        max_assign,
        used_tags,
        current_pairs,
        current_cost + distance,
        best);
      current_pairs.pop_back();
      used_tags[i] = false;
    }
  }

  AssignmentCandidate compute_best_assignment(
    const std::vector<size_t>& object_tag_indices,
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    double max_assign)
  {
    AssignmentCandidate best;
    best.total_cost = std::numeric_limits<double>::infinity();

    std::vector<bool> used_tags(object_tag_indices.size(), false);
    std::vector<std::pair<int, int>> current_pairs;
    search_best_assignment(
      0,
      object_tag_indices,
      msg,
      max_assign,
      used_tags,
      current_pairs,
      0.0,
      best);

    if (!std::isfinite(best.total_cost)) {
      best.total_cost = 0.0;
    }
    return best;
  }

  bool build_latched_pairs(
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    const std::unordered_map<uint32_t, size_t>& index_by_tag_id,
    double max_assign,
    std::vector<std::pair<int, int>>& pairs_out)
  {
    pairs_out.clear();

    bool has_any_assignment = false;
    for (int arm_idx = 0; arm_idx < NUM_ARMS; ++arm_idx) {
      const int latched_tag_id = latched_tag_ids_[arm_idx];
      if (latched_tag_id < 0) {
        continue;
      }
      has_any_assignment = true;

      auto it = index_by_tag_id.find(static_cast<uint32_t>(latched_tag_id));
      if (it == index_by_tag_id.end()) {
        return false;
      }

      const int tag_idx = static_cast<int>(it->second);
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      const double distance = dist2d(tp.x - arm_poses_[arm_idx].x, tp.y - arm_poses_[arm_idx].y);
      if (distance >= max_assign) {
        return false;
      }

      pairs_out.push_back({arm_idx, tag_idx});
    }

    return has_any_assignment;
  }

  void latch_assignment(const AssignmentCandidate& assignment, const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg)
  {
    latched_tag_ids_.fill(-1);
    for (const auto& [arm_idx, tag_idx] : assignment.pairs) {
      latched_tag_ids_[arm_idx] = static_cast<int>(msg->tags[tag_idx].tag_id);
    }
  }

  void clear_latched_assignment()
  {
    latched_tag_ids_.fill(-1);
  }

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
    bool sticky_assignment = get_parameter("sticky_assignment").as_bool();

    // Split tags
    aruco_interfaces::msg::DetectedTagArray loc_array;
    loc_array.header = msg->header;

    // Collect all visible object tags for one whole-frame pickup opportunity.
    std::vector<size_t> object_tag_indices;
    std::unordered_map<uint32_t, size_t> index_by_tag_id;
    for (size_t i = 0; i < msg->tags.size(); ++i) {
      const auto& tag = msg->tags[i];
      if (object_ids_.count(tag.tag_id)) {
        object_tag_indices.push_back(i);
        index_by_tag_id[tag.tag_id] = i;
      } else if (localization_ids_.count(tag.tag_id)) {
        loc_array.tags.push_back(tag);
      }
    }

    // Forward localization tags
    if (!loc_array.tags.empty())
      localization_pub_->publish(loc_array);

    aruco_interfaces::msg::ClusterPickability result;
    result.total_tags = object_tag_indices.size();
    result.header = msg->header;
    result.cluster_id = 0;

    for (int a = 0; a < NUM_ARMS; ++a) {
      aruco_interfaces::msg::ArmAssignment aa;
      aa.arm_index = a;
      aa.assigned = false;
      result.arms[a] = aa;
    }

    AssignmentCandidate best;
    bool reused_latched_assignment = false;

    if (sticky_assignment) {
      std::vector<std::pair<int, int>> latched_pairs;
      if (build_latched_pairs(msg, index_by_tag_id, max_assign, latched_pairs)) {
        best.pairs = latched_pairs;
        best.matched_count = static_cast<int>(latched_pairs.size());
        reused_latched_assignment = true;
      }
    }

    if (!reused_latched_assignment) {
      best = compute_best_assignment(object_tag_indices, msg, max_assign);
      if (sticky_assignment) {
        if (!best.pairs.empty()) {
          latch_assignment(best, msg);
        } else {
          clear_latched_assignment();
        }
      }
    }

    for (const auto& [arm_idx, tag_idx] : best.pairs) {
      auto& aa = result.arms[arm_idx];
      aa.arm_index = arm_idx;
      aa.tag_id = msg->tags[tag_idx].tag_id;
      aa.assigned = true;
    }

    result.assigned_count = static_cast<uint8_t>(best.pairs.size());

    if (!best.pairs.empty()) {
      double tx, ty;
      std::vector<double> residuals;
      fit_rigid_correction(best.pairs, msg, tx, ty, residuals);

      result.correction.x = tx;
      result.correction.y = ty;
      result.correction.z = 0.0;
      result.correction_magnitude = static_cast<float>(dist2d(tx, ty));

      bool all_consistent = true;
      for (double r : residuals) {
        if (r > consist_thr) {
          all_consistent = false;
          break;
        }
      }
      result.is_pickable = all_consistent;
    } else {
      result.is_pickable = false;
      result.correction.x = 0.0;
      result.correction.y = 0.0;
      result.correction.z = 0.0;
      result.correction_magnitude = 0.0f;
    }

    pickability_pub_->publish(result);
  }

  // ── members ────────────────────────────────────────────────────────────────
  std::array<ArmPose, NUM_ARMS> arm_poses_;
  std::array<int, NUM_ARMS> latched_tag_ids_;
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
