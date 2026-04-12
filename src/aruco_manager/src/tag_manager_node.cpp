#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <aruco_interfaces/msg/detected_tag_array.hpp>
#include <aruco_interfaces/msg/cluster_pickability.hpp>
#include <aruco_interfaces/msg/arm_assignment.hpp>

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <deque>
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
  double max_residual = std::numeric_limits<double>::infinity();
  bool is_consistent = false;
  std::vector<std::pair<int, int>> pairs;
};

struct TrackedTag
{
  int track_id = -1;
  uint32_t tag_id = 0;
  double x = 0.0;
  double y = 0.0;
  rclcpp::Time last_seen;
  bool matched_in_frame = false;
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
    declare_parameter<double>("arm_x_origin",           0.0);   // x approach distance
    declare_parameter<double>("max_assignment_distance", 0.5); // m — ignore tag if farther
    declare_parameter<double>("consistency_threshold",   0.03); // m — max residual to be pickable
    declare_parameter<bool>("sticky_assignment",         false); // handler enables this only during pick window
    declare_parameter<double>("tracking_max_distance",   0.08); // m — association radius between frames
    declare_parameter<double>("tracking_timeout_sec",    0.6);  // s — drop stale tracks
    declare_parameter<double>("tracking_min_confidence", 0.6);  // [0,1] ignore unstable detections in tracker
    declare_parameter<double>("assignment_min_confidence", -1.0); // [0,1], <0 => use tracking_min_confidence
    declare_parameter<int>("assignment_switch_confirm_frames", 3); // frames — hysteresis before switching assignment
    declare_parameter<int>("sticky_loss_confirm_frames", 10); // frames — mark lost only after sustained disappearance
    declare_parameter<int>("sticky_stable_window_frames", 5); // frames — averaging window for frozen correction
    declare_parameter<double>("correction_x_bias", 0.05); // m — subtracted from tx to zero out systematic x error

    // Build arm poses
    double y0  = get_parameter("arm_y_origin").as_double();
    double x0  = get_parameter("arm_x_origin").as_double();
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
      "findeeznuts/detected_tags",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&TagManagerNode::tags_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "tag_manager_node started");

    latched_tag_ids_.fill(-1);
    latched_track_ids_.fill(-1);
  }

private:
  // ── helpers ────────────────────────────────────────────────────────────────
  double dist2d(double dx, double dy) { return std::sqrt(dx*dx + dy*dy); }

  void evaluate_candidate_geometry(
    const std::vector<std::pair<int, int>>& pairs,
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    double& out_max_residual,
    bool& out_is_consistent,
    double consist_thr)
  {
    if (pairs.empty()) {
      out_max_residual = std::numeric_limits<double>::infinity();
      out_is_consistent = false;
      return;
    }

    if (pairs.size() == 1) {
      out_max_residual = 0.0;
      out_is_consistent = true;
      return;
    }

    double sum_tx = 0.0;
    double sum_ty = 0.0;
    for (const auto& [arm_idx, tag_idx] : pairs) {
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      sum_tx += tp.x - arm_poses_[arm_idx].x;
      sum_ty += tp.y - arm_poses_[arm_idx].y;
    }

    const double tx = sum_tx / pairs.size();
    const double ty = sum_ty / pairs.size();

    out_max_residual = 0.0;
    for (const auto& [arm_idx, tag_idx] : pairs) {
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      const double rx = (tp.x - arm_poses_[arm_idx].x) - tx;
      const double ry = (tp.y - arm_poses_[arm_idx].y) - ty;
      out_max_residual = std::max(out_max_residual, dist2d(rx, ry));
    }

    out_is_consistent = (out_max_residual <= consist_thr);
  }

  void update_best_candidate(
    const std::vector<std::pair<int, int>>& current_pairs,
    double current_cost,
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    double consist_thr,
    AssignmentCandidate& best)
  {
    const int current_count = static_cast<int>(current_pairs.size());

    double current_max_residual = std::numeric_limits<double>::infinity();
    bool current_is_consistent = false;
    evaluate_candidate_geometry(
      current_pairs, msg, current_max_residual, current_is_consistent, consist_thr);

    const double eps = 1e-9;
    bool better = false;

    if (current_count > best.matched_count) {
      better = true;
    } else if (current_count == best.matched_count) {
      if (current_is_consistent != best.is_consistent) {
        better = current_is_consistent;
      } else if (current_max_residual + eps < best.max_residual) {
        better = true;
      } else if (std::abs(current_max_residual - best.max_residual) <= eps &&
                 current_cost + eps < best.total_cost) {
        better = true;
      }
    }

    if (better) {
      best.matched_count = current_count;
      best.total_cost = current_cost;
      best.max_residual = current_max_residual;
      best.is_consistent = current_is_consistent;
      best.pairs = current_pairs;
    }
  }

  void search_best_assignment(
    int arm_idx,
    const std::vector<size_t>& object_tag_indices,
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    double max_assign,
    double consist_thr,
    std::vector<bool>& used_tags,
    std::vector<std::pair<int, int>>& current_pairs,
    double current_cost,
    AssignmentCandidate& best)
  {
    if (arm_idx >= NUM_ARMS) {
      update_best_candidate(current_pairs, current_cost, msg, consist_thr, best);
      return;
    }

    // Option 1: leave this arm unused.
    search_best_assignment(
      arm_idx + 1,
      object_tag_indices,
      msg,
      max_assign,
      consist_thr,
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
        consist_thr,
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
    double max_assign,
    double consist_thr)
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
      consist_thr,
      used_tags,
      current_pairs,
      0.0,
      best);

    if (!std::isfinite(best.total_cost)) {
      best.total_cost = 0.0;
    }
    return best;
  }

  std::vector<int> update_tracks(
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    const std::vector<size_t>& object_tag_indices)
  {
    const double max_track_dist = get_parameter("tracking_max_distance").as_double();
    const double track_timeout_sec = get_parameter("tracking_timeout_sec").as_double();
    const double min_conf = get_parameter("tracking_min_confidence").as_double();
    const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(track_timeout_sec);
    const rclcpp::Time stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ? now() : rclcpp::Time(msg->header.stamp);

    for (auto& track : tracks_) {
      track.matched_in_frame = false;
    }

    std::vector<int> track_id_by_detection(msg->tags.size(), -1);

    for (size_t det_idx : object_tag_indices) {
      const auto& tag = msg->tags[det_idx];
      if (tag.confidence < min_conf) {
        continue;
      }
      const auto& tp = tag.tag_pose.position;

      int best_track_idx = -1;
      double best_dist = max_track_dist;

      for (size_t i = 0; i < tracks_.size(); ++i) {
        auto& track = tracks_[i];
        if (track.matched_in_frame || track.tag_id != tag.tag_id) {
          continue;
        }
        if ((stamp - track.last_seen) > timeout) {
          continue;
        }

        const double d = dist2d(tp.x - track.x, tp.y - track.y);
        if (d < best_dist) {
          best_dist = d;
          best_track_idx = static_cast<int>(i);
        }
      }

      if (best_track_idx >= 0) {
        auto& track = tracks_[best_track_idx];
        track.x = tp.x;
        track.y = tp.y;
        track.last_seen = stamp;
        track.matched_in_frame = true;
        track_id_by_detection[det_idx] = track.track_id;
      } else {
        TrackedTag new_track;
        new_track.track_id = next_track_id_++;
        new_track.tag_id = tag.tag_id;
        new_track.x = tp.x;
        new_track.y = tp.y;
        new_track.last_seen = stamp;
        new_track.matched_in_frame = true;
        track_id_by_detection[det_idx] = new_track.track_id;
        tracks_.push_back(new_track);
      }
    }

    tracks_.erase(
      std::remove_if(
        tracks_.begin(),
        tracks_.end(),
        [&](const TrackedTag& track) {
          return (stamp - track.last_seen) > timeout;
        }),
      tracks_.end());

    return track_id_by_detection;
  }

  bool build_latched_pairs(
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    const std::unordered_map<int, int>& index_by_track_id,
    double max_assign,
    std::vector<std::pair<int, int>>& pairs_out)
  {
    pairs_out.clear();
    std::unordered_set<int> used_detection_indices;

    bool has_any_assignment = false;
    for (int arm_idx = 0; arm_idx < NUM_ARMS; ++arm_idx) {
      const int latched_track_id = latched_track_ids_[arm_idx];
      if (latched_track_id < 0) {
        continue;
      }
      has_any_assignment = true;

      auto it = index_by_track_id.find(latched_track_id);
      if (it == index_by_track_id.end()) {
        return false;
      }

      const int tag_idx = it->second;
      if (used_detection_indices.count(tag_idx) > 0) {
        return false;
      }
      used_detection_indices.insert(tag_idx);

      if (latched_tag_ids_[arm_idx] >= 0 &&
          static_cast<int>(msg->tags[tag_idx].tag_id) != latched_tag_ids_[arm_idx]) {
        return false;
      }

      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      const double distance = dist2d(tp.x - arm_poses_[arm_idx].x, tp.y - arm_poses_[arm_idx].y);
      if (distance >= max_assign) {
        return false;
      }

      pairs_out.push_back({arm_idx, tag_idx});
    }

    return has_any_assignment;
  }

  void latch_assignment(
    const AssignmentCandidate& assignment,
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    const std::vector<int>& track_id_by_detection)
  {
    latched_tag_ids_.fill(-1);
    latched_track_ids_.fill(-1);
    for (const auto& [arm_idx, tag_idx] : assignment.pairs) {
      const int track_id = track_id_by_detection[tag_idx];
      if (track_id < 0) {
        continue;
      }
      latched_tag_ids_[arm_idx] = static_cast<int>(msg->tags[tag_idx].tag_id);
      latched_track_ids_[arm_idx] = track_id;
    }
  }

  void clear_latched_assignment()
  {
    latched_tag_ids_.fill(-1);
    latched_track_ids_.fill(-1);
  }

  void clear_stable_correction_history()
  {
    stable_correction_window_.clear();
    stable_correction_avg_.x = 0.0;
    stable_correction_avg_.y = 0.0;
    stable_correction_avg_.z = 0.0;
    has_stable_correction_ = false;
  }

  void push_stable_correction_sample(double x, double y, double theta)
  {
    int window_frames = get_parameter("sticky_stable_window_frames").as_int();
    if (window_frames < 1) {
      window_frames = 1;
    }

    geometry_msgs::msg::Vector3 sample;
    sample.x = x;
    sample.y = y;
    sample.z = theta;
    stable_correction_window_.push_back(sample);
    while (static_cast<int>(stable_correction_window_.size()) > window_frames) {
      stable_correction_window_.pop_front();
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_theta = 0.0;
    for (const auto& s : stable_correction_window_) {
      sum_x += s.x;
      sum_y += s.y;
      sum_theta += s.z;
    }
    const double inv_n = 1.0 / static_cast<double>(stable_correction_window_.size());
    stable_correction_avg_.x = sum_x * inv_n;
    stable_correction_avg_.y = sum_y * inv_n;
    stable_correction_avg_.z = sum_theta * inv_n;
    has_stable_correction_ = true;
  }

  std::array<int, NUM_ARMS> extract_assignment_track_ids(
    const AssignmentCandidate& assignment,
    const std::vector<int>& track_id_by_detection)
  {
    std::array<int, NUM_ARMS> track_ids;
    track_ids.fill(-1);
    for (const auto& [arm_idx, tag_idx] : assignment.pairs) {
      if (tag_idx >= 0 && static_cast<size_t>(tag_idx) < track_id_by_detection.size()) {
        track_ids[arm_idx] = track_id_by_detection[tag_idx];
      }
    }
    return track_ids;
  }

  bool same_track_assignment(
    const std::array<int, NUM_ARMS>& a,
    const std::array<int, NUM_ARMS>& b)
  {
    for (int i = 0; i < NUM_ARMS; ++i) {
      if (a[i] != b[i]) {
        return false;
      }
    }
    return true;
  }

  // Least-squares SE(2) rigid correction (tx, ty, theta).
  // Returns translation + rotation and per-pair residuals after applying the fit.
  void fit_rigid_correction(
    const std::vector<std::pair<int,int>>& pairs,   // (arm_index, tag_index)
    const aruco_interfaces::msg::DetectedTagArray::SharedPtr& msg,
    double& out_tx, double& out_ty, double& out_theta,
    std::vector<double>& out_residuals)
  {
    if (pairs.empty()) {
      out_tx = 0.0;
      out_ty = 0.0;
      out_theta = 0.0;
      out_residuals.clear();
      return;
    }

    // 1) Compute centroids of arm targets and observed tag points.
    double mean_ax = 0.0;
    double mean_ay = 0.0;
    double mean_tx = 0.0;
    double mean_ty = 0.0;
    for (const auto& [arm_idx, tag_idx] : pairs) {
      mean_ax += arm_poses_[arm_idx].x;
      mean_ay += arm_poses_[arm_idx].y;
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      mean_tx += tp.x;
      mean_ty += tp.y;
    }
    const double inv_n = 1.0 / static_cast<double>(pairs.size());
    mean_ax *= inv_n;
    mean_ay *= inv_n;
    mean_tx *= inv_n;
    mean_ty *= inv_n;

    // 2) Closed-form 2D rotation fit.
    double c_acc = 0.0;
    double s_acc = 0.0;
    for (const auto& [arm_idx, tag_idx] : pairs) {
      const double ax = arm_poses_[arm_idx].x - mean_ax;
      const double ay = arm_poses_[arm_idx].y - mean_ay;
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      const double tx = tp.x - mean_tx;
      const double ty = tp.y - mean_ty;

      c_acc += ax * tx + ay * ty;
      s_acc += ax * ty - ay * tx;
    }
    out_theta = std::atan2(s_acc, c_acc);

    const double ct = std::cos(out_theta);
    const double st = std::sin(out_theta);

    // 3) Translation aligning rotated arm centroid to observed centroid.
    out_tx = mean_tx - (ct * mean_ax - st * mean_ay);
    out_ty = mean_ty - (st * mean_ax + ct * mean_ay);

    // 4) Residual per pair after applying SE(2) correction.
    out_residuals.clear();
    for (const auto& [arm_idx, tag_idx] : pairs) {
      const auto& tp = msg->tags[tag_idx].tag_pose.position;
      const double pred_x = ct * arm_poses_[arm_idx].x - st * arm_poses_[arm_idx].y + out_tx;
      const double pred_y = st * arm_poses_[arm_idx].x + ct * arm_poses_[arm_idx].y + out_ty;
      const double rx = tp.x - pred_x;
      const double ry = tp.y - pred_y;
      out_residuals.push_back(dist2d(rx, ry));
    }
  }

  // ── main callback ──────────────────────────────────────────────────────────
  void tags_callback(const aruco_interfaces::msg::DetectedTagArray::SharedPtr msg)
  {
    double max_assign  = get_parameter("max_assignment_distance").as_double();
    double consist_thr = get_parameter("consistency_threshold").as_double();
    double tracking_min_conf = get_parameter("tracking_min_confidence").as_double();
    double assignment_min_conf = get_parameter("assignment_min_confidence").as_double();
    if (assignment_min_conf < 0.0) {
      assignment_min_conf = tracking_min_conf;
    }
    bool sticky_assignment = get_parameter("sticky_assignment").as_bool();
    int switch_confirm_frames = get_parameter("assignment_switch_confirm_frames").as_int();
    if (switch_confirm_frames < 1) {
      switch_confirm_frames = 1;
    }
    int sticky_loss_confirm_frames = get_parameter("sticky_loss_confirm_frames").as_int();
    if (sticky_loss_confirm_frames < 1) {
      sticky_loss_confirm_frames = 1;
    }

    if (sticky_assignment && !sticky_mode_prev_) {
      sticky_locked_assignment_ = false;
      sticky_missing_frames_ = 0;
      clear_stable_correction_history();
      RCLCPP_INFO(get_logger(), "Sticky assignment enabled: locking assignment for pick window");
    } else if (!sticky_assignment && sticky_mode_prev_) {
      sticky_locked_assignment_ = false;
      sticky_missing_frames_ = 0;
      clear_latched_assignment();
      clear_stable_correction_history();
      RCLCPP_INFO(get_logger(), "Sticky assignment disabled: returning to dynamic assignment");
    }
    sticky_mode_prev_ = sticky_assignment;

    // Split tags
    aruco_interfaces::msg::DetectedTagArray loc_array;
    loc_array.header = msg->header;

    // Collect all visible object tags for one whole-frame pickup opportunity.
    std::vector<size_t> object_tag_indices;
    std::vector<size_t> assignment_object_tag_indices;
    for (size_t i = 0; i < msg->tags.size(); ++i) {
      const auto& tag = msg->tags[i];
      if (object_ids_.count(tag.tag_id)) {
        object_tag_indices.push_back(i);
        if (tag.confidence >= assignment_min_conf) {
          assignment_object_tag_indices.push_back(i);
        }
      } else if (localization_ids_.count(tag.tag_id)) {
        loc_array.tags.push_back(tag);
      }
    }

    const auto track_id_by_detection = update_tracks(msg, object_tag_indices);
    std::unordered_map<int, int> index_by_track_id;
    for (size_t det_idx : object_tag_indices) {
      const int track_id = track_id_by_detection[det_idx];
      if (track_id >= 0) {
        index_by_track_id[track_id] = static_cast<int>(det_idx);
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
      aa.track_id = -1;
      aa.assigned = false;
      result.arms[a] = aa;
    }

    AssignmentCandidate best;

    if (sticky_assignment) {
      if (!sticky_locked_assignment_) {
        std::vector<std::pair<int, int>> current_latched_pairs;
        if (build_latched_pairs(msg, index_by_track_id, max_assign, current_latched_pairs) &&
            !current_latched_pairs.empty()) {
          best.pairs = current_latched_pairs;
          best.matched_count = static_cast<int>(current_latched_pairs.size());
          sticky_locked_assignment_ = true;
          sticky_missing_frames_ = 0;
        } else {
          AssignmentCandidate instant_best =
            compute_best_assignment(assignment_object_tag_indices, msg, max_assign, consist_thr);
          best = instant_best;
          if (!best.pairs.empty()) {
            latch_assignment(best, msg, track_id_by_detection);
            sticky_locked_assignment_ = true;
            sticky_missing_frames_ = 0;
          } else {
            clear_latched_assignment();
            sticky_locked_assignment_ = false;
          }
        }
        pending_switch_track_ids_.fill(-1);
        pending_switch_frames_ = 0;
      } else {
        std::vector<std::pair<int, int>> locked_pairs;
        if (build_latched_pairs(msg, index_by_track_id, max_assign, locked_pairs)) {
          best.pairs = locked_pairs;
          best.matched_count = static_cast<int>(locked_pairs.size());
          sticky_missing_frames_ = 0;
        } else {
          sticky_missing_frames_ += 1;
        }
        pending_switch_track_ids_.fill(-1);
        pending_switch_frames_ = 0;
      }
    } else {
      sticky_missing_frames_ = 0;
      sticky_locked_assignment_ = false;

      AssignmentCandidate instant_best =
        compute_best_assignment(assignment_object_tag_indices, msg, max_assign, consist_thr);

      std::vector<std::pair<int, int>> active_pairs;
      const bool has_active_assignment =
        build_latched_pairs(msg, index_by_track_id, max_assign, active_pairs);

      if (!has_active_assignment) {
        best = instant_best;
        if (!best.pairs.empty()) {
          latch_assignment(best, msg, track_id_by_detection);
        } else {
          clear_latched_assignment();
        }
        pending_switch_track_ids_.fill(-1);
        pending_switch_frames_ = 0;
      } else {
        AssignmentCandidate active_best;
        active_best.pairs = active_pairs;
        active_best.matched_count = static_cast<int>(active_pairs.size());

        const auto instant_tracks =
          extract_assignment_track_ids(instant_best, track_id_by_detection);
        const auto active_tracks =
          extract_assignment_track_ids(active_best, track_id_by_detection);

        if (same_track_assignment(instant_tracks, active_tracks)) {
          best = active_best;
          pending_switch_track_ids_.fill(-1);
          pending_switch_frames_ = 0;
        } else {
          bool switch_preferred = false;
          if (instant_best.matched_count > active_best.matched_count) {
            switch_preferred = true;
          } else if (instant_best.matched_count == active_best.matched_count) {
            double instant_max_residual = std::numeric_limits<double>::infinity();
            bool instant_consistent = false;
            evaluate_candidate_geometry(
              instant_best.pairs, msg, instant_max_residual, instant_consistent, consist_thr);

            double active_max_residual = std::numeric_limits<double>::infinity();
            bool active_consistent = false;
            evaluate_candidate_geometry(
              active_best.pairs, msg, active_max_residual, active_consistent, consist_thr);

            const double eps = 1e-9;
            if (instant_consistent != active_consistent) {
              switch_preferred = instant_consistent;
            } else if (instant_max_residual + eps < active_max_residual) {
              switch_preferred = true;
            }
          }

          if (switch_preferred && !instant_best.pairs.empty()) {
            if (same_track_assignment(instant_tracks, pending_switch_track_ids_)) {
              pending_switch_frames_ += 1;
            } else {
              pending_switch_track_ids_ = instant_tracks;
              pending_switch_frames_ = 1;
            }

            if (pending_switch_frames_ >= switch_confirm_frames) {
              best = instant_best;
              latch_assignment(best, msg, track_id_by_detection);
              pending_switch_track_ids_.fill(-1);
              pending_switch_frames_ = 0;
            } else {
              best = active_best;
            }
          } else {
            best = active_best;
            pending_switch_track_ids_.fill(-1);
            pending_switch_frames_ = 0;
          }
        }
      }
    }

    const bool cluster_lost =
      sticky_assignment && sticky_locked_assignment_ &&
      (sticky_missing_frames_ > sticky_loss_confirm_frames);
    result.sticky_active = sticky_assignment;
    result.cluster_lost = cluster_lost;
    result.lost_tracking_frames = sticky_assignment ?
      static_cast<uint16_t>(std::min(sticky_missing_frames_, 65535)) : 0;

    for (const auto& [arm_idx, tag_idx] : best.pairs) {
      auto& aa = result.arms[arm_idx];
      aa.arm_index = arm_idx;
      aa.tag_id = msg->tags[tag_idx].tag_id;
      aa.tag_pose = msg->tags[tag_idx].tag_pose;
      if (tag_idx >= 0 && static_cast<size_t>(tag_idx) < track_id_by_detection.size()) {
        aa.track_id = track_id_by_detection[tag_idx];
      } else {
        aa.track_id = -1;
      }
      aa.assigned = true;
    }

    result.assigned_count = static_cast<uint8_t>(best.pairs.size());

    if (!best.pairs.empty()) {
      double tx, ty, theta;
      std::vector<double> residuals;
      fit_rigid_correction(best.pairs, msg, tx, ty, theta, residuals);

      double x_bias = get_parameter("correction_x_bias").as_double();
      result.correction.x = tx + x_bias;
      result.correction.y = ty;
      result.correction.z = theta;
      result.correction_magnitude =
        static_cast<float>(dist2d(result.correction.x, result.correction.y));

      bool all_consistent = true;
      for (double r : residuals) {
        if (r > consist_thr) {
          all_consistent = false;
          break;
        }
      }
      result.is_pickable = all_consistent;
      if (all_consistent) {
        push_stable_correction_sample(result.correction.x, result.correction.y, result.correction.z);
      }
    } else {
      result.is_pickable = false;
      if (sticky_assignment && has_stable_correction_) {
        result.correction = stable_correction_avg_;
        result.correction_magnitude =
          static_cast<float>(dist2d(result.correction.x, result.correction.y));
      } else {
        result.correction.x = 0.0;
        result.correction.y = 0.0;
        result.correction.z = 0.0;
        result.correction_magnitude = 0.0f;
      }
    }

    pickability_pub_->publish(result);
  }

  // ── members ────────────────────────────────────────────────────────────────
  std::array<ArmPose, NUM_ARMS> arm_poses_;
  std::array<int, NUM_ARMS> latched_tag_ids_;
  std::array<int, NUM_ARMS> latched_track_ids_;
  std::array<int, NUM_ARMS> pending_switch_track_ids_{{-1, -1, -1, -1}};
  int pending_switch_frames_ = 0;
  bool sticky_mode_prev_ = false;
  bool sticky_locked_assignment_ = false;
  int sticky_missing_frames_ = 0;
  std::deque<geometry_msgs::msg::Vector3> stable_correction_window_;
  geometry_msgs::msg::Vector3 stable_correction_avg_;
  bool has_stable_correction_ = false;
  std::unordered_set<uint32_t>  object_ids_;
  std::unordered_set<uint32_t>  localization_ids_;
  std::vector<TrackedTag> tracks_;
  int next_track_id_ = 0;

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
