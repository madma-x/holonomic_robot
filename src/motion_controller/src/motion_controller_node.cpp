#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "aruco_interfaces/msg/cluster_pickability.hpp"
#include "aruco_interfaces/srv/align_to_cluster.hpp"
#include "aruco_interfaces/srv/move_relative.hpp"
#include "pid_controller.hpp"
#include <cmath>
#include <mutex>
#include <chrono>
#include <thread>

// Session state for unified motion controller
enum class SessionState {
    IDLE,           // No active motion request
    ALIGN_ACTIVE,   // Visual servo alignment in progress
    REL_MOVE_ACTIVE, // Relative move (odometry-based) in progress
    STOPPING,       // Transitioning to stop
    FAILED          // Session failed or aborted
};

class AlignmentNode : public rclcpp::Node {
public:
    AlignmentNode() : Node("motion_controller_node") {
        // Declare parameters
        this->declare_parameter<double>("control_loop_rate", 50.0);
        this->declare_parameter<int>("cluster_timeout_ms", 500);
        this->declare_parameter<double>("pid_xy.kp", 0.5);
        this->declare_parameter<double>("pid_xy.ki", 0.05);
        this->declare_parameter<double>("pid_xy.kd", 0.1);
        this->declare_parameter<double>("pid_xy.max_integral_error", 0.5);
        this->declare_parameter<double>("pid_xy.max_velocity", 0.1);
        this->declare_parameter<double>("pid_theta.kp", 0.0);
        this->declare_parameter<double>("pid_theta.ki", 0.0);
        this->declare_parameter<double>("pid_theta.kd", 0.0);
        this->declare_parameter<double>("pid_theta.max_integral_error", 0.3);
        this->declare_parameter<double>("pid_theta.max_angular_velocity", 0.0);
        this->declare_parameter<int>("final_stop_hold_ms", 1000);
        this->declare_parameter<double>("pos_tolerance", 0.02);
        this->declare_parameter<double>("ang_tolerance", 0.05);
        this->declare_parameter<bool>("verbose_logging", false);
        this->declare_parameter<std::string>("cluster_topic", "/aruco_manager/cluster_pickability");
        this->declare_parameter<std::string>("odometry_topic", "/odom");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        // Get parameters
        double control_rate = this->get_parameter("control_loop_rate").as_double();
        cluster_timeout_ms_ = this->get_parameter("cluster_timeout_ms").as_int();
        pos_tolerance_ = this->get_parameter("pos_tolerance").as_double();
        ang_tolerance_ = this->get_parameter("ang_tolerance").as_double();
        verbose_logging_ = this->get_parameter("verbose_logging").as_bool();
        final_stop_hold_ms_ = this->get_parameter("final_stop_hold_ms").as_int();
        if (final_stop_hold_ms_ < 0) {
            final_stop_hold_ms_ = 0;
        }
        stop_hold_until_ = this->now();

        // Create subscriptions
        std::string cluster_topic = this->get_parameter("cluster_topic").as_string();
        std::string odom_topic = this->get_parameter("odometry_topic").as_string();

        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        subs_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions cluster_sub_options;
        cluster_sub_options.callback_group = subs_cb_group_;
        cluster_sub_ = this->create_subscription<aruco_interfaces::msg::ClusterPickability>(
            cluster_topic, 10,
            [this](const aruco_interfaces::msg::ClusterPickability::SharedPtr msg) {
                this->cluster_callback(msg);
            },
            cluster_sub_options);

        rclcpp::SubscriptionOptions odom_sub_options;
        odom_sub_options.callback_group = subs_cb_group_;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odom_callback(msg);
            },
            odom_sub_options);

        // Create publisher and service
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);

        service_ = this->create_service<aruco_interfaces::srv::AlignToCluster>(
            "align_to_cluster",
            [this](const std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Request> request,
                   std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Response> response) {
                this->align_service_callback(request, response);
            },
            rclcpp::QoS(10),
            service_cb_group_);

        move_relative_service_ = this->create_service<aruco_interfaces::srv::MoveRelative>(
            "move_relative",
            [this](const std::shared_ptr<aruco_interfaces::srv::MoveRelative::Request> request,
                   std::shared_ptr<aruco_interfaces::srv::MoveRelative::Response> response) {
                this->move_relative_service_callback(request, response);
            },
            rclcpp::QoS(10),
            service_cb_group_);

        // Initialize PID controllers
        init_pid_controllers();

        RCLCPP_INFO(this->get_logger(),
            "Loaded PID params: pid_xy(kp=%.3f, ki=%.3f, kd=%.3f), pid_theta(kp=%.3f, ki=%.3f, kd=%.3f)",
            this->get_parameter("pid_xy.kp").as_double(),
            this->get_parameter("pid_xy.ki").as_double(),
            this->get_parameter("pid_xy.kd").as_double(),
            this->get_parameter("pid_theta.kp").as_double(),
            this->get_parameter("pid_theta.ki").as_double(),
            this->get_parameter("pid_theta.kd").as_double());

        // Create control loop timer
        auto period = std::chrono::duration<double>(1.0 / control_rate);
        timer_ = this->create_wall_timer(
            period,
            [this]() { this->control_loop(); },
            timer_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Motion controller node initialized with dual APIs");
    }

private:
    // Subscriptions and publishers
    rclcpp::Subscription<aruco_interfaces::msg::ClusterPickability>::SharedPtr cluster_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<aruco_interfaces::srv::AlignToCluster>::SharedPtr service_;
    rclcpp::Service<aruco_interfaces::srv::MoveRelative>::SharedPtr move_relative_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr subs_cb_group_;

    // Session state machine
    SessionState session_state_ = SessionState::IDLE;
    int32_t target_cluster_id_ = -1;
    rclcpp::Time session_start_time_;
    bool session_complete_ = false;
    bool session_success_ = false;
    std::string session_message_;

    // Relative move state (used when session_state_ == REL_MOVE_ACTIVE)
    struct RelativeMoveTarget {
        double target_x_m = 0.0;
        double target_y_m = 0.0;
        double target_yaw_rad = 0.0;
        double start_x_m = 0.0;
        double start_y_m = 0.0;
        double start_yaw_rad = 0.0;
        double pos_tolerance_m = 0.01;
        double yaw_tolerance_rad = 0.05;
        float timeout_s = 5.0;
        float max_linear_speed_mps = 0.1;
        float max_angular_speed_rps = 1.0;
    } rel_move_target_;

    // Cluster state
    struct ClusterState {
        aruco_interfaces::msg::ClusterPickability msg;
        rclcpp::Time timestamp;
        bool received = false;
    } latest_cluster_;

    struct PoseXY {
        double x, y, theta;
    };
    PoseXY last_confident_pose_{0.0, 0.0, 0.0};
    bool have_confident_pose_ = false;
    PoseXY frozen_loss_target_{0.0, 0.0, 0.0};
    PoseXY frozen_loss_target_world_{0.0, 0.0, 0.0};
    bool frozen_loss_mode_ = false;

    // Robot state (from odometry)
    struct RobotState {
        double x, y, theta;
    } robot_state_{0.0, 0.0, 0.0};

    // Configuration
    int cluster_timeout_ms_;
    int final_stop_hold_ms_;
    double pos_tolerance_, ang_tolerance_;
    bool verbose_logging_ = false;
    rclcpp::Time stop_hold_until_;

    // Independent PID controllers per axis
    std::unique_ptr<PIDController> pid_x_;     // X position error → vx command
    std::unique_ptr<PIDController> pid_y_;     // Y position error → vy command
    std::unique_ptr<PIDController> pid_theta_; // angle error → angular velocity command

    // Synchronization
    std::mutex state_mutex_;

    void init_pid_controllers() {
        // X and Y use the same gains/limits but are independent instances.
        PIDController::Gains gains_xy = {
            this->get_parameter("pid_xy.kp").as_double(),
            this->get_parameter("pid_xy.ki").as_double(),
            this->get_parameter("pid_xy.kd").as_double()
        };
        PIDController::Limits limits_xy = {
            this->get_parameter("pid_xy.max_integral_error").as_double(),
            this->get_parameter("pid_xy.max_velocity").as_double()
        };
        pid_x_ = std::make_unique<PIDController>(gains_xy, limits_xy);
        pid_y_ = std::make_unique<PIDController>(gains_xy, limits_xy);

        // Theta angle loop: angle error → angular velocity output
        PIDController::Gains gains_theta = {
            this->get_parameter("pid_theta.kp").as_double(),
            this->get_parameter("pid_theta.ki").as_double(),
            this->get_parameter("pid_theta.kd").as_double()
        };
        PIDController::Limits limits_theta = {
            this->get_parameter("pid_theta.max_integral_error").as_double(),
            this->get_parameter("pid_theta.max_angular_velocity").as_double()
        };
        pid_theta_ = std::make_unique<PIDController>(gains_theta, limits_theta);
    }

    void cluster_callback(const aruco_interfaces::msg::ClusterPickability::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // Ignore updates from unrelated clusters while an alignment target is active.
        if (target_cluster_id_ >= 0 && msg->cluster_id != target_cluster_id_) {
            return;
        }

        latest_cluster_.msg = *msg;
        latest_cluster_.timestamp = this->now();
        latest_cluster_.received = true;

        if (msg->sticky_active && msg->cluster_lost) {
            if (!frozen_loss_mode_) {
                // Pickability axes are swapped w.r.t. robot state axes.
                frozen_loss_target_.x = msg->correction.y;
                frozen_loss_target_.y = msg->correction.x;
                frozen_loss_target_.theta = msg->correction.z;

                // Freeze a world-frame target once: current odom + last error.
                frozen_loss_target_world_.x = robot_state_.x + frozen_loss_target_.x;
                frozen_loss_target_world_.y = robot_state_.y + frozen_loss_target_.y;
                frozen_loss_target_world_.theta = robot_state_.theta + frozen_loss_target_.theta;
                while (frozen_loss_target_world_.theta > M_PI) frozen_loss_target_world_.theta -= 2.0 * M_PI;
                while (frozen_loss_target_world_.theta < -M_PI) frozen_loss_target_world_.theta += 2.0 * M_PI;
                frozen_loss_mode_ = true;
                last_confident_pose_ = frozen_loss_target_;
                have_confident_pose_ = true;
                if (verbose_logging_) {
                    RCLCPP_WARN(this->get_logger(),
                        "Sticky cluster lost (%u frames), freezing correction target",
                        static_cast<unsigned int>(msg->lost_tracking_frames));
                }
            }
            return;
        }

        frozen_loss_mode_ = false;

        // Update last confident pose when cluster is seen and is pickable
        if (msg->is_pickable) {
            // Pickability axes are swapped w.r.t. robot state axes.
            last_confident_pose_.x = msg->correction.y;
            last_confident_pose_.y = msg->correction.x;
            last_confident_pose_.theta = msg->correction.z;
            have_confident_pose_ = true;
            if (verbose_logging_) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Cluster %d received: correction=(%.3f, %.3f), magnitude=%.3f",
                    msg->cluster_id, last_confident_pose_.x, last_confident_pose_.y, msg->correction_magnitude);
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        robot_state_.x = msg->pose.pose.position.x;
        robot_state_.y = msg->pose.pose.position.y;
        
        // Extract theta from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_state_.theta = std::atan2(2.0 * (qw * qz + qx * qy),
                                         1.0 - 2.0 * (qy * qy + qz * qz));
    }

    void align_service_callback(
        const std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Request> request,
        std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Response> response) {

        std::unique_lock<std::mutex> lock(state_mutex_);

        // Reject if session already active
        if (session_state_ != SessionState::IDLE) {
            response->success = false;
            response->status_message = "Motion controller busy with existing session";
            RCLCPP_WARN(this->get_logger(),
                "Align service called while session active (state=%d)", static_cast<int>(session_state_));
            return;
        }

        // Start alignment session
        target_cluster_id_ = request->cluster_id;
        session_state_ = SessionState::ALIGN_ACTIVE;
        session_complete_ = false;
        session_success_ = false;
        stop_hold_until_ = this->now();
        
        // Keep a recent matching cluster sample if available
        if (latest_cluster_.received && latest_cluster_.msg.cluster_id != request->cluster_id) {
            latest_cluster_.received = false;
        }
        frozen_loss_mode_ = false;
        session_start_time_ = this->now();

        // Reset PID controllers
        pid_x_->reset();
        pid_y_->reset();
        pid_theta_->reset();

        RCLCPP_INFO(this->get_logger(),
            "Alignment session started: cluster_id=%d, threshold=%.4f, max_wait=%.2fs",
            request->cluster_id, request->alignment_threshold, request->max_wait_time);

        // Wait for alignment to complete (blocking)
        auto wait_timeout = rclcpp::Duration::from_seconds(
            request->max_wait_time > 0 ? request->max_wait_time : 60.0);
        auto start_time = this->now();

        while (!session_complete_) {
            if (request->max_wait_time > 0 &&
                (this->now() - start_time) > wait_timeout) {
                session_complete_ = true;
                session_success_ = false;
                session_message_ = "Timeout waiting for alignment";
                RCLCPP_ERROR(this->get_logger(),
                    "Alignment timed out after %.2fs (cluster_id=%d)",
                    request->max_wait_time, request->cluster_id);
                break;
            }
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            lock.lock();
        }

        // Populate response
        response->success = session_success_;
        response->final_correction.x = last_confident_pose_.x;
        response->final_correction.y = last_confident_pose_.y;
        response->final_correction.z = last_confident_pose_.theta;
        response->final_magnitude = std::hypot(last_confident_pose_.x, last_confident_pose_.y);
        response->status_message = session_message_;

        // Transition to stopping state
        session_state_ = SessionState::STOPPING;
        stop_hold_until_ = this->now() + rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(final_stop_hold_ms_) * 1000000LL);

        // Always send a final stop command
        auto stop_cmd = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(stop_cmd);

        RCLCPP_INFO(this->get_logger(),
            "Alignment session complete: success=%d, message=%s",
            session_success_, session_message_.c_str());

        // Return to idle after response
        session_state_ = SessionState::IDLE;
    }

    void move_relative_service_callback(
        const std::shared_ptr<aruco_interfaces::srv::MoveRelative::Request> request,
        std::shared_ptr<aruco_interfaces::srv::MoveRelative::Response> response) {

        std::unique_lock<std::mutex> lock(state_mutex_);

        // Reject if session already active
        if (session_state_ != SessionState::IDLE) {
            response->success = false;
            response->residual_distance_m = 0.0f;
            response->residual_yaw_rad = 0.0f;
            response->status_message = "Motion controller busy with existing session";
            RCLCPP_WARN(this->get_logger(),
                "MoveRelative service called while session active (state=%d)", static_cast<int>(session_state_));
            return;
        }

        // Start relative move session
        session_state_ = SessionState::REL_MOVE_ACTIVE;
        session_complete_ = false;
        session_success_ = false;
        session_start_time_ = this->now();

        // Store relative move targets
        rel_move_target_.target_x_m = request->target_x_m;
        rel_move_target_.target_y_m = request->target_y_m;
        rel_move_target_.target_yaw_rad = request->target_yaw_rad;
        rel_move_target_.start_x_m = robot_state_.x;
        rel_move_target_.start_y_m = robot_state_.y;
        rel_move_target_.start_yaw_rad = robot_state_.theta;
        rel_move_target_.pos_tolerance_m = request->pos_tolerance_m;
        rel_move_target_.yaw_tolerance_rad = request->yaw_tolerance_rad;
        rel_move_target_.timeout_s = request->timeout_s;
        rel_move_target_.max_linear_speed_mps = request->max_linear_speed_mps;
        rel_move_target_.max_angular_speed_rps = request->max_angular_speed_rps;

        // Reset PID controllers
        pid_x_->reset();
        pid_y_->reset();
        pid_theta_->reset();

        RCLCPP_INFO(this->get_logger(),
            "Relative move session started: target=(%.3f, %.3f, %.3f), timeout=%.2fs",
            request->target_x_m, request->target_y_m, request->target_yaw_rad, request->timeout_s);

        // Wait for move to complete (blocking)
        auto wait_timeout = rclcpp::Duration::from_seconds(
            request->timeout_s > 0 ? request->timeout_s : 60.0);
        auto start_time = this->now();

        while (!session_complete_) {
            if (request->timeout_s > 0 &&
                (this->now() - start_time) > wait_timeout) {
                session_complete_ = true;
                session_success_ = false;
                session_message_ = "Timeout waiting for relative move";
                RCLCPP_ERROR(this->get_logger(),
                    "Relative move timed out after %.2fs", request->timeout_s);
                break;
            }
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            lock.lock();
        }

        // Populate response
        double final_x = robot_state_.x - rel_move_target_.start_x_m;
        double final_y = robot_state_.y - rel_move_target_.start_y_m;
        double final_yaw = robot_state_.theta - rel_move_target_.start_yaw_rad;
        while (final_yaw > M_PI) final_yaw -= 2.0 * M_PI;
        while (final_yaw < -M_PI) final_yaw += 2.0 * M_PI;

        response->success = session_success_;
        response->final_x_m = final_x;
        response->final_y_m = final_y;
        response->final_yaw_rad = final_yaw;
        response->residual_distance_m = std::hypot(
            rel_move_target_.target_x_m - final_x,
            rel_move_target_.target_y_m - final_y);
        response->residual_yaw_rad = rel_move_target_.target_yaw_rad - final_yaw;
        response->status_message = session_message_;

        // Transition to stopping state
        session_state_ = SessionState::STOPPING;
        stop_hold_until_ = this->now() + rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(final_stop_hold_ms_) * 1000000LL);

        // Always send a final stop command
        auto stop_cmd = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(stop_cmd);

        RCLCPP_INFO(this->get_logger(),
            "Relative move session complete: success=%d, traveled=(%.3f, %.3f, %.3f), message=%s",
            session_success_, final_x, final_y, final_yaw, session_message_.c_str());

        // Return to idle after response
        session_state_ = SessionState::IDLE;
    }

    void control_loop() {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // Handle post-stop hold period
        if (session_state_ == SessionState::STOPPING && this->now() < stop_hold_until_) {
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
            if (verbose_logging_) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "Publishing stop cmd_vel during final hold window");
            }
            return;
        }

        // If returning to idle after stop hold
        if (session_state_ == SessionState::STOPPING && this->now() >= stop_hold_until_) {
            session_state_ = SessionState::IDLE;
        }

        // No active goal: do not publish cmd_vel.
        if (session_state_ == SessionState::IDLE) {
            return;
        }

        // ========== Alignment Session (Visual Servo) ==========
        if (session_state_ == SessionState::ALIGN_ACTIVE) {
            control_loop_alignment();
            return;
        }

        // ========== Relative Move Session (Odometry-based) ==========
        if (session_state_ == SessionState::REL_MOVE_ACTIVE) {
            control_loop_relative_move();
            return;
        }
    }

    void control_loop_alignment() {
        // Check cluster timeout
        bool cluster_available = false;
        PoseXY error_pose{0.0, 0.0, 0.0};

        if (frozen_loss_mode_) {
            // In loss mode, compute fresh delta error against frozen world target.
            error_pose.x = frozen_loss_target_world_.x - robot_state_.x;
            error_pose.y = frozen_loss_target_world_.y - robot_state_.y;
            error_pose.theta = frozen_loss_target_world_.theta - robot_state_.theta;
            while (error_pose.theta > M_PI) error_pose.theta -= 2.0 * M_PI;
            while (error_pose.theta < -M_PI) error_pose.theta += 2.0 * M_PI;
            cluster_available = true;
            if (verbose_logging_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Control loop using frozen loss target while sticky cluster is lost");
            }
        } else if (latest_cluster_.received) {
            auto age_ms = (this->now() - latest_cluster_.timestamp).nanoseconds() / 1e6;
            if (age_ms < cluster_timeout_ms_) {
                // Correction is robot-frame error with swapped X/Y axes from pickability.
                cluster_available = true;
                error_pose.x = latest_cluster_.msg.correction.y;
                error_pose.y = latest_cluster_.msg.correction.x;
                error_pose.theta = latest_cluster_.msg.correction.z;
                if (verbose_logging_) {
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Control loop using live cluster correction (age=%.1f ms)", age_ms);
                }
            } else if (verbose_logging_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Latest cluster stale (age=%.1f ms, timeout=%d ms)", age_ms, cluster_timeout_ms_);
            }
        } else {
            // Grace period at session startup: wait briefly for first target
            auto startup_ms = (this->now() - session_start_time_).nanoseconds() / 1e6;
            if (startup_ms < cluster_timeout_ms_) {
                auto stop_cmd = geometry_msgs::msg::Twist();
                cmd_vel_pub_->publish(stop_cmd);
                if (verbose_logging_) {
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "Waiting for first cluster sample after session start (%.1f ms)", startup_ms);
                }
                return;
            } else if (verbose_logging_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "No cluster message received yet while alignment is active");
            }
        }

        // If cluster not available, use last confident pose
        if (!cluster_available && have_confident_pose_) {
            error_pose = last_confident_pose_;
            if (verbose_logging_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Cluster lost, aligning with last confident error");
            }
        } else if (!cluster_available && !have_confident_pose_) {
            // No target available
            session_complete_ = true;
            session_success_ = false;
            session_message_ = "No cluster available and no last confident pose";
            RCLCPP_ERROR(this->get_logger(),
                "Alignment failed: no cluster available and no last confident pose");
            return;
        }

        // Use correction directly as robot-frame error.
        const double error_x = error_pose.x;
        const double error_y = error_pose.y;

        // Check convergence
        double pos_error = std::hypot(error_x, error_y);
        if (pos_error < pos_tolerance_) {
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(cmd_msg);
            session_complete_ = true;
            session_success_ = true;
            session_message_ = "Alignment successful";
            if (verbose_logging_) {
                RCLCPP_INFO(this->get_logger(),
                    "Alignment converged: pos_err=%.4f (tol=%.4f)",
                    pos_error, pos_tolerance_);
            }
            return;
        }

        // Control loop timing
        static rclcpp::Time last_loop_time = this->now();
        double dt = (this->now() - last_loop_time).nanoseconds() / 1e9;
        last_loop_time = this->now();
        if (dt <= 0.0) dt = 0.02; // Fallback

        // Invert XY command direction so positive correction error commands
        // motion that reduces the observed robot-frame offset.
        double cmd_vx = pid_x_->update(-error_x, dt);
        double cmd_vy = pid_y_->update(-error_y, dt);
        double cmd_omega = 0.0; // yaw control disabled

        // Publish command
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = cmd_vx;
        cmd_msg.linear.y = cmd_vy;
        cmd_msg.angular.z = cmd_omega;
        cmd_vel_pub_->publish(cmd_msg);

        if (verbose_logging_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Alignment control: pos_err=(%.4f, %.4f), cmd_vel=(%.3f, %.3f, %.3f)",
                error_x, error_y, cmd_vx, cmd_vy, cmd_omega);
        }
    }

    void control_loop_relative_move() {
        // Compute current error relative to session start + target delta
        double target_world_x = rel_move_target_.start_x_m + rel_move_target_.target_x_m;
        double target_world_y = rel_move_target_.start_y_m + rel_move_target_.target_y_m;
        double target_world_theta = rel_move_target_.start_yaw_rad + rel_move_target_.target_yaw_rad;
        while (target_world_theta > M_PI) target_world_theta -= 2.0 * M_PI;
        while (target_world_theta < -M_PI) target_world_theta += 2.0 * M_PI;

        // Error in world frame
        double error_x_world = target_world_x - robot_state_.x;
        double error_y_world = target_world_y - robot_state_.y;
        double error_theta_world = target_world_theta - robot_state_.theta;
        while (error_theta_world > M_PI) error_theta_world -= 2.0 * M_PI;
        while (error_theta_world < -M_PI) error_theta_world += 2.0 * M_PI;

        // Check convergence
        double pos_error = std::hypot(error_x_world, error_y_world);
        double ang_error = std::abs(error_theta_world);
        if (pos_error < rel_move_target_.pos_tolerance_m &&
            ang_error < rel_move_target_.yaw_tolerance_rad) {
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(cmd_msg);
            session_complete_ = true;
            session_success_ = true;
            session_message_ = "Relative move successful";
            if (verbose_logging_) {
                RCLCPP_INFO(this->get_logger(),
                    "Relative move converged: pos_err=%.4f, yaw_err=%.4f",
                    pos_error, ang_error);
            }
            return;
        }

        // Control loop timing
        static rclcpp::Time last_loop_time = this->now();
        double dt = (this->now() - last_loop_time).nanoseconds() / 1e9;
        last_loop_time = this->now();
        if (dt <= 0.0) dt = 0.02; // Fallback

        // Convert world-frame errors to robot frame for holonomic control
        double cos_theta = std::cos(robot_state_.theta);
        double sin_theta = std::sin(robot_state_.theta);
        double error_x_robot = error_x_world * cos_theta + error_y_world * sin_theta;
        double error_y_robot = -error_x_world * sin_theta + error_y_world * cos_theta;

        // Compute velocity commands with velocity scaling
        double cmd_vx = pid_x_->update(error_x_robot, dt);
        double cmd_vy = pid_y_->update(error_y_robot, dt);
        double cmd_omega = pid_theta_->update(error_theta_world, dt);

        // Apply velocity limits from request
        double linear_speed = std::hypot(cmd_vx, cmd_vy);
        if (linear_speed > rel_move_target_.max_linear_speed_mps &&
            linear_speed > 1e-6) {
            double scale = rel_move_target_.max_linear_speed_mps / linear_speed;
            cmd_vx *= scale;
            cmd_vy *= scale;
        }
        cmd_omega = std::clamp(cmd_omega, 
                               -static_cast<double>(rel_move_target_.max_angular_speed_rps),
                               static_cast<double>(rel_move_target_.max_angular_speed_rps));

        // Publish command
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = cmd_vx;
        cmd_msg.linear.y = cmd_vy;
        cmd_msg.angular.z = cmd_omega;
        cmd_vel_pub_->publish(cmd_msg);

        if (verbose_logging_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Relative move control: pos_err=%.4f, yaw_err=%.4f, cmd_vel=(%.3f, %.3f, %.3f)",
                pos_error, ang_error, cmd_vx, cmd_vy, cmd_omega);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlignmentNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
