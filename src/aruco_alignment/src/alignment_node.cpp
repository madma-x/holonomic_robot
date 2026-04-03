#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "aruco_interfaces/msg/cluster_pickability.hpp"
#include "aruco_interfaces/srv/align_to_cluster.hpp"
#include "pid_controller.hpp"
#include <cmath>
#include <mutex>
#include <chrono>

class AlignmentNode : public rclcpp::Node {
public:
    AlignmentNode() : Node("alignment_node") {
        // Declare parameters
        this->declare_parameter<double>("control_loop_rate", 50.0);
        this->declare_parameter<int>("cluster_timeout_ms", 500);
        this->declare_parameter<double>("pid_xy.kp", 0.5);
        this->declare_parameter<double>("pid_xy.ki", 0.05);
        this->declare_parameter<double>("pid_xy.kd", 0.1);
        this->declare_parameter<double>("pid_xy.max_integral_error", 0.5);
        this->declare_parameter<double>("pid_xy.max_velocity", 0.5);
        this->declare_parameter<double>("pid_theta.kp", 0.3);
        this->declare_parameter<double>("pid_theta.ki", 0.02);
        this->declare_parameter<double>("pid_theta.kd", 0.05);
        this->declare_parameter<double>("pid_theta.max_integral_error", 0.3);
        this->declare_parameter<double>("pid_theta.max_angular_velocity", 1.0);
        this->declare_parameter<double>("pos_tolerance", 0.02);
        this->declare_parameter<double>("ang_tolerance", 0.05);
        this->declare_parameter<std::string>("cluster_topic", "/aruco_manager/cluster_pickability");
        this->declare_parameter<std::string>("odometry_topic", "/odom");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        // Get parameters
        double control_rate = this->get_parameter("control_loop_rate").as_double();
        cluster_timeout_ms_ = this->get_parameter("cluster_timeout_ms").as_int();
        pos_tolerance_ = this->get_parameter("pos_tolerance").as_double();
        ang_tolerance_ = this->get_parameter("ang_tolerance").as_double();

        // Create subscriptions
        std::string cluster_topic = this->get_parameter("cluster_topic").as_string();
        std::string odom_topic = this->get_parameter("odometry_topic").as_string();
        cluster_sub_ = this->create_subscription<aruco_interfaces::msg::ClusterPickability>(
            cluster_topic, 10,
            [this](const aruco_interfaces::msg::ClusterPickability::SharedPtr msg) {
                this->cluster_callback(msg);
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odom_callback(msg);
            });

        // Create publisher and service
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);

        service_ = this->create_service<aruco_interfaces::srv::AlignToCluster>(
            "align_to_cluster",
            [this](const std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Request> request,
                   std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Response> response) {
                this->service_callback(request, response);
            });

        // Initialize PID controllers
        init_pid_controllers();

        // Create control loop timer
        auto period = std::chrono::duration<double>(1.0 / control_rate);
        timer_ = this->create_wall_timer(
            period,
            [this]() { this->control_loop(); });

        RCLCPP_INFO(this->get_logger(), "Alignment node initialized");
    }

private:
    // Subscriptions and publishers
    rclcpp::Subscription<aruco_interfaces::msg::ClusterPickability>::SharedPtr cluster_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<aruco_interfaces::srv::AlignToCluster>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

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

    // Robot state (from odometry)
    struct RobotState {
        double x, y, theta;
    } robot_state_{0.0, 0.0, 0.0};

    // Alignment control state
    int32_t target_cluster_id_ = -1;
    bool alignment_active_ = false;
    rclcpp::Time alignment_start_time_;
    bool alignment_complete_ = false;
    bool alignment_success_ = false;
    std::string alignment_message_;

    // Configuration
    int cluster_timeout_ms_;
    double pos_tolerance_, ang_tolerance_;

    // Single-loop PID controllers (one per axis)
    std::unique_ptr<PIDController> pid_xy_;   // XY position error → velocity command
    std::unique_ptr<PIDController> pid_theta_; // angle error → angular velocity command

    // Synchronization
    std::mutex state_mutex_;

    void init_pid_controllers() {
        // XY position loop: position error → velocity output
        PIDController::Gains gains_xy = {
            this->get_parameter("pid_xy.kp").as_double(),
            this->get_parameter("pid_xy.ki").as_double(),
            this->get_parameter("pid_xy.kd").as_double()
        };
        PIDController::Limits limits_xy = {
            this->get_parameter("pid_xy.max_integral_error").as_double(),
            this->get_parameter("pid_xy.max_velocity").as_double()
        };
        pid_xy_ = std::make_unique<PIDController>(gains_xy, limits_xy);

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

        // Update last confident pose when cluster is seen and is pickable
        if (msg->is_pickable) {
            last_confident_pose_.x = msg->correction.x;
            last_confident_pose_.y = msg->correction.y;
            last_confident_pose_.theta = 0.0; // Correction is in XY plane
            have_confident_pose_ = true;

            RCLCPP_DEBUG(this->get_logger(),
                "Cluster %d received: correction=(%.3f, %.3f), magnitude=%.3f",
                msg->cluster_id, msg->correction.x, msg->correction.y, msg->correction_magnitude);
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

    void service_callback(
        const std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Request> request,
        std::shared_ptr<aruco_interfaces::srv::AlignToCluster::Response> response) {

        std::lock_guard<std::mutex> lock(state_mutex_);

        target_cluster_id_ = request->cluster_id;
        alignment_active_ = true;
        alignment_complete_ = false;
        latest_cluster_.received = false;
        alignment_start_time_ = this->now();

        // Reset PID controllers
        pid_xy_->reset();
        pid_theta_->reset();

        RCLCPP_INFO(this->get_logger(),
            "Alignment service called: cluster_id=%d, threshold=%.4f, max_wait=%.2fs",
            request->cluster_id, request->alignment_threshold, request->max_wait_time);

        // Wait for alignment to complete (blocking)
        auto wait_timeout = rclcpp::Duration::from_seconds(
            request->max_wait_time > 0 ? request->max_wait_time : 60.0);
        auto start_time = this->now();

        while (!alignment_complete_) {
            if (request->max_wait_time > 0 &&
                (this->now() - start_time) > wait_timeout) {
                alignment_complete_ = true;
                alignment_success_ = false;
                alignment_message_ = "Timeout waiting for alignment";
                break;
            }
            state_mutex_.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            state_mutex_.lock();
        }

        // Populate response
        response->success = alignment_success_;
        response->final_correction.x = last_confident_pose_.x;
        response->final_correction.y = last_confident_pose_.y;
        response->final_correction.z = 0.0;
        response->final_magnitude = std::hypot(last_confident_pose_.x, last_confident_pose_.y);
        response->status_message = alignment_message_;

        alignment_active_ = false;

        RCLCPP_INFO(this->get_logger(),
            "Alignment complete: success=%d, message=%s",
            alignment_success_, alignment_message_.c_str());
    }

    void control_loop() {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!alignment_active_) {
            // Publish zero velocity when not aligning
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(cmd_msg);
            return;
        }

        // Check cluster timeout
        bool cluster_available = false;
        PoseXY target_pose{0.0, 0.0, 0.0};

        if (latest_cluster_.received) {
            auto age_ms = (this->now() - latest_cluster_.timestamp).nanoseconds() / 1e6;
            if (age_ms < cluster_timeout_ms_) {
                // Cluster is fresh
                cluster_available = true;
                target_pose.x = latest_cluster_.msg.correction.x;
                target_pose.y = latest_cluster_.msg.correction.y;
                target_pose.theta = 0.0;
            }
        }

        // If cluster not available, use last confident pose
        if (!cluster_available && have_confident_pose_) {
            target_pose = last_confident_pose_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Cluster lost, aligning to last confident pose");
        } else if (!cluster_available && !have_confident_pose_) {
            // No target available
            alignment_complete_ = true;
            alignment_success_ = false;
            alignment_message_ = "No cluster available and no last confident pose";
            return;
        }

        // Compute errors in robot frame
        double error_x = target_pose.x - robot_state_.x;
        double error_y = target_pose.y - robot_state_.y;
        double error_theta = target_pose.theta - robot_state_.theta;

        // Normalize angle error to [-pi, pi]
        while (error_theta > M_PI) error_theta -= 2.0 * M_PI;
        while (error_theta < -M_PI) error_theta += 2.0 * M_PI;

        // Check convergence
        double pos_error = std::hypot(error_x, error_y);
        if (pos_error < pos_tolerance_ && std::abs(error_theta) < ang_tolerance_) {
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(cmd_msg);
            alignment_complete_ = true;
            alignment_success_ = true;
            alignment_message_ = "Alignment successful";
            return;
        }

        // Control loop timing
        static rclcpp::Time last_loop_time = this->now();
        double dt = (this->now() - last_loop_time).nanoseconds() / 1e9;
        last_loop_time = this->now();
        if (dt <= 0.0) dt = 0.02; // Fallback

        // Single PID loop per axis: position error → velocity command
        double cmd_vx = pid_xy_->update(error_x, dt);
        double cmd_vy = pid_xy_->update(error_y, dt);
        double cmd_omega = pid_theta_->update(error_theta, dt);

        // Publish command
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = cmd_vx;
        cmd_msg.linear.y = cmd_vy;
        cmd_msg.angular.z = cmd_omega;
        cmd_vel_pub_->publish(cmd_msg);

        RCLCPP_DEBUG(this->get_logger(),
            "Control: pos_err=(%.4f, %.4f), ang_err=%.4f, cmd_vel=(%.3f, %.3f, %.3f)",
            error_x, error_y, error_theta, cmd_vx, cmd_vy, cmd_omega);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlignmentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
