#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "can_interface/msg/can_frame.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>

class CANBridge : public rclcpp::Node
{
public:
    CANBridge() : Node("can_bridge"),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
        odom_x_(0.0), odom_y_(0.0), odom_theta_(0.0),
        last_vx_(0.0), last_vy_(0.0), last_wz_(0.0)
    {
        odom_x_ = declare_parameter("initial_x", 0.2);
        odom_y_ = declare_parameter("initial_y", 0.2);
        odom_theta_ = declare_parameter("initial_yaw", 0.0);

        init_can();
        init_motors();

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CANBridge::cmd_callback, this, std::placeholders::_1));

        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&CANBridge::initial_pose_callback, this, std::placeholders::_1));

        can_tx_sub_ = create_subscription<can_interface::msg::CanFrame>(
            "/can_tx", 10,
            std::bind(&CANBridge::can_tx_callback, this, std::placeholders::_1));

        can_rx_pub_ = create_publisher<can_interface::msg::CanFrame>(
            "/can_rx", 10);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        last_odom_time_ = now();

        // 50 Hz dead-reckoning odometry from cmd_vel integration
        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CANBridge::publish_odom, this));

        running_ = true;
        rx_thread_ = std::thread(&CANBridge::rx_loop, this);

        RCLCPP_INFO(
            get_logger(),
            "CAN Bridge started (initial pose: x=%.3f, y=%.3f, yaw=%.3f rad, waiting for /initialpose or tracker)",
            odom_x_, odom_y_, odom_theta_);
    }

    ~CANBridge()
    {
        running_ = false;
        if (rx_thread_.joinable())
            rx_thread_.join();
        close(can_socket_);
    }

private:
    int can_socket_;
    std::thread rx_thread_;
    std::atomic<bool> running_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr can_tx_sub_;
    rclcpp::Publisher<can_interface::msg::CanFrame>::SharedPtr can_rx_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Dead-reckoning state
    double odom_x_, odom_y_, odom_theta_;
    double last_vx_, last_vy_, last_wz_;
    rclcpp::Time last_odom_time_;
    std::mutex odom_mutex_;
    std::atomic<bool> tracker_odom_active_{false};

    static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    static double normalize_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void publish_odom_message(
        const rclcpp::Time & stamp,
        double x,
        double y,
        double theta,
        double vx,
        double vy,
        double wz)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;
        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_footprint";
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    void set_pose(double x, double y, double theta, bool reset_velocity)
    {
        const auto stamp = now();
        double vx;
        double vy;
        double wz;

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_x_ = x;
            odom_y_ = y;
            odom_theta_ = normalize_angle(theta);
            last_odom_time_ = stamp;

            if (reset_velocity)
            {
                last_vx_ = 0.0;
                last_vy_ = 0.0;
                last_wz_ = 0.0;
            }

            vx = last_vx_;
            vy = last_vy_;
            wz = last_wz_;
        }

        publish_odom_message(stamp, x, y, normalize_angle(theta), vx, vy, wz);
    }

    void init_can()
    {
        RCLCPP_INFO(get_logger(), "Initializing CAN interface...");

        // Create CAN socket
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to create CAN socket: %s", strerror(errno));
            throw std::runtime_error("Failed to create CAN socket");
        }
        RCLCPP_INFO(get_logger(), "CAN socket created successfully");

        // Get interface index
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to get CAN interface index: %s", strerror(errno));
            close(can_socket_);
            throw std::runtime_error("Failed to get CAN interface index");
        }
        RCLCPP_INFO(get_logger(), "CAN interface 'can0' found with index %d", ifr.ifr_ifindex);

        // Bind socket to interface
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to bind CAN socket: %s", strerror(errno));
            close(can_socket_);
            throw std::runtime_error("Failed to bind CAN socket");
        }
        RCLCPP_INFO(get_logger(), "CAN socket bound successfully");

        // Set socket to non-blocking mode
        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

        // Drain any stale frames left in the kernel RX buffer from a previous
        // run so the node starts with a clean slate.
        {
            struct can_frame stale{};
            int drained = 0;
            while (read(can_socket_, &stale, sizeof(stale)) > 0) { ++drained; }
            if (drained > 0)
                RCLCPP_WARN(get_logger(), "Drained %d stale CAN frame(s) from RX buffer at startup", drained);
        }

        RCLCPP_INFO(get_logger(), "CAN interface initialized successfully");
    }

    void init_motors()
    {
        RCLCPP_INFO(get_logger(), "Initializing motors...");
        
        uint8_t motor_ids[] = {0x1, 0x2, 0x3};

        for (uint8_t id : motor_ids)
        {
            uint8_t data[3];
            data[0] = 0xF3;  // Code
            data[1] = 0x01;  // Enable motor
            data[2] = calculate_crc_with_id(id, data, 2);

            send_frame(id, 3, data);

            // Wait briefly for response
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_INFO(get_logger(), "Motor initialization commands sent");
    }

    void send_frame(uint16_t id, uint8_t dlc, uint8_t *data)
    {
        struct can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = dlc;
        memcpy(frame.data, data, dlc);
        write(can_socket_, &frame, sizeof(frame));
    }

    void publish_odom()
    {
        if (tracker_odom_active_)
        {
            return;
        }

        rclcpp::Time current_time = now();
        double dt;
        double x;
        double y;
        double theta;
        double vx, vy, wz;

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            dt = (current_time - last_odom_time_).seconds();
            last_odom_time_ = current_time;
            vx = last_vx_;
            vy = last_vy_;
            wz = last_wz_;

            // Integrate pose in world frame (holonomic: vx/vy are robot-frame velocities)
            const double delta_x = (vx * std::cos(odom_theta_) - vy * std::sin(odom_theta_)) * dt;
            const double delta_y = (vx * std::sin(odom_theta_) + vy * std::cos(odom_theta_)) * dt;
            const double delta_theta = wz * dt;

            odom_x_ += delta_x;
            odom_y_ += delta_y;
            odom_theta_ = normalize_angle(odom_theta_ + delta_theta);

            x = odom_x_;
            y = odom_y_;
            theta = odom_theta_;
        }

        publish_odom_message(current_time, x, y, theta, vx, vy, wz);
    }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!msg->header.frame_id.empty() &&
            msg->header.frame_id != "map" &&
            msg->header.frame_id != "odom")
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring /initialpose in frame '%s' (expected 'map' or 'odom')",
                msg->header.frame_id.c_str());
            return;
        }

        tracker_odom_active_ = false;

        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double theta = quaternion_to_yaw(msg->pose.pose.orientation);

        set_pose(x, y, theta, true);

        RCLCPP_INFO(
            get_logger(),
            "Odometry pose set from /initialpose: x=%.3f, y=%.3f, yaw=%.3f rad",
            x, y, theta);
    }

    // Motor command for three-wheel holonomic robot (120° spacing)
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v_x = msg->linear.x;
        double v_y = msg->linear.y;
        double w_z = msg->angular.z;

        // Store velocities for odometry integration
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            last_vx_ = v_x;
            last_vy_ = v_y;
            last_wz_ = w_z;
        }

        const double wheel_radius = 0.03;
        const double robot_radius = 0.125;  // distance from center to wheel

        // Three wheels at -60°, 60°, 180°
        const double angle0 = -M_PI*5.0/6.0;
        const double angle1 = -M_PI/6.0;
        const double angle2 = M_PI/2.0;

        // Inverse kinematics for holonomic drive
        double v0 = cos(angle0)*v_x - sin(angle0)*v_y + robot_radius*w_z;
        double v1 = cos(angle1)*v_x - sin(angle1)*v_y + robot_radius*w_z;
        double v2 = cos(angle2)*v_x - sin(angle2)*v_y + robot_radius*w_z;

        // Convert to RPM
        double rpm0 = (v0 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm1 = (v1 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm2 = (v2 / (2*M_PI*wheel_radius)) * 60.0;

        const uint8_t acc = 220;  // acceleration
        send_motor_command(0x1, rpm0, acc);
        send_motor_command(0x2, rpm1, acc);
        send_motor_command(0x3, rpm2, acc);
    }

    void can_tx_callback(const can_interface::msg::CanFrame::SharedPtr msg)
    {
        uint8_t data[8];
        memcpy(data, msg->data.data(), msg->dlc);
        send_frame(msg->id, msg->dlc, data);
    }

    void rx_loop()
    {
        struct can_frame frame;

        while (running_ && rclcpp::ok())
        {
            int nbytes = read(can_socket_, &frame, sizeof(frame));

            if (nbytes < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                RCLCPP_ERROR(get_logger(), "CAN read error: %s", strerror(errno));
                continue;
            }
            if (nbytes > 0)
            {
                uint16_t id = frame.can_id & 0x7FF;

                // -------------------------
                // Motor enable response (F3)
                // -------------------------
                if (frame.data[0] == 0xF3 && frame.can_dlc >= 3)
                {
                    uint8_t status = frame.data[1];
                    if (status == 0x01)
                        RCLCPP_INFO(get_logger(), "Motor %d enabled successfully", id);
                    else
                        RCLCPP_ERROR(get_logger(), "Motor %d enable failed", id);
                }

                // -------------------------
                // Motor feedback (1,2,3)
                // -------------------------
                else if (id == 1 || id == 2 || id == 3)
                {
                    auto msg = can_interface::msg::CanFrame();
                    msg.id = id;
                    msg.dlc = frame.can_dlc;

                    for (int i = 0; i < frame.can_dlc; i++)
                        msg.data[i] = frame.data[i];

                    can_rx_pub_->publish(msg);
                }

                // -------------------------
                // Movement tracker (ID 4)
                // -------------------------
                else if (id == 4 && frame.can_dlc >= 8)
                {
                    float x, y;

                    std::memcpy(&x, &frame.data[0], sizeof(float));
                    std::memcpy(&y, &frame.data[4], sizeof(float));

                    // If tracker provides theta, decode it here.
                    // For now assume theta = 0:
                    double theta = 0.0;
                    tracker_odom_active_ = true;
                    set_pose(x, y, theta, false);
                }
            }
        }
    }

    uint8_t calculate_crc_with_id(uint16_t id, uint8_t *data, int len)
    {
        uint16_t sum = id;
        for (int i = 0; i < len; i++)
            sum += data[i];
        return sum & 0xFF;
    }

    void send_motor_command(uint16_t id, double rpm, uint8_t acc)
    {
        uint16_t speed = static_cast<uint16_t>(std::abs(rpm));
        if (speed > 3000) speed = 3000;

        // dir: 0 = CCW (forward), 1 = CW (reverse) — per protocol spec
        uint8_t dir = (rpm >= 0) ? 0 : 1;

        // Byte2: bit7 = dir, bits3-0 = speed upper nibble
        uint8_t byte2 = ((dir & 0x1) << 7) | ((speed >> 8) & 0x0F);
        uint8_t byte3 = speed & 0xFF;

        uint8_t data[5];
        data[0] = 0xF6;
        data[1] = byte2;
        data[2] = byte3;
        data[3] = acc;
        data[4] = calculate_crc_with_id(id, data, 4);

        send_frame(id, 5, data);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANBridge>());
    rclcpp::shutdown();
    return 0;
}