#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "can_interface/msg/can_frame.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

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

class CANBridge : public rclcpp::Node
{
public:
    CANBridge() : Node("can_bridge")
    {
        init_can();
        init_motors();

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CANBridge::cmd_callback, this, std::placeholders::_1));

        can_tx_sub_ = create_subscription<can_interface::msg::CanFrame>(
            "/can_tx", 10,
            std::bind(&CANBridge::can_tx_callback, this, std::placeholders::_1));

        can_rx_pub_ = create_publisher<can_interface::msg::CanFrame>(
            "/can_rx", 10);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        running_ = true;
        rx_thread_ = std::thread(&CANBridge::rx_loop, this);

        RCLCPP_INFO(get_logger(), "CAN Bridge started");
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
    rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr can_tx_sub_;
    rclcpp::Publisher<can_interface::msg::CanFrame>::SharedPtr can_rx_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

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

        RCLCPP_INFO(get_logger(), "CAN interface initialized successfully");
    }

    void init_motors()
    {
        RCLCPP_INFO(get_logger(), "Initializing motors...");
        
        uint8_t motor_ids[] = {0x1, 0x2, 0x3};
        bool all_success = true;

        for (uint8_t id : motor_ids)
        {
            uint8_t data[3];
            data[0] = 0xF3;  // Code
            data[1] = 0x01;  // Enable motor
            data[2] = calculate_crc(data, 2);

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

    // Motor command for three-wheel holonomic robot (120° spacing)
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v_x = msg->linear.x;
        double v_y = msg->linear.y;
        double w_z = msg->angular.z;

        const double wheel_radius = 0.03;
        const double robot_radius = 0.15;  // distance from center to wheel

        // Three wheels at 0°, 120°, 240°
        const double angle0 = 0;
        const double angle1 = 2*M_PI/3.0;
        const double angle2 = 4*M_PI/3.0;

        // Inverse kinematics for holonomic drive
        double v0 = cos(angle0)*v_x + sin(angle0)*v_y + robot_radius*w_z;
        double v1 = cos(angle1)*v_x + sin(angle1)*v_y + robot_radius*w_z;
        double v2 = cos(angle2)*v_x + sin(angle2)*v_y + robot_radius*w_z;

        // Convert to RPM
        double rpm0 = (v0 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm1 = (v1 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm2 = (v2 / (2*M_PI*wheel_radius)) * 60.0;

        const uint8_t acc = 2;  // acceleration
        send_motor_command(0x1, rpm0, acc);
        send_motor_command(0x2, rpm1, acc);
        send_motor_command(0x3, rpm2, acc);
    }

    void can_tx_callback(const my_robot_can::msg::CanFrame::SharedPtr msg)
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
                    auto msg = my_robot_can::msg::CanFrame();
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

                    nav_msgs::msg::Odometry odom;
                    odom.header.stamp = now();
                    odom.header.frame_id = "odom";
                    odom.child_frame_id = "base_link";

                    odom.pose.pose.position.x = x;
                    odom.pose.pose.position.y = y;
                    odom.pose.pose.position.z = 0.0;

                    // If tracker provides theta, decode it here.
                    // For now assume theta = 0:
                    double theta = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, theta);

                    odom.pose.pose.orientation.x = q.x();
                    odom.pose.pose.orientation.y = q.y();
                    odom.pose.pose.orientation.z = q.z();
                    odom.pose.pose.orientation.w = q.w();

                    odom_pub_->publish(odom);
                }
            }
        }
    }

    uint8_t calculate_crc(uint8_t *data, int len)
    {
        uint8_t crc = 0;
        for (int i = 0; i < len; i++)
            crc ^= data[i];
        return crc;
    }

    void send_motor_command(uint16_t id, double rpm, uint8_t acc)
    {
        // Clamp speed to 0-3000 RPM range
        int16_t speed = static_cast<int16_t>(std::abs(rpm));
        if (speed > 3000) speed = 3000;
        if (speed < 0) speed = 0;

        // Determine direction: 0 = CCW, 1 = CW
        uint8_t dir = (rpm >= 0) ? 1 : 0;

        // Encode speed: upper 4 bits in Byte2, lower 8 bits in Byte3
        uint8_t byte2 = 0xF6 | ((dir & 0x1) << 7) | ((speed >> 8) & 0x0F);
        uint8_t byte3 = speed & 0xFF;

        uint8_t data[5];
        data[0] = 0xF6;  // Code
        data[1] = byte2;
        data[2] = byte3;
        data[3] = acc;
        data[4] = calculate_crc(data, 4);

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