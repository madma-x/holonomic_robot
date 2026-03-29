#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>

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

class CANTester : public rclcpp::Node
{
public:
    CANTester() : Node("can_tester")
    {
        init_can();

        // Services for testing
        test_enable_srv_ = create_service<std_srvs::srv::Trigger>(
            "/can_test/enable_motors",
            std::bind(&CANTester::test_enable_motors, this, std::placeholders::_1, std::placeholders::_2));

        test_disable_srv_ = create_service<std_srvs::srv::Trigger>(
            "/can_test/disable_motors",
            std::bind(&CANTester::test_disable_motors, this, std::placeholders::_1, std::placeholders::_2));

        test_spin_srv_ = create_service<std_srvs::srv::Trigger>(
            "/can_test/spin_motors",
            std::bind(&CANTester::test_spin_motors, this, std::placeholders::_1, std::placeholders::_2));

        test_stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "/can_test/stop_motors",
            std::bind(&CANTester::test_stop_motors, this, std::placeholders::_1, std::placeholders::_2));

        test_immediate_stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "/can_test/emergency_stop",
            std::bind(&CANTester::test_immediate_stop, this, std::placeholders::_1, std::placeholders::_2));

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CANTester::cmd_vel_callback, this, std::placeholders::_1));

        running_ = true;
        rx_thread_ = std::thread(&CANTester::rx_loop, this);

        RCLCPP_INFO(get_logger(), "CAN Tester initialized");
        RCLCPP_INFO(get_logger(), "Available services:");
        RCLCPP_INFO(get_logger(), "  - /can_test/enable_motors");
        RCLCPP_INFO(get_logger(), "  - /can_test/disable_motors");
        RCLCPP_INFO(get_logger(), "  - /can_test/spin_motors (3 sec @ 500 RPM, then gradual stop)");
        RCLCPP_INFO(get_logger(), "  - /can_test/stop_motors (gradual deceleration)");
        RCLCPP_INFO(get_logger(), "  - /can_test/emergency_stop (immediate stop, use with caution!)");
        RCLCPP_INFO(get_logger(), "  - Subscribe to /cmd_vel for holonomic movement");
    }

    ~CANTester()
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

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_disable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_spin_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_immediate_stop_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void init_can()
    {
        RCLCPP_INFO(get_logger(), "Initializing CAN interface...");

        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to create CAN socket: %s", strerror(errno));
            throw std::runtime_error("Failed to create CAN socket");
        }
        RCLCPP_INFO(get_logger(), "CAN socket created");

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_FATAL(get_logger(), "Failed to get CAN interface: %s", strerror(errno));
            close(can_socket_);
            throw std::runtime_error("Failed to get CAN interface");
        }
        RCLCPP_INFO(get_logger(), "CAN interface 'can0' found (index: %d)", ifr.ifr_ifindex);

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

       //int flags = fcntl(can_socket_, F_GETFL, 0);
    //fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(get_logger(), "CAN interface ready");
    }

    void set_motor_enable_state(uint16_t id, uint8_t enable)
    {
        enable = (enable != 0) ? 0x01 : 0x00;

        uint8_t data[3];
        data[0] = 0xF3;          // Enable command code
        data[1] = enable;        // 0x00: disable, 0x01: enable
        data[2] = calculate_crc_with_id(id, data, 2);

        send_frame(id, 3, data);

        RCLCPP_INFO(get_logger(), "Motor %d %s command sent",
                    id, enable == 0x01 ? "enable" : "disable");
    }

    void enable_motor(uint16_t id)
    {
        set_motor_enable_state(id, 0x01);
    }

    void disable_motor(uint16_t id)
    {
        set_motor_enable_state(id, 0x00);
    }
    void send_frame(uint16_t id, uint8_t dlc, uint8_t *data)
    {
        struct can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = dlc;
        memcpy(frame.data, data, dlc);
        write(can_socket_, &frame, sizeof(frame));
        RCLCPP_DEBUG(get_logger(), "Sent CAN frame ID:0x%X DLC:%d", id, dlc);
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

    void stop_motor_gradual(uint16_t id, uint8_t decel_acc)
    {
    uint8_t data[5];
    data[0] = 0xF6;
    data[1] = 0x00;     // speed = 0
    data[2] = 0x00;
    data[3] = decel_acc;
    data[4] = calculate_crc_with_id(id, data, 4);

    send_frame(id, 5, data);
    RCLCPP_INFO(get_logger(), "Motor %d gradual stop command sent (decel: %d)", id, decel_acc);
    }

    void stop_motor_immediate(uint16_t id)
    {
        // Immediate stop command
        // Speed = 0, acc = 0
        uint8_t data[5];
        data[0] = 0xF6;  // Code
        data[1] = 0xF6;  // Direction bit 7 = 0, speed upper 4 bits = 0
        data[2] = 0x00;  // Speed lower 8 bits = 0
        data[3] = 0x00;  // Acceleration = 0 (immediate stop)
        data[4] = calculate_crc_with_id(id, data, 4);

        send_frame(id, 5, data);
        RCLCPP_INFO(get_logger(), "Motor %d immediate stop command sent", id);
    }

    bool test_enable_motors(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(get_logger(), "=== Testing Motor Enable ===");
        
        for (uint8_t id = 1; id <= 3; id++)
        {
            enable_motor(id);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        response->success = true;
        response->message = "Enable commands sent to motors 1, 2, 3";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        return true;
    }

    bool test_disable_motors(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(get_logger(), "=== Testing Motor Disable ===");

        for (uint8_t id = 1; id <= 3; id++)
        {
            disable_motor(id);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        response->success = true;
        response->message = "Disable commands sent to motors 1, 2, 3";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        return true;
    }

    bool test_spin_motors(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(get_logger(), "=== Testing Motor Spin (3 sec) ===");
        
        // Spin all motors at 500 RPM for 3 seconds
        const double test_rpm = 500.0;
        const uint8_t acc = 10;
        const int duration_ms = 3000;

        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start).count() < duration_ms)
        {
            send_motor_command(1, test_rpm, acc);
            send_motor_command(2, test_rpm, acc);
            send_motor_command(3, test_rpm, acc);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(get_logger(), "Spinning complete, starting gradual stop...");
        
        // Gradual stop with deceleration
        for (int i = 0; i < 30; i++)  // ~3 seconds
        {
            stop_motor_gradual(1, 2);
            stop_motor_gradual(2, 2);
            stop_motor_gradual(3, 2);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        response->success = true;
        response->message = "Spun motors at 500 RPM for 3 seconds, then gradual stop";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        return true;
    }

    bool test_stop_motors(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(get_logger(), "=== Stopping Motors (Gradual) ===");
        
        for (uint8_t id = 1; id <= 3; id++)
        {
            stop_motor_gradual(id, 5);  // Deceleration rate 5
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        response->success = true;
        response->message = "Gradual stop command sent to all motors (decel: 5)";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        return true;
    }

    bool test_immediate_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_WARN(get_logger(), "=== EMERGENCY STOP (Immediate) ===");
        
        for (uint8_t id = 1; id <= 3; id++)
        {
            stop_motor_immediate(id);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        response->success = true;
        response->message = "EMERGENCY STOP sent (immediate stop to all motors)";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return true;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v_x = msg->linear.x;
        double v_y = msg->linear.y;
        double w_z = msg->angular.z;

        const double wheel_radius = 0.03;
        const double robot_radius = 0.15;

        const double angle0 = 0;
        const double angle1 = 2*M_PI/3.0;
        const double angle2 = 4*M_PI/3.0;

        double v0 = cos(angle0)*v_x - sin(angle0)*v_y + robot_radius*w_z;
        double v1 = cos(angle1)*v_x - sin(angle1)*v_y + robot_radius*w_z;
        double v2 = cos(angle2)*v_x - sin(angle2)*v_y + robot_radius*w_z;

        double rpm0 = (v0 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm1 = (v1 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm2 = (v2 / (2*M_PI*wheel_radius)) * 60.0;

        RCLCPP_DEBUG(get_logger(), "cmd_vel: vx=%.2f vy=%.2f wz=%.2f -> rpm0=%.0f rpm1=%.0f rpm2=%.0f",
                     v_x, v_y, w_z, rpm0, rpm1, rpm2);

        const uint8_t acc = 0;
        send_motor_command(1, rpm0, acc);
        send_motor_command(2, rpm1, acc);
        send_motor_command(3, rpm2, acc);
    }

    void rx_loop()
    {
        struct can_frame frame;

        while (running_ && rclcpp::ok())
        {
            int nbytes = read(can_socket_, &frame, sizeof(frame));

            if (nbytes <= 0)
                continue;

            uint16_t id = frame.can_id & 0x7FF;

            if (frame.can_dlc >= 3 && frame.data[0] == 0xF3)
            {
                uint8_t status = frame.data[1];
                RCLCPP_INFO(get_logger(), "Motor %d enable/disable set response: %s",
                            id, status == 0x01 ? "SUCCESS" : "FAILED");
            }
            else if (frame.data[0] == 0xF6 && frame.can_dlc >= 3)
            {
                uint8_t stop_status = frame.data[1];
                const char* status_str = "";

                switch (stop_status)
                {
                    case 0: status_str = "FAILED"; break;
                    case 1: status_str = "STOPPING"; break;
                    case 2: status_str = "STOPPED"; break;
                    default: status_str = "UNKNOWN"; break;
                }

                RCLCPP_INFO(get_logger(), "Motor %d stop response: %s", id, status_str);
            }
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANTester>());
    rclcpp::shutdown();
    return 0;
}

