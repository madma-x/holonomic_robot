#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "can_interface/msg/can_frame.hpp"
#include "robot_hw_interfaces/msg/safety_state.hpp"

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

        // TRANSIENT_LOCAL so we get the last safety state on startup if already published
        auto safety_qos = rclcpp::QoS(1).reliable().transient_local();
        safety_sub_ = create_subscription<robot_hw_interfaces::msg::SafetyState>(
            "/safety_state", safety_qos,
            std::bind(&CANBridge::safety_cb, this, std::placeholders::_1));

        running_ = true;
        rx_thread_ = std::thread(&CANBridge::rx_loop, this);

        RCLCPP_INFO(get_logger(), "CAN Bridge started");
    }

    ~CANBridge()
    {
        running_ = false;
        if (rx_thread_.joinable())
            rx_thread_.join();
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (can_socket_ >= 0)
            close(can_socket_);
    }

private:
    int can_socket_ = -1;
    std::mutex socket_mutex_;
    std::thread rx_thread_;
    std::atomic<bool> running_;

    // false while safety is SAFE_OFF — gates external command paths
    std::atomic<bool> safety_on_{true};
    // tracks previous safety state to detect edges; only touched in safety_cb
    bool prev_safety_on_{true};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr can_tx_sub_;
    rclcpp::Publisher<can_interface::msg::CanFrame>::SharedPtr can_rx_pub_;
    rclcpp::Subscription<robot_hw_interfaces::msg::SafetyState>::SharedPtr safety_sub_;

    // ---------------------------------------------------------------- socket --

    // Open, bind, and set non-blocking. Called with socket_mutex_ already held
    // (or before rx_thread_ starts). Sets can_socket_.
    void open_and_bind_socket()
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
            throw std::runtime_error(std::string("Failed to create CAN socket: ") + strerror(errno));

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            close(can_socket_);
            can_socket_ = -1;
            throw std::runtime_error(std::string("Failed to get CAN interface index: ") + strerror(errno));
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(can_socket_);
            can_socket_ = -1;
            throw std::runtime_error(std::string("Failed to bind CAN socket: ") + strerror(errno));
        }

        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    }

    // Drain kernel RX buffer. Call while holding socket_mutex_ (or before
    // rx_thread_ starts).
    void drain_rx_buffer()
    {
        struct can_frame stale{};
        int drained = 0;
        while (read(can_socket_, &stale, sizeof(stale)) > 0) { ++drained; }
        if (drained > 0)
            RCLCPP_WARN(get_logger(), "Drained %d stale CAN frame(s) from RX buffer", drained);
    }

    void init_can()
    {
        RCLCPP_INFO(get_logger(), "Initializing CAN interface...");
        open_and_bind_socket();
        drain_rx_buffer();
        RCLCPP_INFO(get_logger(), "CAN interface initialized successfully");
    }

    // Close and reopen the socket to flush both the kernel TX queue (discards
    // frames not yet sent to the bus) and the RX buffer (discards stale
    // incoming frames). Takes socket_mutex_ — do NOT call from rx_loop.
    void flush_can_socket()
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (can_socket_ >= 0) {
            close(can_socket_);
            can_socket_ = -1;
        }
        try {
            open_and_bind_socket();
            drain_rx_buffer();
            RCLCPP_INFO(get_logger(), "CAN socket flushed and reopened");
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Failed to reopen CAN socket: %s", e.what());
        }
    }

    // ---------------------------------------------------------------- motors --

    void init_motors()
    {
        RCLCPP_INFO(get_logger(), "Initializing motors...");
        uint8_t motor_ids[] = {0x1, 0x2, 0x3};
        for (uint8_t id : motor_ids) {
            uint8_t data[3];
            data[0] = 0xF3;
            data[1] = 0x01;
            data[2] = calculate_crc_with_id(id, data, 2);
            send_frame(id, 3, data);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        RCLCPP_INFO(get_logger(), "Motor initialization commands sent");
    }

    // ---------------------------------------------------------------- safety --

    void safety_cb(const robot_hw_interfaces::msg::SafetyState::SharedPtr msg)
    {
        using SS = robot_hw_interfaces::msg::SafetyState;
        bool now_safe = (msg->state == SS::SAFE_ON);
        bool was_safe = prev_safety_on_;
        prev_safety_on_ = now_safe;

        if (was_safe && !now_safe) {
            // Edge: SAFE_ON → SAFE_OFF
            // Gate commands first so nothing new arrives while we flush.
            safety_on_ = false;
            RCLCPP_WARN(get_logger(), "Safety SAFE_OFF — flushing CAN queue.");
            flush_can_socket();
        } else if (!was_safe && now_safe) {
            // Edge: SAFE_OFF → SAFE_ON
            // Flush first to discard any frames that accumulated while motors
            // were offline, then re-enable motors, then open the command gate.
            RCLCPP_INFO(get_logger(),
                "Safety SAFE_ON — flushing CAN queue and reinitializing motors.");
            flush_can_socket();
            init_motors();
            safety_on_ = true;
        }
    }

    // ----------------------------------------------------------- CAN send/recv --

    void send_frame(uint16_t id, uint8_t dlc, uint8_t *data)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (can_socket_ < 0) return;
        struct can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = dlc;
        memcpy(frame.data, data, dlc);
        write(can_socket_, &frame, sizeof(frame));
    }

    // Motor command for three-wheel holonomic robot (120° spacing)
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!safety_on_) return;

        double v_x = msg->linear.x;
        double v_y = msg->linear.y;
        double w_z = msg->angular.z;

        const double wheel_radius = 0.03;
        const double robot_radius = 0.125;

        // Three wheels at -60°, 60°, 180°
        const double angle0 = -M_PI*5.0/6.0;
        const double angle1 = -M_PI/6.0;
        const double angle2 = M_PI/2.0;

        double v0 = cos(angle0)*v_x - sin(angle0)*v_y + robot_radius*w_z;
        double v1 = cos(angle1)*v_x - sin(angle1)*v_y + robot_radius*w_z;
        double v2 = cos(angle2)*v_x - sin(angle2)*v_y + robot_radius*w_z;

        double rpm0 = (v0 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm1 = (v1 / (2*M_PI*wheel_radius)) * 60.0;
        double rpm2 = (v2 / (2*M_PI*wheel_radius)) * 60.0;

        const uint8_t acc = 220;
        send_motor_command(0x1, rpm0, acc);
        send_motor_command(0x2, rpm1, acc);
        send_motor_command(0x3, rpm2, acc);
    }

    void can_tx_callback(const can_interface::msg::CanFrame::SharedPtr msg)
    {
        if (!safety_on_) return;
        uint8_t data[8];
        memcpy(data, msg->data.data(), msg->dlc);
        send_frame(msg->id, msg->dlc, data);
    }

    void rx_loop()
    {
        struct can_frame frame;

        while (running_ && rclcpp::ok())
        {
            int nbytes = -1;
            int saved_errno = EAGAIN;
            {
                std::lock_guard<std::mutex> lock(socket_mutex_);
                if (can_socket_ >= 0) {
                    nbytes = read(can_socket_, &frame, sizeof(frame));
                    saved_errno = errno;
                }
            }

            if (nbytes < 0)
            {
                if (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                // EBADF / transient errors during socket flush — sleep and retry
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (saved_errno != EBADF)
                    RCLCPP_ERROR(get_logger(), "CAN read error: %s", strerror(saved_errno));
                continue;
            }
            if (nbytes > 0)
            {
                uint16_t id = frame.can_id & 0x7FF;

                if (frame.data[0] == 0xF3 && frame.can_dlc >= 3)
                {
                    uint8_t status = frame.data[1];
                    if (status == 0x01)
                        RCLCPP_INFO(get_logger(), "Motor %d enabled successfully", id);
                    else
                        RCLCPP_ERROR(get_logger(), "Motor %d enable failed", id);
                }
                else if (id == 1 || id == 2 || id == 3)
                {
                    auto out = can_interface::msg::CanFrame();
                    out.id = id;
                    out.dlc = frame.can_dlc;
                    for (int i = 0; i < frame.can_dlc; i++)
                        out.data[i] = frame.data[i];
                    can_rx_pub_->publish(out);
                }
            }
        }
    }

    // ---------------------------------------------------------------- helpers --

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
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANBridge>());
    rclcpp::shutdown();
    return 0;
}
