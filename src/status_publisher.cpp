#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;

/*
 * StatusPublisher class inheriting from rclcpp::Node.
 * Publishes RobotStatus messages to /robot_status topic every 1000ms.
 */

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher()
        : Node("status_publisher"), battery_level_(100.0), mission_count_(0)
    {
        // Create publisher for /robot_status
        publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>("/robot_status", 10);

        // Create timer that fires every 1000ms
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&StatusPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = ros2_custom_msgs::msg::RobotStatus();
        message.robot_name = "Explorer1";
        message.battery_level = battery_level_;
        message.is_active = true;
        message.mission_count = mission_count_;

        // Update state
        battery_level_ -= 0.5;
        mission_count_++;

        // Publish and log
        RCLCPP_INFO(this->get_logger(), "Publishing: name='%s', battery=%.1f, active=%s, mission=%d",
                    message.robot_name.c_str(), message.battery_level,
                    message.is_active ? "true" : "false", message.mission_count);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
    double battery_level_;
    int32_t mission_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
