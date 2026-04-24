#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class GazeboBridge : public rclcpp::Node
{
public:
    GazeboBridge() : Node("gazebo_bridge")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GazeboBridge::loop, this));
    }

    void set_target(double x, double y)
    {
        target_x_ = x;
        target_y_ = y;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double target_x_{0.0};
    double target_y_{0.0};

    void loop()
    {
        auto msg = geometry_msgs::msg::Twist();

        // movimento semplice verso target
        msg.linear.x = 0.5;
        msg.angular.z = 0.0;

        cmd_pub_->publish(msg);
    }
};