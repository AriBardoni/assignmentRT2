#include <memory>
#include <thread>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "navigation_action/action/navigate.hpp"

using Navigate = navigation_action::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

namespace navigation_action
{

class NavigationServer : public rclcpp::Node
{
public:
    NavigationServer(const rclcpp::NodeOptions & options)
    : Node("navigation_action_server", options)
    {
        action_server_ = rclcpp_action::create_server<Navigate>(
            this,
            "navigate",
            std::bind(&NavigationServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigationServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&NavigationServer::handle_accepted, this,
                      std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        this->declare_parameter("yaw_precision", M_PI / 9.0);
        this->declare_parameter("yaw_precision_2", M_PI / 18.0);
        this->declare_parameter("dist_precision", 0.1);
        this->declare_parameter("kp_a", -3.0);
        this->declare_parameter("linear_speed", 0.3);

        RCLCPP_INFO(get_logger(), "Navigation Action Server started");
    }

private:
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    double goal_x_{0.0}, goal_y_{0.0}, goal_theta_{0.0};

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Navigate::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Goal: x=%.2f y=%.2f theta=%.2f",
                    goal->x, goal->y, goal->theta);

        goal_x_ = goal->x;
        goal_y_ = goal->y;
        goal_theta_ = goal->theta;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate>)
    {
        RCLCPP_WARN(get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        std::thread{std::bind(&NavigationServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        auto feedback = std::make_shared<Navigate::Feedback>();
        auto result = std::make_shared<Navigate::Result>();

        int state = 0;

        const double yaw_precision = this->get_parameter("yaw_precision").as_double();
        const double yaw_precision_2 = this->get_parameter("yaw_precision_2").as_double();
        const double dist_precision = this->get_parameter("dist_precision").as_double();
        const double kp_a = this->get_parameter("kp_a").as_double();
        const double linear_speed = this->get_parameter("linear_speed").as_double();

        rclcpp::Rate rate(20);

        while (rclcpp::ok() && state != 3)
        {
            if (goal_handle->is_canceling())
            {
                geometry_msgs::msg::Twist stop;
                cmd_vel_pub_->publish(stop);

                result->success = false;
                result->message = "Cancelled";
                goal_handle->canceled(result);
                return;
            }

            geometry_msgs::msg::Twist cmd_vel;

            try {
                auto transform = tf_buffer_->lookupTransform(
                    "odom", "base_footprint", tf2::TimePointZero);

                double current_x = transform.transform.translation.x;
                double current_y = transform.transform.translation.y;
                double current_yaw = tf2::getYaw(transform.transform.rotation);

                double dx = goal_x_ - current_x;
                double dy = goal_y_ - current_y;

                double distance = std::sqrt(dx*dx + dy*dy);
                double angle_to_goal = std::atan2(dy, dx);
                double yaw_error = normalize_angle(angle_to_goal - current_yaw);
                double final_yaw_error = normalize_angle(goal_theta_ - current_yaw);

                feedback->remaining_distance = distance;
                feedback->remaining_angle = yaw_error;
                feedback->state = state;
                goal_handle->publish_feedback(feedback);

                switch (state)
                {
                    case 0:
                        cmd_vel.linear.x = 0.0;
                        if (std::fabs(yaw_error) > yaw_precision)
                        {
                            cmd_vel.angular.z = kp_a * yaw_error;
                            cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -0.5, 0.6);
                        }
                        else
                        {
                            state = 1;
                        }
                        break;

                    case 1:
                        if (distance <= dist_precision)
                        {
                            state = 2;
                        }
                        else if (std::fabs(yaw_error) > yaw_precision)
                        {
                            state = 0;
                        }
                        else
                        {
                            cmd_vel.linear.x = linear_speed;
                            cmd_vel.angular.z = kp_a * 0.3 * yaw_error;
                        }
                        break;

                    case 2:
                        if (std::fabs(final_yaw_error) > yaw_precision_2)
                        {
                            cmd_vel.angular.z = kp_a * final_yaw_error;
                            cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -0.5, 0.6);
                        }
                        else
                        {
                            state = 3;
                        }
                        break;
                }

                cmd_vel_pub_->publish(cmd_vel);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "TF error: %s", ex.what());
            }

            rate.sleep();
        }

        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());

        result->success = true;
        result->message = "Goal reached";
        goal_handle->succeed(result);
    }
};

} // namespace navigation_action

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_action::NavigationServer)