#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "navigation_action/action/navigate.hpp"

namespace navigation_action
{

// Action server fro navigation
class NavigationServer : public rclcpp::Node
{
public:
    using Navigate = navigation_action::action::Navigate;
    using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

    explicit NavigationServer(const rclcpp::NodeOptions & options)
    : Node("navigation_server", options)
    {
        // Publish velocity command 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribe to odometry to get robot position and orientation
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&NavigationServer::odom_callback, this, std::placeholders::_1)
        );

        // Create action sevrer 
        action_server_ = rclcpp_action::create_server<Navigate>(
            this,
            "navigate",
            std::bind(&NavigationServer::handle_goal, this, 
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigationServer::handle_cancel, this, 
                      std::placeholders::_1),
            std::bind(&NavigationServer::handle_accepted, this, 
                      std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "NavigationServer ready");
    }

private:
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleNavigate> current_goal_handle_;
    // Flag for execution control
    std::atomic<bool> running_{false};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // Current robot position and orientation 
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Target goal 
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_theta_ = 0.0;

    // Extract position/orientation 
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Convert quaternions to euler angles 
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
    }

    // Called when new goal received 
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Navigate::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(),
            "Goal received: x=%.2f y=%.2f theta=%.2f",
            goal->x, goal->y, goal->theta);

        // Storing the goal 
        goal_x_ = goal->x;
        goal_y_ = goal->y;
        goal_theta_ = goal->theta;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Called when cancel is requested 
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel request");
        running_ = false;
        
        // Stop the robot 
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Called after goal accepted
    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        running_ = false;
        
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        current_goal_handle_ = goal_handle;
        running_ = true;

        std::thread{
            std::bind(&NavigationServer::execute, this, goal_handle)
        }.detach();
    }

    // Navigation logic (finite state machine)
    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        auto feedback = std::make_shared<Navigate::Feedback>();
        auto result = std::make_shared<Navigate::Result>();

        rclcpp::Rate rate(20);

        while (rclcpp::ok() && running_)
        {
            if (goal_handle->is_canceling())
            {
                running_ = false;
                geometry_msgs::msg::Twist stop;
                cmd_vel_pub_->publish(stop);
                result->success = false;
                result->message = "Cancelled";
                goal_handle->canceled(result);
                return;
            }

            // Compute distance and angle to goal
            double dx = goal_x_ - current_x_;
            double dy = goal_y_ - current_y_;
            double distance = std::sqrt(dx*dx + dy*dy);

            // Feedback
            feedback->remaining_distance = distance;
            double angle_to_goal = std::atan2(dy, dx);
            double yaw_error = angle_to_goal - current_yaw_;

            while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

            feedback->remaining_angle = yaw_error;
            feedback->state = (distance < 0.1) ? 2 : (std::abs(yaw_error) > 0.1 ? 0 : 1);
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "dist=%.2f yaw_err=%.2f", distance, yaw_error);

            geometry_msgs::msg::Twist cmd;

            if (distance < 0.1)
            {
                // Final orientation alignment 
                double final_yaw_error = goal_theta_ - current_yaw_;
                while (final_yaw_error > M_PI) final_yaw_error -= 2 * M_PI;
                while (final_yaw_error < -M_PI) final_yaw_error += 2 * M_PI;

                if (std::abs(final_yaw_error) > 0.05)
                {
                    cmd.angular.z = std::clamp(1.5 * final_yaw_error, -0.5, 0.5);
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "Final alignment: error=%.2f", final_yaw_error);
                }
                else
                {
                    // Goal reached 
                    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
                    if (running_)
                    {
                        result->success = true;
                        result->message = "Goal reached";
                        goal_handle->succeed(result);
                    }
                    running_ = false;
                    return;
                }
            }
            else
            {
                // Move toward goal 
                double angle_to_goal = std::atan2(dy, dx);
                double yaw_error = angle_to_goal - current_yaw_;
                while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
                while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

                // Proportional controller 
                cmd.linear.x = std::clamp(0.5 * distance, -0.3, 0.3);
                cmd.angular.z = std::clamp(1.2 * yaw_error, -0.8, 0.8);
            }

            cmd_vel_pub_->publish(cmd);
            rate.sleep();
        }

        // Stop robot 
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
    }
};

} 

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_action::NavigationServer)