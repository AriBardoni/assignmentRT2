#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_navigation/action/navigate.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using Navigate = rt2_navigation::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

namespace rt2_navigation
{

class NavigationServer : public rclcpp::Node
{
public:
    NavigationServer(const rclcpp::NodeOptions & options)
    : Node("navigation_action_server", options)
    {
        // Action server
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

        // TF broadcaster
        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(),
                    "Navigation Action Server (component) started");
    }

private:

    // ================= ACTION SERVER =================
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ================= GOAL HANDLER =================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Navigate::Goal> goal)
    {
        RCLCPP_INFO(get_logger(),
            "Goal received: x=%.2f y=%.2f theta=%.2f",
            goal->x, goal->y, goal->theta);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // ================= CANCEL HANDLER =================
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate>)
    {
        RCLCPP_WARN(get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // ================= ACCEPT GOAL =================
    void handle_accepted(
        const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        std::thread{
            std::bind(&NavigationServer::execute, this, goal_handle)
        }.detach();
    }

    // ================= EXECUTION =================
    void execute(
        const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<Navigate::Feedback>();
        auto result = std::make_shared<Navigate::Result>();

        rclcpp::Rate rate(10);

        for (int i = 0; i <= 100; i++)
        {
            // cancel check
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);

                RCLCPP_WARN(get_logger(), "Goal canceled");
                return;
            }

            // simple interpolation movement
            feedback->current_x = goal->x * i / 100.0;
            feedback->current_y = goal->y * i / 100.0;
            feedback->current_theta = goal->theta * i / 100.0;

            // ================= TF BROADCAST =================
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->now();
            t.header.frame_id = "map";
            t.child_frame_id = "base_link";

            t.transform.translation.x = feedback->current_x;
            t.transform.translation.y = feedback->current_y;
            t.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, feedback->current_theta);

            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);

            // publish feedback
            goal_handle->publish_feedback(feedback);

            rate.sleep();
        }

        result->success = true;
        goal_handle->succeed(result);

        RCLCPP_INFO(get_logger(), "Goal reached");
    }
};

} // namespace rt2_navigation

// register component
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_navigation::NavigationServer)