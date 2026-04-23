#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_navigation/action/navigate.hpp"

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
        // Create the action server and bind callbacks for goal handling
        action_server_ = rclcpp_action::create_server<Navigate>(
            this,
            "navigate",
            std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Navigation Action Server (component) started");
    }

private:

    // Action server instance
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;

    // Callback executed when a new goal is received
    // This function decides whether to accept or reject the goal
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Navigate::Goal> goal)
    {
        RCLCPP_INFO(get_logger(),
            "Goal received: x=%.2f y=%.2f theta=%.2f",
            goal->x, goal->y, goal->theta);

        // Accept all valid goals and start execution
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Callback executed when a cancel request is received
    // This allows preemption of the current goal
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate>)
    {
        RCLCPP_WARN(get_logger(), "Cancel requested");

        // Always accept cancel requests in this simple implementation
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Callback executed when a goal is accepted
    // The execution is moved to a separate thread to avoid blocking
    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        std::thread{
            std::bind(&NavigationServer::execute, this, goal_handle)
        }.detach();
    }

    // Main execution loop for the navigation task
    // Simulates motion toward the goal and publishes feedback
    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        // Retrieve goal data (target position and orientation)
        auto goal = goal_handle->get_goal();

        // Feedback and result messages
        auto feedback = std::make_shared<Navigate::Feedback>();
        auto result = std::make_shared<Navigate::Result>();

        rclcpp::Rate rate(10); // Execution frequency (10 Hz)

        // Simulated motion loop
        for (int i = 0; i <= 100; i++)
        {
            // Check if a cancel request has been issued
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);

                RCLCPP_WARN(get_logger(), "Goal canceled");
                return;
            }

            // Compute intermediate robot state (linear interpolation)
            feedback->current_x = goal->x * i / 100.0;
            feedback->current_y = goal->y * i / 100.0;
            feedback->current_theta = goal->theta * i / 100.0;

            // Publish feedback to the client
            goal_handle->publish_feedback(feedback);

            rate.sleep();
        }

        // Mark goal as successfully completed
        result->success = true;
        goal_handle->succeed(result);

        RCLCPP_INFO(get_logger(), "Goal reached");
    }
};

} // namespace rt2_navigation

// Register the class as a ROS2 component so it can be loaded in a container
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_navigation::NavigationServer)