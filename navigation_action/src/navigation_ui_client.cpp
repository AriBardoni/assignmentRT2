#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_navigation/action/navigate.hpp"

using Navigate = rt2_navigation::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

namespace rt2_navigation
{

class NavigationUIClient : public rclcpp::Node
{
public:
    NavigationUIClient(const rclcpp::NodeOptions & options)
    : Node("navigation_ui_client", options)
    {
        // Create action client connected to "navigate" action server
        client_ = rclcpp_action::create_client<Navigate>(this, "navigate");

        RCLCPP_INFO(get_logger(), "Navigation UI Client started");

        // Simple CLI loop runs in separate thread to avoid blocking executor
        ui_thread_ = std::thread(std::bind(&NavigationUIClient::ui_loop, this));
    }

    ~NavigationUIClient()
    {
        if (ui_thread_.joinable())
            ui_thread_.join();
    }

private:

    rclcpp_action::Client<Navigate>::SharedPtr client_;
    std::shared_ptr<GoalHandleNavigate> current_goal_;
    std::thread ui_thread_;

    // Main user interface loop (CLI)
    void ui_loop()
    {
        while (rclcpp::ok())
        {
            std::string command;
            std::cout << "\nCommand (send / cancel / exit): ";
            std::cin >> command;

            if (command == "send")
            {
                send_goal();
            }
            else if (command == "cancel")
            {
                cancel_goal();
            }
            else if (command == "exit")
            {
                RCLCPP_INFO(get_logger(), "Exiting UI");
                rclcpp::shutdown();
                return;
            }
        }
    }

    // Send a navigation goal to the action server
    void send_goal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }

        auto goal = Navigate::Goal();

        std::cout << "Insert x y theta: ";
        std::cin >> goal.x >> goal.y >> goal.theta;

        auto send_options = rclcpp_action::Client<Navigate>::SendGoalOptions();

        // Feedback callback
        send_options.feedback_callback =
            [this](GoalHandleNavigate::SharedPtr,
                   const std::shared_ptr<const Navigate::Feedback> feedback)
            {
                RCLCPP_INFO(get_logger(),
                    "Feedback: x=%.2f y=%.2f theta=%.2f",
                    feedback->current_x,
                    feedback->current_y,
                    feedback->current_theta);
            };

        // Result callback
        send_options.result_callback =
            [this](const GoalHandleNavigate::WrappedResult & result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(get_logger(), "Goal succeeded");
                }
                else if (result.code == rclcpp_action::ResultCode::ABORTED)
                {
                    RCLCPP_WARN(get_logger(), "Goal aborted");
                }
                else if (result.code == rclcpp_action::ResultCode::CANCELED)
                {
                    RCLCPP_WARN(get_logger(), "Goal canceled");
                }
            };

        // Send goal asynchronously
        client_->async_send_goal(goal, send_options);
    }

    // Cancel the current goal
    void cancel_goal()
    {
        if (!client_)
        {
            RCLCPP_WARN(get_logger(), "Client not initialized");
            return;
        }

        RCLCPP_WARN(get_logger(), "Sending cancel request");
        client_->async_cancel_all_goals();
    }
};

} // namespace rt2_navigation

// Register as ROS2 component (required for container execution)
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_navigation::NavigationUIClient)