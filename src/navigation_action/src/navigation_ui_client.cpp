/*#include <memory>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "navigation_action/action/navigate.hpp"

using Navigate = navigation_action::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

class NavigationUIClient : public rclcpp::Node
{
public:
    NavigationUIClient()
    : Node("navigation_ui_client")
    {
        client_ = rclcpp_action::create_client<Navigate>(this, "navigate");
        RCLCPP_INFO(get_logger(), "UI Client ready. Commands: send, cancel, exit");
    }
    
    void run_ui()
    {
        std::string command;
        while (rclcpp::ok())
        {
            std::cout << "\n> ";
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
                rclcpp::shutdown();
                return;
            }
        }
    }

private:
    rclcpp_action::Client<Navigate>::SharedPtr client_;
    std::shared_ptr<GoalHandleNavigate> current_goal_;
    
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
        
        auto opts = rclcpp_action::Client<Navigate>::SendGoalOptions();
        
        opts.goal_response_callback = 
            [this](const GoalHandleNavigate::SharedPtr & gh) {
                if (gh) {
                    current_goal_ = gh;
                    RCLCPP_INFO(get_logger(), "Goal accepted");
                } else {
                    RCLCPP_ERROR(get_logger(), "Goal rejected");
                }
            };
        
        opts.feedback_callback =
            [this](GoalHandleNavigate::SharedPtr,
                   const std::shared_ptr<const Navigate::Feedback> fb) {
                RCLCPP_INFO(get_logger(),
                    "Feedback: remaining_dist=%.2f remaining_angle=%.2f state=%d",
                    fb->remaining_distance, fb->remaining_angle, fb->state);
            };
        
        opts.result_callback =
            [this](const GoalHandleNavigate::WrappedResult & result) {
                current_goal_.reset();
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Goal succeeded: %s", result.result->message.c_str());
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_WARN(get_logger(), "Goal canceled");
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    RCLCPP_ERROR(get_logger(), "Goal aborted");
                }
            };
        
        client_->async_send_goal(goal, opts);
    }
    
    void cancel_goal()
    {
        if (current_goal_) {
            RCLCPP_WARN(get_logger(), "Cancelling goal...");
            client_->async_cancel_goal(current_goal_);
        } else {
            RCLCPP_WARN(get_logger(), "No active goal");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationUIClient>();
    std::thread ui_thread(&NavigationUIClient::run_ui, node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/

#include <memory>
#include <thread>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navigation_action/action/navigate.hpp"

using Navigate = navigation_action::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

class NavigationUIClient : public rclcpp::Node
{
public:
    NavigationUIClient()
    : Node("navigation_ui_client")
    {
        client_ = rclcpp_action::create_client<Navigate>(this, "navigate");
        RCLCPP_INFO(get_logger(), "UI ready: send | cancel | exit");
    }

    void run_ui()
    {
        std::string cmd;

        while (rclcpp::ok())
        {
            std::cout << "\n> ";
            std::cin >> cmd;

            if (cmd == "send") send_goal();
            else if (cmd == "cancel") cancel_goal();
            else if (cmd == "exit") break;
        }
    }

private:
    rclcpp_action::Client<Navigate>::SharedPtr client_;
    std::shared_ptr<GoalHandleNavigate> current_goal_;

    void send_goal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "Server not available");
            return;
        }

        Navigate::Goal goal;

        std::cout << "Insert x y theta: ";
        std::cin >> goal.x >> goal.y >> goal.theta;

        auto opts = rclcpp_action::Client<Navigate>::SendGoalOptions();

        opts.goal_response_callback =
            [this](const GoalHandleNavigate::SharedPtr & gh)
            {
                if (gh)
                {
                    current_goal_ = gh;
                    RCLCPP_INFO(get_logger(), "Goal accepted");
                }
            };

        opts.feedback_callback =
            [this](GoalHandleNavigate::SharedPtr,
                   const std::shared_ptr<const Navigate::Feedback> fb)
            {
                RCLCPP_INFO(get_logger(),
                    "Feedback: dist=%.2f angle=%.2f state=%d",
                    fb->remaining_distance,
                    fb->remaining_angle,
                    fb->state);
            };

        opts.result_callback =
            [this](const GoalHandleNavigate::WrappedResult & res)
            {
                current_goal_.reset();

                if (res.code == rclcpp_action::ResultCode::SUCCEEDED)
                    RCLCPP_INFO(get_logger(), "SUCCESS: %s", res.result->message.c_str());
            };

        client_->async_send_goal(goal, opts);
    }

    void cancel_goal()
    {
        if (current_goal_)
            client_->async_cancel_goal(current_goal_);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NavigationUIClient>();
    std::thread ui_thread(&NavigationUIClient::run_ui, node);

    rclcpp::spin(node);

    ui_thread.join();   // FIX CRASH

    rclcpp::shutdown();
    return 0;
}
