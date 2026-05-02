#include <memory>
#include <thread>
#include <iostream>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navigation_action/action/navigate.hpp"

using Navigate = navigation_action::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

// Action client 
class NavigationUIClient : public rclcpp::Node
{
public:
    NavigationUIClient()
    : Node("navigation_ui_client"), running_(true)
    {
        // Creating the client connected to the server 
        client_ = rclcpp_action::create_client<Navigate>(this, "navigate");
        
        // Thread for user input 
        input_thread_ = std::thread([this]() {
            while (running_)
            {
                std::string cmd;
                std::cout << "\033[1;33m[INPUT] > \033[0m" << std::flush;
                std::getline(std::cin, cmd);
                
                if (cmd == "send") {
                    send_goal();
                } else if (cmd == "cancel") {
                    cancel_goal();
                } else if (cmd == "exit") {
                    running_ = false;
                    rclcpp::shutdown();
                }
            }
        });
        
        RCLCPP_INFO(get_logger(), "\033[1;36mUI ready: send | cancel | exit\033[0m");
    }
    
    ~NavigationUIClient()
    {
        running_ = false;
        if (input_thread_.joinable()) input_thread_.join();
    }

private:
    rclcpp_action::Client<Navigate>::SharedPtr client_;
    std::shared_ptr<GoalHandleNavigate> current_goal_;
    std::thread input_thread_;
    std::atomic<bool> running_;
    
    // Send new goal 
    void send_goal()
    {
        // Wait for server availability 
        if (!client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }
        
        auto goal = Navigate::Goal();
        std::cout << "Insert x y theta: " << std::flush;
        std::cin >> goal.x >> goal.y >> goal.theta;
        std::cin.ignore(); // pulisce il buffer
        
        auto opts = rclcpp_action::Client<Navigate>::SendGoalOptions();
        
        // Called when server responds to goal request 
        opts.goal_response_callback = 
            [this](const GoalHandleNavigate::SharedPtr & gh) {
                if (gh) {
                    current_goal_ = gh;
                    RCLCPP_INFO(get_logger(), "\033[1;32mGoal accepted\033[0m");
                } else {
                    RCLCPP_ERROR(get_logger(), "Goal rejected");
                }
            };
        
        // Called with navigation feedback 
        opts.feedback_callback =
            [this](GoalHandleNavigate::SharedPtr,
                   const std::shared_ptr<const Navigate::Feedback> fb) {
                // Stampa solo ogni tanto per non intasare
                static int counter = 0;
                if (counter++ % 10 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "Feedback: dist=%.2f angle=%.2f state=%d",
                        fb->remaining_distance, fb->remaining_angle, fb->state);
                }
            };
        
        // Called when goal completes 
        opts.result_callback =
            [this](const GoalHandleNavigate::WrappedResult & result) {
                current_goal_.reset();
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "\033[1;32mSUCCESS: %s\033[0m", result.result->message.c_str());
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_WARN(get_logger(), "Goal canceled");
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    RCLCPP_ERROR(get_logger(), "Goal aborted");
                }
            };
        
        client_->async_send_goal(goal, opts);
    }
    
    // Cancel the currently active goal 
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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}