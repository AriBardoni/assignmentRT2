#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace navigation_action
{

class RobotBroadcaster : public rclcpp::Node
{
public:
    explicit RobotBroadcaster(const rclcpp::NodeOptions & options)
        : Node("robot_broadcaster", options)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = msg->header.stamp;
                t.header.frame_id = "odom";
                t.child_frame_id = "base_footprint";  // o "link_chassis"
                t.transform.translation.x = msg->pose.pose.position.x;
                t.transform.translation.y = msg->pose.pose.position.y;
                t.transform.translation.z = msg->pose.pose.position.z;
                t.transform.rotation = msg->pose.pose.orientation;
                tf_broadcaster_->sendTransform(t);
            });
        
        RCLCPP_INFO(get_logger(), "Robot Broadcaster started");
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

} // namespace navigation_action

RCLCPP_COMPONENTS_REGISTER_NODE(navigation_action::RobotBroadcaster)