#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

/* This node publishes pose for the gui, as well as has watchdog timers to check health of robot-hmi connection */
/* Stops the robot when the connection is severed */
class GuiIntegrationNode : public rclcpp::Node
{
public:
    GuiIntegrationNode()
    : Node("gui_integration_node")
    {
        targetFrame = this->declare_parameter<std::string>("target_frame", "base_link");
        baseFrame = this->declare_parameter<std::string>("base_frame", "map");

        nav_waypoint_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "follow_waypoints");
        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_pose", 1);

        timer = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / publish_frequency)),
            std::bind(&GuiIntegrationNode::posePublisher, this));

        heartbeat_service_ = this->create_service<std_srvs::srv::Trigger>(
            "heartbeat_service", std::bind(&GuiIntegrationNode::heartbeat_callback, this, std::placeholders::_1, std::placeholders::_2));

        timeout_timer = this->create_wall_timer(
            1s, std::bind(&GuiIntegrationNode::gui_service_timeout, this));
        timeout_timer->cancel();
    }

private:
    void gui_service_timeout()
    {
        auto time_since_last_call = this->now() - last_service_call;
        if (time_since_last_call > 2s) {
            RCLCPP_INFO(this->get_logger(), "No service call received for 6 seconds. Canceling navigation.");
            cancelNavigation();
            timeout_timer->cancel();
        }
    }

    void heartbeat_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        last_service_call = this->now();
        if (timeout_timer->is_canceled()) {
            RCLCPP_INFO(this->get_logger(), "Timeout timer start");
            timeout_timer->reset();
        }

        response->success = publisher_ready;
        response->message = "Service executed successfully!";
    }

    void posePublisher()
    {
        geometry_msgs::msg::TransformStamped t;

        try {
            t = tfBuffer->lookupTransform(baseFrame, targetFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform");
            publisher_ready = false;
            return;
        }

        geometry_msgs::msg::TwistStamped twist;

        twist.header.stamp = this->get_clock()->now();
        twist.header.frame_id = baseFrame;

        twist.twist.linear.x = t.transform.translation.x;
        twist.twist.linear.y = t.transform.translation.y;
        twist.twist.linear.z = t.transform.translation.z;

        double x = t.transform.rotation.x;
        double y = t.transform.rotation.y;
        double z = t.transform.rotation.z;
        double w = t.transform.rotation.w;

        twist.twist.angular.z = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
        publisher_ready = true;
        publisher->publish(twist);
    }

    void cancelNavigation()
    {
        if (nav_waypoint_client_->action_server_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "Canceling waypoint following");
            nav_waypoint_client_->async_cancel_all_goals();
        }
        if (nav_to_pose_client_->action_server_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "Canceling navigate to pose");
            nav_to_pose_client_->async_cancel_all_goals();
        }
    }

    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::string targetFrame;
    std::string baseFrame;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher{nullptr};

    rclcpp::TimerBase::SharedPtr timer{nullptr};
    double publish_frequency{15.0};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr heartbeat_service_;

    rclcpp::TimerBase::SharedPtr timeout_timer{nullptr};
    rclcpp::Time last_service_call;

    // JUST FOR EMERGENCIES WHEN HMI STOPS WORKING
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr nav_waypoint_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;

    bool publisher_ready = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuiIntegrationNode>());
    rclcpp::shutdown();
    return 0;
}
