#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GuiIntegrationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    GuiIntegrationNode()
    : LifecycleNode("gui_integration_node")
    {
        state_server = this->create_service<std_srvs::srv::Trigger>(
            "trigger_service", std::bind(&GuiIntegrationNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));

        targetFrame = this->declare_parameter<std::string>("target_frame", "base_link");
        baseFrame = this->declare_parameter<std::string>("base_frame", "map");
        
        timeout_timer = this->create_wall_timer(
            1s, std::bind(&GuiIntegrationNode::gui_service_timeout, this));
        timeout_timer->cancel();  // Start with the timer cancelled
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override
    {
      (void)previous_state;

      nav_client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "follow_waypoints");  // Different action name

      tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

      publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_pose", 1);

      timer = this->create_wall_timer(
          std::chrono::milliseconds((int)(1000.0 / publish_frequency)), std::bind(&GuiIntegrationNode::posePublisher, this));

      timer->cancel();

      return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
      (void)previous_state;

      publisher.reset();
      timer.reset();

      return LifecycleCallbackReturn::SUCCESS;
    }


    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state)
    {

      RCLCPP_INFO(this->get_logger(), "Activating pose publisher");

      timer->reset();
      rclcpp_lifecycle::LifecycleNode::on_activate(previous_state); // Without manager?????

      return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
      
      timer->cancel();

      publisher_ready = false;

      rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);

      return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state)
    {
      (void)previous_state;

      publisher.reset();
      timer.reset();

      return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State& previous_state)
    {
      (void)previous_state;
      // HANDLE ERRORS IN THE CALLBACKS, this is rather unuseful
      RCLCPP_WARN(this->get_logger(), "Pose publisher error!");
      
      publisher.reset();
      timer.reset();
      
      return LifecycleCallbackReturn::FAILURE;
    }
private:
    void gui_service_timeout()
    {
        auto current_time = this->now();
        auto time_since_last_call = current_time - last_service_call;
        if (time_since_last_call > 6s) {
            RCLCPP_INFO(this->get_logger(), "No service call received for 10 seconds. Shutting down.");
            this->deactivate();
            cancelNavigation();
            // this->cleanup();
            timeout_timer->cancel();
        }
    }

    void trigger_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            this->configure();
        }
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            this->activate();
        }
        // Reset the timeout timer

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
          t = tfBuffer->lookupTransform(
            baseFrame, targetFrame,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform");
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
        if (nav_client) {
            RCLCPP_INFO(this->get_logger(), "Canceling waypoint following");
            auto cancel_future = nav_client->async_cancel_all_goals();
        }
    }

    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::string targetFrame;
    std::string baseFrame;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher{nullptr};

    rclcpp::TimerBase::SharedPtr timer{nullptr};
    double publish_frequency{4.0};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_server;

    rclcpp::TimerBase::SharedPtr timeout_timer{nullptr};
    rclcpp::Time last_service_call;

    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr nav_client;

    bool publisher_ready = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuiIntegrationNode>();

    rclcpp::spin(node->get_node_base_interface());
    
    rclcpp::shutdown();

    return 0;
}