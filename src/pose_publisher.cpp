#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class PoseListener : public rclcpp::Node
{
public:
    PoseListener()
    : Node("pose_listener")
    {
        targetFrame = this->declare_parameter<std::string>("target_frame", "base_link");
        baseFrame = this->declare_parameter<std::string>("base_frame", "map");

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_pose", 1);

        timer = this->create_wall_timer(
            0.5s, std::bind(&PoseListener::onTimer, this));
    }
private:
    void onTimer()
    {
        geometry_msgs::msg::TransformStamped t;

        try {
          t = tfBuffer->lookupTransform(
            baseFrame, targetFrame,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform");
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
        publisher->publish(twist);
    }

    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::string targetFrame;
    std::string baseFrame;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher{nullptr};

    rclcpp::TimerBase::SharedPtr timer{nullptr};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseListener>());
    rclcpp::shutdown();

    return 0;
}
