#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

        publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("diffbot_pose", 1);

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

        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = baseFrame;

        pose.pose.position.x = t.transform.translation.x;
        pose.pose.position.y = t.transform.translation.y;
        pose.pose.position.z = t.transform.translation.z;

        pose.pose.orientation.w = t.transform.rotation.w;
        pose.pose.orientation.x = t.transform.rotation.x;
        pose.pose.orientation.y = t.transform.rotation.y;
        pose.pose.orientation.z = t.transform.rotation.z;


        publisher->publish(pose);

    }

    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::string targetFrame;
    std::string baseFrame;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher{nullptr};

    rclcpp::TimerBase::SharedPtr timer{nullptr};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseListener>());
    rclcpp::shutdown();

    return 0;
}
