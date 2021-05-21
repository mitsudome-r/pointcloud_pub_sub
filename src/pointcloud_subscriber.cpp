#include <cmath>
#include "pointcloud_pub_sub/pointcloud_subscriber.hpp"

// PCL
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp_components/register_node_macro.hpp"


PointCloudSubscriber::PointCloudSubscriber(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("pointcloud_subscriber", node_options),
  count(0)
{
    // parameters
    executor_type_ = this->declare_parameter("executor_type", "single_thread");
    publish_received_message_ = this->declare_parameter("publish_received_message", true);

    // pub/sub
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("received_pointcloud2", 10);
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud2", 10,
        std::bind(&PointCloudSubscriber::onPointCloud, this, std::placeholders::_1));
}

void PointCloudSubscriber::onPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
{
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000 /* ms */,
        "recieved pointcloud");
    if(publish_received_message_)
    {
      pointcloud_publisher_->publish(*msg_ptr);
    }
}

std::string PointCloudSubscriber::getExecutorType()
{
  return executor_type_;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudSubscriber)
