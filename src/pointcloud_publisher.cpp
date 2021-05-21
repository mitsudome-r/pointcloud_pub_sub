#include <cmath>
#include "pointcloud_pub_sub/pointcloud_publisher.hpp"

// PCL
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp_components/register_node_macro.hpp"


PointCloudPublisher::PointCloudPublisher(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("pointcloud_publisher", node_options)
{
    // parameters
    rate_ = this->declare_parameter("rate", 10.0);
    width_ = this->declare_parameter("width", 100);
    length_ = this->declare_parameter("length", 100);
    frame_id_ = this->declare_parameter("frame_id", "map");
    executor_type_ = this->declare_parameter("executor_type", "single_thread");

    // pub/sub
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud2", 10);

    // pointcloud initialization
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.points.resize(width_ * length_);
    for (int x = 0; x < width_; x++)
    {
      for (int y = 0; y < length_; y++)
      {
        pcl::PointXYZI tmp_point;
        tmp_point.x = static_cast<float>(x / 100.0);
        tmp_point.y = static_cast<float>(y / 100.0);
        tmp_point.z = static_cast<float>(((x + y) % 100) / 100.0f);
        tmp_point.intensity = 1.0;
        cloud.push_back(tmp_point);
      }
    }

    pcl::toROSMsg(cloud, pointcloud_msg_);
    pointcloud_msg_.header.frame_id = frame_id_;
}

void PointCloudPublisher::setTimer()
{
    auto timer_callback = std::bind(&PointCloudPublisher::onTimer, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void PointCloudPublisher::publish()
{
    pointcloud_msg_.header.stamp = this->now();
    pointcloud_publisher_->publish(pointcloud_msg_);
}

void PointCloudPublisher::onTimer()
{
  publish();
}

double PointCloudPublisher::getRate()
{
  return rate_;
}

std::string PointCloudPublisher::getExecutorType()
{
  return executor_type_;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudPublisher)
