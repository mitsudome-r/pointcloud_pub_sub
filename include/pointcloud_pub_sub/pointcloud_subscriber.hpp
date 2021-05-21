#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudSubscriber: public rclcpp::Node{
public:
    PointCloudSubscriber(const rclcpp::NodeOptions & node_options);
    std::string getExecutorType();
private:
    std::string executor_type_;
    bool publish_received_message_;
    int count;
    // sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    void onPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
};
