#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudPublisher: public rclcpp::Node{
public:
    PointCloudPublisher(const rclcpp::NodeOptions & node_options);
    void publish();
    void setTimer();
    double getRate();
    std::string getExecutorType();
private:
    int width_;
    int length_;
    double rate_;
    std::string executor_type_;
    std::string frame_id_;
    sensor_msgs::msg::PointCloud2 pointcloud_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    void onTimer();
};
