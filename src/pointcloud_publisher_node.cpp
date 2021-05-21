#include <cmath>

#include "pointcloud_pub_sub/pointcloud_publisher.hpp"

// PCL
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::shared_ptr<PointCloudPublisher>(new PointCloudPublisher(rclcpp::NodeOptions()));
  rclcpp::Rate loop_rate(node->getRate());
  const std::string executor_type= node->getExecutorType();

  if(executor_type == "single_thread")
  {
    node->setTimer();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }else if(executor_type == "static_single_thread")
  {
    node->setTimer();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

  }else if(executor_type == "multi_thread")
  {
    node->setTimer();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }else if(executor_type == "controlled_loop")
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while(rclcpp::ok())
    {
      loop_rate.sleep();
      node->publish();
      executor.spin_some();
    }
  }
  else{
    RCLCPP_ERROR(node->get_logger(), "invalid executor!!!");
  }
  rclcpp::shutdown();
}
