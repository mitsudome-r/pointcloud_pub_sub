#include <cmath>

#include "pointcloud_pub_sub/pointcloud_subscriber.hpp"

// PCL
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::shared_ptr<PointCloudSubscriber>(new PointCloudSubscriber(rclcpp::NodeOptions()));
  const std::string executor_type= node->getExecutorType();

  if(executor_type == "single_thread")
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }else if(executor_type == "static_single_thread")
  {
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

  }else if(executor_type == "multi_thread")
  {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }else if(executor_type == "controlled_loop")
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::Rate loop_rate(50);

    while(rclcpp::ok())
    {
      executor.spin_some();
      loop_rate.sleep();
    }
  }
  else{
    RCLCPP_ERROR(node->get_logger(), "invalid executor!!!");
  }
  rclcpp::shutdown();
}
