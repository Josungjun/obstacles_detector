#include <rclcpp/rclcpp.hpp>
#include "obstacle_detector/obstacle_visualizer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<obstacle_detector::ObstacleVisualizer>());
  rclcpp::shutdown();
  return 0;
}
