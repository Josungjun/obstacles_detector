#ifndef OBSTACLE_DETECTOR_OBSTACLE_VISUALIZER_HPP
#define OBSTACLE_DETECTOR_OBSTACLE_VISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "obstacle_detector/msg/obstacles.hpp"

namespace obstacle_detector
{

class ObstacleVisualizer : public rclcpp::Node
{
public:
  explicit ObstacleVisualizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void obstaclesCallback(const obstacle_detector::msg::Obstacles::SharedPtr msg);

  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

} // namespace obstacle_detector

#endif // OBSTACLE_DETECTOR_OBSTACLE_VISUALIZER_HPP
