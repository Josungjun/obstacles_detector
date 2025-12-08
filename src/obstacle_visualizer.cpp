#include "obstacle_detector/obstacle_visualizer.hpp"

namespace obstacle_detector
{

ObstacleVisualizer::ObstacleVisualizer(const rclcpp::NodeOptions & options)
: Node("obstacle_visualizer", options)
{
  obstacles_sub_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
    "raw_obstacles", 10,
    std::bind(&ObstacleVisualizer::obstaclesCallback, this, std::placeholders::_1));

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);
}

void ObstacleVisualizer::obstaclesCallback(const obstacle_detector::msg::Obstacles::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Segments
  if (!msg->segments.empty()) {
    visualization_msgs::msg::Marker segment_marker;
    segment_marker.header = msg->header;
    segment_marker.ns = "segments";
    segment_marker.id = 0;
    segment_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    segment_marker.action = visualization_msgs::msg::Marker::ADD;
    segment_marker.scale.x = 0.05; // Line width
    segment_marker.color.r = 0.0;
    segment_marker.color.g = 1.0;
    segment_marker.color.b = 0.0;
    segment_marker.color.a = 1.0;
    segment_marker.lifetime = rclcpp::Duration(0, 200000000); // 0.2s

    for (const auto & segment : msg->segments) {
      segment_marker.points.push_back(segment.first_point);
      segment_marker.points.push_back(segment.last_point);
    }
    marker_array.markers.push_back(segment_marker);
  }

  // Circles
  int id = 0;
  for (const auto & circle : msg->circles) {
    visualization_msgs::msg::Marker circle_marker;
    circle_marker.header = msg->header;
    circle_marker.ns = "circles";
    circle_marker.id = id++;
    circle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    circle_marker.action = visualization_msgs::msg::Marker::ADD;
    circle_marker.pose.position = circle.center;
    circle_marker.pose.position.z = 0.1; // Slightly elevated
    circle_marker.pose.orientation.w = 1.0;
    circle_marker.scale.x = circle.true_radius * 2.0;
    circle_marker.scale.y = circle.true_radius * 2.0;
    circle_marker.scale.z = 0.2; // Height
    circle_marker.color.r = 1.0;
    circle_marker.color.g = 0.0;
    circle_marker.color.b = 0.0;
    circle_marker.color.a = 0.8;
    circle_marker.lifetime = rclcpp::Duration(0, 200000000); // 0.2s

    marker_array.markers.push_back(circle_marker);
  }

  markers_pub_->publish(marker_array);
}

} // namespace obstacle_detector
