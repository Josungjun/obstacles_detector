/*
 * ROS 2 Ported Header
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"


#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" 
#include "sensor_msgs/msg/point_cloud.hpp" 

#include "obstacle_detector/msg/obstacles.hpp"

#include "obstacle_detector/utilities/point.h"
#include "obstacle_detector/utilities/segment.h"
#include "obstacle_detector/utilities/circle.h"
#include "obstacle_detector/utilities/point_set.h"

namespace obstacle_detector
{

class ObstacleExtractor : public rclcpp::Node
{
public:
  explicit ObstacleExtractor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ObstacleExtractor();

private:
  
  void updateParams(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void pclCallback(const sensor_msgs::msg::PointCloud::SharedPtr pcl_msg);

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  void initialize();

  void processPoints();
  void groupPoints();
  void publishObstacles();

  void detectSegments(const PointSet& point_set);
  void mergeSegments();
  bool compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment);
  bool checkSegmentsProximity(const Segment& s1, const Segment& s2);
  bool checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2);

  void detectCircles();
  void mergeCircles();
  bool compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle);
  
  void updateIO();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pcl_sub_;
  rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr obstacles_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr params_srv_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time stamp_; 
  std::string base_frame_id_;

  std::list<Point> input_points_;
  std::list<Segment> segments_;
  std::list<Circle> circles_;

  bool p_active_;
  bool p_use_scan_;
  bool p_use_pcl_;

  bool p_use_split_and_merge_;
  bool p_circles_from_visibles_;
  bool p_discard_converted_segments_;
  bool p_transform_coordinates_;
  bool p_use_front_half_only_;

  int p_min_group_points_;

  double p_distance_proportion_;
  double p_max_group_distance_;
  double p_max_split_distance_;
  double p_max_merge_separation_;
  double p_max_merge_spread_;
  double p_max_circle_radius_;
  double p_radius_enlargement_;

  double p_min_x_limit_;
  double p_max_x_limit_;
  double p_min_y_limit_;
  double p_max_y_limit_;

  std::string p_frame_id_;
};

}
