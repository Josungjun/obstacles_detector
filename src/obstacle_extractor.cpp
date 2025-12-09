/*
 * ROS 2 Ported Source
 */

#include "obstacle_detector/obstacle_extractor.hpp"
#include "obstacle_detector/utilities/figure_fitting.h"
#include "obstacle_detector/utilities/math_utilities.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;
using namespace obstacle_detector;
using namespace std::chrono_literals; // for 100ms, etc.

ObstacleExtractor::ObstacleExtractor(const rclcpp::NodeOptions & options)
: Node("obstacle_extractor", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  // *** after find a param for clustering
  // p_active_ = false;

  // params_srv_ = this->create_service<std_srvs::srv::Empty>(
  //   "params", 
  //   std::bind(&ObstacleExtractor::updateParams, this, std::placeholders::_1, std::placeholders::_2));
  
  // initialize();

  p_active_ = this->declare_parameter("active", true);
  p_use_scan_ = this->declare_parameter("use_scan", false);
  p_use_pcl_ = this->declare_parameter("use_pcl", true);
  
  p_use_split_and_merge_ = this->declare_parameter("use_split_and_merge", true);
  p_circles_from_visibles_ = this->declare_parameter("circles_from_visibles", true);
  p_discard_converted_segments_ = this->declare_parameter("discard_converted_segments", true);
  p_transform_coordinates_ = this->declare_parameter("transform_coordinates", true);
  p_use_front_half_only_ = this->declare_parameter("use_front_half_only", false);

  p_min_group_points_ = this->declare_parameter("min_group_points", 5);

  p_max_group_distance_ = this->declare_parameter("max_group_distance", 0.1);
  p_distance_proportion_ = this->declare_parameter("distance_proportion", 0.00628);
  p_max_split_distance_ = this->declare_parameter("max_split_distance", 0.2);
  p_max_merge_separation_ = this->declare_parameter("max_merge_separation", 0.2);
  p_max_merge_spread_ = this->declare_parameter("max_merge_spread", 0.2);
  p_max_circle_radius_ = this->declare_parameter("max_circle_radius", 0.6);
  p_radius_enlargement_ = this->declare_parameter("radius_enlargement", 0.25);

  p_min_x_limit_ = this->declare_parameter("min_x_limit", -10.0);
  p_max_x_limit_ = this->declare_parameter("max_x_limit", 10.0);
  p_min_y_limit_ = this->declare_parameter("min_y_limit", -10.0);
  p_max_y_limit_ = this->declare_parameter("max_y_limit", 10.0);

  p_frame_id_ = this->declare_parameter("frame_id", std::string("map"));

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleExtractor::parametersCallback, this, std::placeholders::_1));

  updateIO();
}

ObstacleExtractor::~ObstacleExtractor() {
}

void ObstacleExtractor::updateParams(
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) 
{
  bool prev_active = p_active_;

  auto get_param = [&](const std::string& name, auto& variable, auto default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, default_value);
    }
    this->get_parameter(name, variable);
  };

  get_param("active", p_active_, true);
  get_param("use_scan", p_use_scan_, false);
  get_param("use_pcl", p_use_pcl_, true);

  get_param("use_split_and_merge", p_use_split_and_merge_, true);
  get_param("circles_from_visibles", p_circles_from_visibles_, true);
  get_param("discard_converted_segments", p_discard_converted_segments_, true);
  get_param("transform_coordinates", p_transform_coordinates_, true);
  get_param("use_front_half_only", p_use_front_half_only_, false);

  get_param("min_group_points", p_min_group_points_, 5);

  get_param("max_group_distance", p_max_group_distance_, 0.1);
  get_param("distance_proportion", p_distance_proportion_, 0.00628);
  get_param("max_split_distance", p_max_split_distance_, 0.2);
  get_param("max_merge_separation", p_max_merge_separation_, 0.2);
  get_param("max_merge_spread", p_max_merge_spread_, 0.2);
  get_param("max_circle_radius", p_max_circle_radius_, 0.6);
  get_param("radius_enlargement", p_radius_enlargement_, 0.25);

  get_param("min_x_limit", p_min_x_limit_, -10.0);
  get_param("max_x_limit", p_max_x_limit_,  10.0);
  get_param("min_y_limit", p_min_y_limit_, -10.0);
  get_param("max_y_limit", p_max_y_limit_,  10.0);

  get_param("frame_id", p_frame_id_, std::string("base_link"));

  if (p_active_ != prev_active) {
    if (p_active_) {
      rclcpp::QoS qos(10);

      if (p_use_scan_)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", qos, std::bind(&ObstacleExtractor::scanCallback, this, std::placeholders::_1));
      else if (p_use_pcl_)
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
          "pcl", qos, std::bind(&ObstacleExtractor::pclCallback, this, std::placeholders::_1));

      obstacles_pub_ = this->create_publisher<obstacle_detector::msg::Obstacles>("raw_obstacles", qos);
    }
    else {
      obstacle_detector::msg::Obstacles obstacles_msg;
      obstacles_msg.header.frame_id = p_frame_id_;
      obstacles_msg.header.stamp = this->now();
      obstacles_pub_->publish(obstacles_msg);
      scan_sub_.reset();
      pcl_sub_.reset();
      obstacles_pub_.reset(); 
    }
  }
}

void ObstacleExtractor::updateIO() {
  if (p_active_) {
    if (obstacles_pub_) return; 

    rclcpp::QoS qos(10);

    if (p_use_scan_) {
      scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos, std::bind(&ObstacleExtractor::scanCallback, this, std::placeholders::_1));
      // PCL 구독자는 꺼줌 (동시 실행 방지)
      pcl_sub_.reset(); 
    }
    else if (p_use_pcl_) {
      pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
        "pcl", qos, std::bind(&ObstacleExtractor::pclCallback, this, std::placeholders::_1));
      // Scan 구독자는 꺼줌
      scan_sub_.reset();
    }

    obstacles_pub_ = this->create_publisher<obstacle_detector::msg::Obstacles>("raw_obstacles", qos);
  }
  else {
    if (obstacles_pub_) {
        obstacle_detector::msg::Obstacles obstacles_msg;
        obstacles_msg.header.frame_id = p_frame_id_;
        obstacles_msg.header.stamp = this->now();
        obstacles_pub_->publish(obstacles_msg);
    }

    scan_sub_.reset();
    pcl_sub_.reset();
    obstacles_pub_.reset(); 
  }
}

rcl_interfaces::msg::SetParametersResult ObstacleExtractor::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool io_update_needed = false;

  for (const auto &param : parameters) {
    const std::string & name = param.get_name();

    if (name == "active") {
      p_active_ = param.as_bool();
      io_update_needed = true;
    }
    else if (name == "use_scan") {
      p_use_scan_ = param.as_bool();
      io_update_needed = true;
    }
    else if (name == "use_pcl") {
      p_use_pcl_ = param.as_bool();
      io_update_needed = true;
    }
    else if (name == "use_split_and_merge") {
      p_use_split_and_merge_ = param.as_bool();
    }
    else if (name == "circles_from_visibles") {
      p_circles_from_visibles_ = param.as_bool();
    }
    else if (name == "discard_converted_segments") {
      p_discard_converted_segments_ = param.as_bool();
    }
    else if (name == "transform_coordinates") {
      p_transform_coordinates_ = param.as_bool();
    }
    else if (name == "use_front_half_only") {
      p_use_front_half_only_ = param.as_bool();
    }
    else if (name == "min_group_points") {
      p_min_group_points_ = param.as_int();
    }
    else if (name == "max_group_distance") {
      p_max_group_distance_ = param.as_double();
    }
    else if (name == "distance_proportion") {
      p_distance_proportion_ = param.as_double();
    }
    else if (name == "max_split_distance") {
      p_max_split_distance_ = param.as_double();
    }
    else if (name == "max_merge_separation") {
      p_max_merge_separation_ = param.as_double();
    }
    else if (name == "max_merge_spread") {
      p_max_merge_spread_ = param.as_double();
    }
    else if (name == "max_circle_radius") {
      p_max_circle_radius_ = param.as_double();
    }
    else if (name == "radius_enlargement") {
      p_radius_enlargement_ = param.as_double();
    }
    else if (name == "min_x_limit") {
      p_min_x_limit_ = param.as_double();
    }
    else if (name == "max_x_limit") {
      p_max_x_limit_ = param.as_double();
    }
    else if (name == "min_y_limit") {
      p_min_y_limit_ = param.as_double();
    }
    else if (name == "max_y_limit") {
      p_max_y_limit_ = param.as_double();
    }
    else if (name == "frame_id") {
      p_frame_id_ = param.as_string();
    }
  }

  if (io_update_needed) {
    updateIO();
  }

  return result;
}

void ObstacleExtractor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  base_frame_id_ = scan_msg->header.frame_id;
  stamp_ = scan_msg->header.stamp;

  geometry_msgs::msg::TransformStamped transform_stamped;
  bool transform_valid = false;
  if (p_use_front_half_only_) {
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "base_link", base_frame_id_, 
        tf2::TimePointZero); 
      transform_valid = true;
    }
    catch (tf2::TransformException& ex) {
    }
  }

  double phi = scan_msg->angle_min;

  for (const float r : scan_msg->ranges) {
    if (r >= scan_msg->range_min && r <= scan_msg->range_max) {
      bool use_point = true;
      
      if (p_use_front_half_only_) {
        if (transform_valid) {
            double lx = r * cos(phi);
            double ly = r * sin(phi);

            geometry_msgs::msg::PointStamped ps_in, ps_out;
            ps_in.point.x = lx;
            ps_in.point.y = ly;
            ps_in.point.z = 0.0;
            
            tf2::doTransform(ps_in, ps_out, transform_stamped);

            if (ps_out.point.x < 0.0) 
                use_point = false;
        }
        else {
            if (cos(phi) < 0.0)
                use_point = false;
        }
      }

      if (use_point)
        input_points_.push_back(Point::fromPoolarCoords(r, phi)); 
    }

    phi += scan_msg->angle_increment;
  }

  processPoints();
}

void ObstacleExtractor::pclCallback(const sensor_msgs::msg::PointCloud::SharedPtr pcl_msg) {
  base_frame_id_ = pcl_msg->header.frame_id;
  stamp_ = pcl_msg->header.stamp;

  for (const auto& point : pcl_msg->points)
    input_points_.push_back(Point(point.x, point.y));

  processPoints();
}

void ObstacleExtractor::processPoints() {
  segments_.clear();
  circles_.clear();
  groupPoints();
  mergeSegments();

  detectCircles();
  mergeCircles();

  publishObstacles();

  input_points_.clear();
}

void ObstacleExtractor::groupPoints() {
  static double sin_dp = sin(2.0 * p_distance_proportion_);

  PointSet point_set;
  point_set.begin = input_points_.begin();
  point_set.end = input_points_.begin();
  point_set.num_points = 1;
  point_set.is_visible = true;

  for (auto point = std::next(input_points_.begin()); point != input_points_.end(); ++point) {
    double range = (*point).length();
    double distance = (*point - *point_set.end).length();

    if (distance < p_max_group_distance_ + range * p_distance_proportion_) {
      point_set.end = point;
      point_set.num_points++;
    }
    else {
      double prev_range = (*point_set.end).length();
      double p = (range + prev_range + distance) / 2.0;
      double S = sqrt(p * (p - range) * (p - prev_range) * (p - distance));
      double sin_d = 2.0 * S / (range * prev_range);

      if (abs(sin_d) < sin_dp && range < prev_range)
        point_set.is_visible = false;

      detectSegments(point_set);

      point_set.begin = point;
      point_set.end = point;
      point_set.num_points = 1;
      point_set.is_visible = (abs(sin_d) > sin_dp || range < prev_range);
    }
  }
  detectSegments(point_set);
}

void ObstacleExtractor::detectSegments(const PointSet& point_set) {
  if (point_set.num_points < p_min_group_points_) return;

  Segment segment(*point_set.begin, *point_set.end);
  if (p_use_split_and_merge_) segment = fitSegment(point_set);

  auto set_divider = point_set.begin;
  double max_distance = 0.0;
  double distance = 0.0;
  int split_index = 0;
  int point_index = 0;

  for (auto point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;
    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = (*point).length();
      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
        max_distance = distance;
        set_divider = point;
        split_index = point_index;
      }
    }
  }

  if (max_distance > 0.0 && split_index > p_min_group_points_ && split_index < point_set.num_points - p_min_group_points_) {
    set_divider = input_points_.insert(set_divider, *set_divider);
    
    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;
    subset1.is_visible = point_set.is_visible;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;
    subset2.is_visible = point_set.is_visible;

    detectSegments(subset1);
    detectSegments(subset2);
  } else {
    if (!p_use_split_and_merge_) segment = fitSegment(point_set);
    segments_.push_back(segment);
  }
}

void ObstacleExtractor::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment merged_segment;
      if (compareSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
  if (&s1 == &s2)
    return false;

  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2)) {
    vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (checkSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

bool ObstacleExtractor::checkSegmentsProximity(const Segment& s1, const Segment& s2) {
  return (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
          s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_);
}

bool ObstacleExtractor::checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2) {
  return (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
          segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s2.last_point)  < p_max_merge_spread_);
}

void ObstacleExtractor::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
    if (p_circles_from_visibles_) {
      bool segment_is_visible = true;
      for (const PointSet& ps : segment->point_sets) {
        if (!ps.is_visible) {
          segment_is_visible = false;
          break;
        }
      }
      if (!segment_is_visible)
        continue;
    }

    Circle circle(*segment);
    circle.radius += p_radius_enlargement_;

    if (circle.radius < p_max_circle_radius_) {
      circles_.push_back(circle);

      if (p_discard_converted_segments_) {
        segment = segments_.erase(segment);
        --segment;
      }
    }
  }
}

void ObstacleExtractor::mergeCircles() {
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    for (auto j = i; j != circles_.end(); ++j) {
      Circle merged_circle;

      if (compareCircles(*i, *j, merged_circle)) {
        auto temp_itr = circles_.insert(i, merged_circle);
        circles_.erase(i);
        circles_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle) {
  if (&c1 == &c2)
    return false;

  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }

  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }

  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius / (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += max(c1.radius, c2.radius);

    if (circle.radius < p_max_circle_radius_) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(), c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(), c2.point_sets.end());
      merged_circle = circle;
      return true;
    }
  }

  return false;
}

void ObstacleExtractor::publishObstacles() {
  auto obstacles_msg = std::make_unique<obstacle_detector::msg::Obstacles>();
  obstacles_msg->header.stamp = stamp_;

  if (p_transform_coordinates_) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
      transform_stamped = tf_buffer_->lookupTransform(
        p_frame_id_, base_frame_id_, 
        tf2::TimePointZero, 
        50ms); // timeout
    }
    catch (tf2::TransformException& ex) {
      RCLCPP_INFO_STREAM(this->get_logger(), ex.what());
      return;
    }

    auto transform_point = [&](const Point& p) -> Point {
        geometry_msgs::msg::PointStamped ps_in, ps_out;
        ps_in.point.x = p.x;
        ps_in.point.y = p.y;
        ps_in.point.z = 0.0;
        
        tf2::doTransform(ps_in, ps_out, transform_stamped);
        return Point(ps_out.point.x, ps_out.point.y);
    };

    for (Segment& s : segments_) {
      s.first_point = transform_point(s.first_point);
      s.last_point = transform_point(s.last_point);
    }

    for (Circle& c : circles_)
      c.center = transform_point(c.center);

    obstacles_msg->header.frame_id = p_frame_id_;
  }
  else {
    obstacles_msg->header.frame_id = base_frame_id_;
  }

  for (const Segment& s : segments_) {
    obstacle_detector::msg::SegmentObstacle segment;
    segment.first_point.x = s.first_point.x;
    segment.first_point.y = s.first_point.y;
    segment.last_point.x = s.last_point.x;
    segment.last_point.y = s.last_point.y;
    obstacles_msg->segments.push_back(segment);
  }

  for (const Circle& c : circles_) {
    if (c.center.x > p_min_x_limit_ && c.center.x < p_max_x_limit_ &&
        c.center.y > p_min_y_limit_ && c.center.y < p_max_y_limit_) {
        obstacle_detector::msg::CircleObstacle circle;
        circle.center.x = c.center.x;
        circle.center.y = c.center.y;
        circle.velocity.x = 0.0;
        circle.velocity.y = 0.0;
        circle.radius = c.radius;
        circle.true_radius = c.radius - p_radius_enlargement_;
        obstacles_msg->circles.push_back(circle);
    }
  }

  obstacles_pub_->publish(std::move(obstacles_msg));
}

void ObstacleExtractor::initialize() { 
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  auto res = std::make_shared<std_srvs::srv::Empty::Response>();
  updateParams(req, res); 
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detector::ObstacleExtractor)