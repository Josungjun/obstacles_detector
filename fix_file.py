import os

file_path = '/home/hello-sungjun/ros2_ws/src/obstacle_detector/src/obstacle_extractor.cpp'

with open(file_path, 'r') as f:
    content = f.read()

old_block = """// ... compareSegments, checkSegmentsProximity, checkSegmentsCollinearity, 
// detectCircles, mergeCircles, compareCircles 함수는 
// ROS API를 사용하지 않으므로 원본 코드 그대로 사용 가능 (생략) ...
// (하지만 실제 컴파일 시에는 위쪽 cpp 파일에 포함되어 있어야 함)

bool ObstacleExtractor::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
    // (이전 로직과 동일)
    if (&s1 == &s2) return false;
    if (s1.first_point.cross(s2.first_point) < 0.0) return compareSegments(s2, s1, merged_segment);
    if (checkSegmentsProximity(s1, s2)) {
        std::vector<PointSet> point_sets;
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

"""
new_block = """bool ObstacleExtractor::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2)) {
    std::vector<PointSet> point_sets;
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

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }

  // If circles intersect and are 'small' - merge
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
}"""

if old_block in content:
    new_content = content.replace(old_block, new_block)
    with open(file_path, 'w') as f:
        f.write(new_content)
    print("File updated successfully.")
else:
    print("Could not find the block to replace.")
    # Debug: print a part of content to see what's wrong
    print("Content snippet around expected location:")
    start_idx = content.find("void ObstacleExtractor::mergeSegments()")
    if start_idx != -1:
        print(content[start_idx:start_idx+1000])
