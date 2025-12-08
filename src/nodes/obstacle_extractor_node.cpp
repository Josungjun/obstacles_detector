/*
 * ROS 2 Ported Main Node
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "obstacle_detector/obstacle_extractor.hpp"

using namespace obstacle_detector;

int main(int argc, char** argv) {
  // 1. ROS 2 초기화
  rclcpp::init(argc, argv);

  // 2. NodeOptions 설정
  // (Launch 파일에서 전달하는 파라미터나 리매핑 정보를 받기 위해 필수입니다)
  rclcpp::NodeOptions options;

  try {
    // 3. 로깅 (노드 생성 전이라 임시 로거 사용)
    RCLCPP_INFO(rclcpp::get_logger("obstacle_extractor_node"), "[Obstacle Extractor]: Initializing node");

    // 4. 노드 객체 생성 (Shared Pointer 권장)
    // ROS 1: ObstacleExtractor od(nh, nh_local);
    // ROS 2: Node를 상속받았으므로 그냥 생성하면 됩니다.
    auto node = std::make_shared<ObstacleExtractor>(options);

    // 5. Spin (콜백 처리 루프)
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    // 표준 예외 처리
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_extractor_node"), "[Obstacle Extractor]: %s", e.what());
  }
  catch (const char* s) {
    // const char* 예외 처리 (원본 코드 대응)
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_extractor_node"), "[Obstacle Extractor]: %s", s);
  }
  catch (...) {
    // 그 외 모든 예외
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_extractor_node"), "[Obstacle Extractor]: Unexpected error");
  }

  // 6. 종료
  rclcpp::shutdown();
  return 0;
}