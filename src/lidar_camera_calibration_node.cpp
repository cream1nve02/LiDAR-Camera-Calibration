#include <lidar_camera_calibration/lidar_camera_calibration.h>

/**
 * @brief 메인 함수 - ROS 노드 시작점
 * @param argc 명령줄 인자 개수
 * @param argv 명령줄 인자 배열
 * @return 종료 코드
 */
int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "lidar_camera_calibration_node");
    
    // 라이다-카메라 캘리브레이션 객체 생성
    lidar_camera_calibration::LidarCameraCalibration calibration;
    
    // ROS 메시지 처리
    ros::spin();
    
    return 0;
} 