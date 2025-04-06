cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_calibration)

# C++14 지원 설정
add_compile_options(-std=c++14)

# 디버그 모드 활성화
set(CMAKE_BUILD_TYPE Debug)

# 캐시킹(catkin) 패키지 의존성 설정
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  image_transport
  message_filters
)

# OpenCV 의존성 설정
find_package(OpenCV REQUIRED)
# PCL 의존성 설정
find_package(PCL REQUIRED)

# 캐시킹 패키지 설정
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge pcl_ros pcl_conversions image_transport message_filters
  DEPENDS OpenCV PCL
)

# include 디렉토리 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 링커 디렉토리 설정
link_directories(${PCL_LIBRARY_DIRS})

# 컴파일러 정의 추가
add_definitions(${PCL_DEFINITIONS})

# 실행 파일 빌드 설정 - lidar_camera_calibration.cpp 추가
add_executable(lidar_camera_calibration_node 
  src/lidar_camera_calibration_node.cpp
  src/lidar_camera_calibration.cpp
)

# 의존 라이브러리 연결
target_link_libraries(lidar_camera_calibration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
) 