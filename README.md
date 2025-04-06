# 라이다-카메라 캘리브레이션 ROS 패키지

이 패키지는 라이다 포인트 클라우드 데이터를 카메라 이미지 위에 투영하는 ROS 노드를 제공합니다.

## 기능
- 라이다-카메라 간 캘리브레이션 변환 행렬 계산
- 라이다 포인트 클라우드를 카메라 이미지에 투영
- 투영된 결과 이미지 발행

## 의존성
- ROS (테스트 환경: ROS Noetic)
- OpenCV
- PCL (Point Cloud Library)
- Eigen

## 설치 방법
```bash
# ROS 워크스페이스의 src 디렉토리로 이동
cd ~/catkin_ws/src

# 패키지 복제
git clone https://github.com/your_username/lidar_camera_calibration.git

# 워크스페이스로 돌아가 빌드
cd ~/catkin_ws
catkin_make

# 워크스페이스 설정 소싱
source devel/setup.bash
```

## 사용 방법
1. 파라미터 설정: `config/calibration_params.yaml` 파일을 편집하여 카메라와 라이다의 위치 및 자세 파라미터를 조정합니다.

2. 런치 파일 실행:
```bash
roslaunch lidar_camera_calibration lidar_camera_calibration.launch
```

3. 필요한 토픽 확인:
   - 입력: `/velodyne_points` (PointCloud2) - 라이다 데이터
   - 입력: `/image_jpeg/compressed` (CompressedImage) - 카메라 이미지
   - 출력: `/lidar_camera_calibration/projection` (Image) - 포인트 클라우드가 투영된 이미지

## 파라미터 설명
- `camera/width`, `camera/height`: 카메라 이미지 해상도
- `camera/fov`: 카메라 시야각 (Field of View)
- `camera/x`, `camera/y`, `camera/z`: 차량 좌표계 기준 카메라 위치
- `camera/roll`, `camera/pitch`, `camera/yaw`: 카메라 자세 (RPY)
- `lidar/x`, `lidar/y`, `lidar/z`: 차량 좌표계 기준 라이다 위치
- `lidar/roll`, `lidar/pitch`, `lidar/yaw`: 라이다 자세 (RPY)

## 라이센스
MIT # LiDAR-Camera-Calibaration
