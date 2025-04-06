# LiDAR-Camera Calibration ROS Package

This package provides a ROS node that projects LiDAR point cloud data onto camera images.

## Features
- Calculate calibration transformation matrix between LiDAR and camera
- Project LiDAR point cloud onto camera images
- Publish the projected result image

## Dependencies
- ROS (Tested on: ROS Noetic)
- OpenCV
- PCL (Point Cloud Library)
- Eigen

## Installation
```bash
# Navigate to the src directory of your ROS workspace
cd ~/catkin_ws/src

# Clone the package
git clone https://github.com/cream1nve02/lidar_camera_calibration.git

# Return to the workspace and build
cd ~/catkin_ws
catkin_make

# Source the workspace setup
source devel/setup.bash
```

## Usage
1. Set parameters: Edit the `config/calibration_params.yaml` file to adjust the position and orientation parameters of the camera and LiDAR.

2. Run the launch file:
```bash
roslaunch lidar_camera_calibration lidar_camera_calibration.launch
```

3. Required topics:
   - input: `/velodyne_points` (PointCloud2) - 3D LiDAR
   - input: `/image_jpeg/compressed` (CompressedImage) - Camera
   - output: `/lidar_camera_calibration/projection` (Image) - Image with point cloud projected

## Parameters
- `camera/width`, `camera/height`: Camera image resolution
- `camera/fov`: Camera Field of View
- `camera/x`, `camera/y`, `camera/z`: Camera position relative to the vehicle coordinate system
- `camera/roll`, `camera/pitch`, `camera/yaw`: Camera orientation (RPY)
- `lidar/x`, `lidar/y`, `lidar/z`: LiDAR position relative to the vehicle coordinate system
- `lidar/roll`, `lidar/pitch`, `lidar/yaw`: LiDAR orientation (RPY)

## Creator
Chaemin Park
# LiDAR-Camera-Calibration
