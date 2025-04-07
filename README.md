# LiDAR-Camera Calibration ROS Package

This package provides a ROS node that projects LiDAR point cloud data onto camera images.

## Features
- Calculate calibration transformation matrix between LiDAR and camera
- Project LiDAR point cloud onto camera images
- Publish the projected result image
- Interactive calibration GUI for fine-tuning parameters
- Point cloud filtering with adjustable parameters
- Distance-based color visualization

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
   - input: `/ouster/points` (PointCloud2) - 3D LiDAR
   - input: `/usb_cam/image_raw/compressed` (CompressedImage) - Camera
   - output: `/lidar_camera_calibration/projection` (Image) - Image with point cloud projected

## Parameters
- `camera/width`, `camera/height`: Camera image resolution
- `camera/fov`: Camera Field of View
- `camera/x`, `camera/y`, `camera/z`: Camera position relative to the vehicle coordinate system
- `camera/roll`, `camera/pitch`, `camera/yaw`: Camera orientation (RPY)
- `lidar/x`, `lidar/y`, `lidar/z`: LiDAR position relative to the vehicle coordinate system
- `lidar/roll`, `lidar/pitch`, `lidar/yaw`: LiDAR orientation (RPY)

## Interactive Calibration GUI
The package provides two GUI windows for interactive calibration:

1. **LiDAR-Camera Calibration Window**:
   - Adjust Camera position (X, Y, Z) in cm
   - Adjust Camera orientation (Roll, Pitch, Yaw) in degrees
   - Adjust LiDAR position (X, Y, Z) in cm
   - Adjust LiDAR orientation (Roll, Pitch, Yaw) in degrees
   - Save calibration parameters

2. **Point Cloud Filter Settings Window**:
   - Adjust Voxel Size (cm): Reduces point density using voxel grid filter
   - Adjust Max Distance (10cm units): Filters points beyond specified distance
   - Adjust Point Skip: Uses every n-th point to reduce density

## Recent Updates
- Added point cloud filtering capabilities with adjustable parameters
- Added interactive GUI for real-time calibration parameter adjustments
- Implemented distance-based color visualization with color legend
- Added save functionality for calibration parameters
- English UI/GUI for improved accessibility
- Added filtering statistics and display information overlay

## Calibration Parameter Format
After calibration, parameters can be saved in this format:
```yaml
# Calibration parameters - Saved at: 2025-04-07 22:05:50
# Camera parameter settings
camera:
  width: 640          # Image width (pixels)
  height: 480         # Image height (pixels)
  fov: 90           # Field of View - degrees
  x: 0.97              # Vehicle-frame X position (meters)
  y: 0.00              # Vehicle-frame Y position (meters)
  z: 1.42              # Vehicle-frame Z position (meters)
  roll: 0           # Roll angle (radians) - X-axis rotation
  pitch: 0          # Pitch angle (radians) - Y-axis rotation
  yaw: 0            # Yaw angle (radians) - Z-axis rotation

# LiDAR parameter settings
lidar:
  x: 1.39              # Vehicle-frame X position (meters)
  y: 0.00              # Vehicle-frame Y position (meters)
  z: 0.95             # Vehicle-frame Z position (meters)
  roll: 0           # Roll angle (radians) - X-axis rotation
  pitch: 0          # Pitch angle (radians) - Y-axis rotation
  yaw: 0           # Yaw angle (radians) - Z-axis rotation
```

## Creator
Chaemin Park
