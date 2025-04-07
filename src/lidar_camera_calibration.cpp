#include <lidar_camera_calibration/lidar_camera_calibration.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <ctime>
#include <ros/package.h>
#include <limits>
#include <pcl/filters/voxel_grid.h>

namespace lidar_camera_calibration {

// Global instance pointer for trackbar callbacks
static LidarCameraCalibration* g_instance = nullptr;

// Filtering option callback functions
void onVoxelSizeChanged(int value, void*);
void onMaxDistanceChanged(int value, void*);
void onSkipPointsChanged(int value, void*);

// Callback function declarations (within namespace)
void onCameraXChanged(int value, void*);
void onCameraYChanged(int value, void*);
void onCameraZChanged(int value, void*);
void onCameraRollChanged(int value, void*);
void onCameraPitchChanged(int value, void*);
void onCameraYawChanged(int value, void*);
void onLidarXChanged(int value, void*);
void onLidarYChanged(int value, void*);
void onLidarZChanged(int value, void*);
void onLidarRollChanged(int value, void*);
void onLidarPitchChanged(int value, void*);
void onLidarYawChanged(int value, void*);
void onSaveButtonChanged(int value, void*);

// Filtering callback functions implementation
void onVoxelSizeChanged(int value, void*) {
    if (g_instance) g_instance->setVoxelSize(value / 100.0f);  // cm -> m
}

void onMaxDistanceChanged(int value, void*) {
    if (g_instance) g_instance->setMaxDistance(value / 10.0f);  // 10cm units
}

void onSkipPointsChanged(int value, void*) {
    if (g_instance) g_instance->setPointSkip(value);
}

// Callback function implementations
void onCameraXChanged(int value, void*) {
    if (g_instance) g_instance->updateCameraParameter(value, 0, 0);
}

void onCameraYChanged(int value, void*) {
    if (g_instance) g_instance->updateCameraParameter(value, 0, 1);
}

void onCameraZChanged(int value, void*) {
    if (g_instance) g_instance->updateCameraParameter(value, 0, 2);
}

void onCameraRollChanged(int value, void*) {
    if (g_instance) g_instance->updateCameraParameter(value, 1, 0);
}

void onCameraPitchChanged(int value, void*) {
    if (g_instance) g_instance->updateCameraParameter(value, 1, 1);
}

void onCameraYawChanged(int value, void*) {
    if (g_instance) g_instance->updateCameraParameter(value, 1, 2);
}

void onLidarXChanged(int value, void*) {
    if (g_instance) g_instance->updateLidarParameter(value, 0, 0);
}

void onLidarYChanged(int value, void*) {
    if (g_instance) g_instance->updateLidarParameter(value, 0, 1);
}

void onLidarZChanged(int value, void*) {
    if (g_instance) g_instance->updateLidarParameter(value, 0, 2);
}

void onLidarRollChanged(int value, void*) {
    if (g_instance) g_instance->updateLidarParameter(value, 1, 0);
}

void onLidarPitchChanged(int value, void*) {
    if (g_instance) g_instance->updateLidarParameter(value, 1, 1);
}

void onLidarYawChanged(int value, void*) {
    if (g_instance) g_instance->updateLidarParameter(value, 1, 2);
}

void onSaveButtonChanged(int value, void*) {
    if (g_instance && value == 1) {
        g_instance->saveCalibrationParameters();
        cv::setTrackbarPos("Save Parameters", g_instance->getCalibrationWindowName(), 0);
    }
}

LidarCameraCalibration::LidarCameraCalibration() : it_(nh_) {
    // Set global instance
    g_instance = this;
    
    // Load parameters
    nh_.param<int>("camera/width", image_width_, 640);
    nh_.param<int>("camera/height", image_height_, 480);
    nh_.param<double>("camera/fov", fov_, 90.0);
    nh_.param<double>("camera/x", cam_x_, 1.8);
    nh_.param<double>("camera/y", cam_y_, 0.0);
    nh_.param<double>("camera/z", cam_z_, 2.0);
    nh_.param<double>("camera/roll", cam_roll_, 0.0);
    nh_.param<double>("camera/pitch", cam_pitch_, 0.0);
    nh_.param<double>("camera/yaw", cam_yaw_, 0.0);
    
    nh_.param<double>("lidar/x", lidar_x_, 2.0);
    nh_.param<double>("lidar/y", lidar_y_, 0.0);
    nh_.param<double>("lidar/z", lidar_z_, 1.25);
    nh_.param<double>("lidar/roll", lidar_roll_, 0.0);
    nh_.param<double>("lidar/pitch", lidar_pitch_, 0.0);
    nh_.param<double>("lidar/yaw", lidar_yaw_, 0.0);
    
    // Initialize filtering parameters
    voxel_size_ = 0.05;  // 5cm voxel size (default)
    max_distance_ = 30.0;  // 30m maximum distance (default)
    point_skip_ = 1;  // Use all points (default)
    
    // Initialize point clouds
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Calculate transformation matrices
    calculateTransformMatrix();
    calculateCameraMatrix();
    
    // Setup time synchronization
    setupTimeSynchronization();
    
    // Setup projection image publisher
    image_pub_ = it_.advertise("/lidar_camera_calibration/projection", 1);
    
    // Setup GUI
    calibration_window_name_ = "LiDAR-Camera Calibration";
    filter_window_name_ = "Point Cloud Filter Settings";
    setupCalibrationGUI();
    setupFilterGUI();
    
    ROS_INFO("LiDAR-Camera calibration node initialized.");
}

LidarCameraCalibration::~LidarCameraCalibration() {
    g_instance = nullptr;
    ROS_INFO("LiDAR-Camera calibration node terminated.");
}

void LidarCameraCalibration::setVoxelSize(float size) {
    voxel_size_ = size;
    ROS_INFO("Voxel size set to %.2f m", voxel_size_);
}

void LidarCameraCalibration::setMaxDistance(float dist) {
    max_distance_ = dist;
    ROS_INFO("Maximum distance set to %.1f m", max_distance_);
}

void LidarCameraCalibration::setPointSkip(int skip) {
    point_skip_ = std::max(1, skip);
    ROS_INFO("Point skip set to %d", point_skip_);
}

void LidarCameraCalibration::setupTimeSynchronization() {
    // Setup synchronized subscribers
    lidar_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/ouster/points", 1));
    image_sub_.reset(new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh_, "/usb_cam/image_raw/compressed", 1));
    
    // Setup approximate time synchronization policy (10ms time window)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> SyncPolicy;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *lidar_sub_, *image_sub_));
    
    // Register callback
    sync_->registerCallback(boost::bind(&LidarCameraCalibration::syncCallback, this, _1, _2));
    
    ROS_INFO("Time synchronization for LiDAR and camera set up with 10ms window");
}

void LidarCameraCalibration::syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, 
                                          const sensor_msgs::CompressedImageConstPtr& img_msg) {
    // Process LiDAR data
    pcl::fromROSMsg(*cloud_msg, *cloud_);
    ROS_INFO_THROTTLE(1, "Synchronized LiDAR data received: %zu points, stamp: %.3f", 
                      cloud_->size(), cloud_msg->header.stamp.toSec());
    
    // Perform filtering
    filterPointCloud();
    
    // Process image data
    try {
        cv::Mat compressed_img = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);
        image_ = compressed_img.clone();
        ROS_INFO_THROTTLE(1, "Synchronized image data received: %dx%d, stamp: %.3f", 
                         image_.cols, image_.rows, img_msg->header.stamp.toSec());
        
        // If data is valid, perform projection
        if (!image_.empty() && !filtered_cloud_->empty()) {
            ROS_INFO_THROTTLE(1, "Time difference between messages: %.3f ms", 
                     std::abs((cloud_msg->header.stamp - img_msg->header.stamp).toSec() * 1000.0));
            projectPointsToImage();
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Image conversion error: %s", e.what());
    }
}

void LidarCameraCalibration::filterPointCloud() {
    filtered_cloud_->clear();
    
    // Record original cloud size
    size_t original_size = cloud_->size();
    
    // Filter points that are closer than the specified maximum distance
    pcl::PointCloud<pcl::PointXYZ>::Ptr distance_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud_->size(); i += point_skip_) {
        const auto& point = cloud_->points[i];
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance <= max_distance_) {
            distance_filtered->push_back(point);
        }
    }
    
    // Size of cloud after skip and distance filtering
    size_t distance_filtered_size = distance_filtered->size();
    
    // Apply VoxelGrid filter
    if (voxel_size_ > 0.01) {  // Only apply if voxel size is sufficiently large
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(distance_filtered);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*filtered_cloud_);
    } else {
        *filtered_cloud_ = *distance_filtered;
    }
    
    // Log filtering results
    ROS_INFO_THROTTLE(1, "Point cloud filtering: Original=%zu, Skip+Distance filtered=%zu, After voxelization=%zu", 
                    original_size, distance_filtered_size, filtered_cloud_->size());
}

void LidarCameraCalibration::projectPointsToImage() {
    // Skip if image is empty
    if (image_.empty() || filtered_cloud_->empty()) {
        return;
    }
    
    // Create projection image
    cv::Mat projection_image = image_.clone();
    
    // Project point cloud
    std::vector<cv::Point2i> image_points;
    std::vector<double> point_distances;  // Store point distances
    
    int total_points = 0;
    int valid_points = 0;
    int behind_camera = 0;
    int outside_image = 0;
    
    // Variables for distance range
    double min_distance = std::numeric_limits<double>::max();
    double max_distance = 0.0;
    
    for (const auto& point : filtered_cloud_->points) {
        total_points++;
        
        // Convert LiDAR point to homogeneous coordinates
        Eigen::Vector4d lidar_point(point.x, point.y, point.z, 1.0);
        
        // Transform LiDAR coordinates to camera coordinates
        Eigen::Vector3d camera_point = transformLidarToCamera(lidar_point);
        
        // Check if point is behind camera
        if (camera_point(2) <= 0) {
            behind_camera++;
            continue;
        }
        
        // Transform camera 3D coordinates to image 2D pixel coordinates
        cv::Point2i image_point = transformCameraToImage(camera_point);
        
        // Only add valid image points
        if (image_point.x != -1 && image_point.y != -1) {
            // Calculate distance: 3D Euclidean distance (in camera coordinate system)
            double distance = std::sqrt(
                camera_point(0) * camera_point(0) + 
                camera_point(1) * camera_point(1) + 
                camera_point(2) * camera_point(2)
            );
            
            // Update min/max distance
            min_distance = std::min(min_distance, distance);
            max_distance = std::max(max_distance, distance);
            
            image_points.push_back(image_point);
            point_distances.push_back(distance);
            valid_points++;
        } else {
            outside_image++;
        }
    }
    
    // Log point statistics
    ROS_INFO_THROTTLE(1, "Point statistics: Total=%d, Valid=%d, Behind Camera=%d, Outside Image=%d",
               total_points, valid_points, behind_camera, outside_image);
    ROS_INFO_THROTTLE(1, "Distance range: Min=%.2f, Max=%.2f meters", min_distance, max_distance);
    
    // Define color map for distance visualization (JET color map)
    // Implement finer color changes based on distance
    const int COLOR_MAP_SIZE = 1024;  // Very fine color differentiation
    std::vector<cv::Scalar> colorMap(COLOR_MAP_SIZE);
    
    for (int i = 0; i < COLOR_MAP_SIZE; i++) {
        double ratio = static_cast<double>(i) / (COLOR_MAP_SIZE - 1);
        
        // JET color map implementation (blue -> cyan -> green -> yellow -> red)
        double r, g, b;
        
        if (ratio < 0.125) {
            r = 0;
            g = 0;
            b = 0.5 + ratio * 4.0;
        } else if (ratio < 0.375) {
            r = 0;
            g = (ratio - 0.125) * 4.0;
            b = 1.0;
        } else if (ratio < 0.625) {
            r = (ratio - 0.375) * 4.0;
            g = 1.0;
            b = 1.0 - (ratio - 0.375) * 4.0;
        } else if (ratio < 0.875) {
            r = 1.0;
            g = 1.0 - (ratio - 0.625) * 4.0;
            b = 0;
        } else {
            r = 1.0 - (ratio - 0.875) * 4.0;
            g = 0;
            b = 0;
        }
        
        // Save color (convert to 0-255 range)
        colorMap[i] = cv::Scalar(b * 255, g * 255, r * 255);
    }
    
    // Draw points on image (color coding based on distance)
    for (size_t i = 0; i < image_points.size(); i++) {
        // Convert distance to color map index (log scale applied)
        double normalized_distance = (point_distances[i] - min_distance) / (max_distance - min_distance);
        
        // Apply log scaling to allocate more colors to closer distances
        normalized_distance = std::log(normalized_distance * 9.0 + 1.0) / std::log(10.0);
        
        int color_idx = std::min(COLOR_MAP_SIZE - 1, 
                                 std::max(0, static_cast<int>(normalized_distance * (COLOR_MAP_SIZE - 1))));
        
        cv::Scalar color = colorMap[color_idx];
        
        // Adjust point size based on distance (closer points are larger)
        int point_size = std::max(1, static_cast<int>(4 * (1.0 - normalized_distance) + 1));
        cv::circle(projection_image, image_points[i], point_size, color, -1);
    }
    
    // Display text information
    std::string info_text = "Valid points: " + std::to_string(valid_points) + " / " + std::to_string(total_points);
    cv::putText(projection_image, info_text, cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    
    // Display distance information
    std::string dist_text = "Distance range: " + std::to_string(int(min_distance*100)/100.0) + 
                            " - " + std::to_string(int(max_distance*100)/100.0) + " m";
    cv::putText(projection_image, dist_text, cv::Point(20, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    
    // Display filtering information
    std::string filter_text = "Filter: Voxel=" + std::to_string(int(voxel_size_*100)/100.0) + 
                             "m, MaxDist=" + std::to_string(int(max_distance_*10)/10.0) + 
                             "m, Skip=" + std::to_string(point_skip_);
    cv::putText(projection_image, filter_text, cv::Point(20, 120), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    
    // Display timestamp
    ros::Time current_time = ros::Time::now();
    std::string time_text = "Time: " + std::to_string(current_time.toSec());
    cv::putText(projection_image, time_text, cv::Point(20, 90), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    
    // Draw color legend
    int legend_width = 200;
    int legend_height = 20;
    int legend_x = projection_image.cols - legend_width - 20;
    int legend_y = 30;
    
    // Create mini image for color legend
    cv::Mat legend_image(legend_height, legend_width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < legend_width; i++) {
        double ratio = static_cast<double>(i) / legend_width;
        int color_idx = std::min(COLOR_MAP_SIZE - 1, 
                               std::max(0, static_cast<int>(ratio * (COLOR_MAP_SIZE - 1))));
        cv::line(legend_image, 
                cv::Point(i, 0), 
                cv::Point(i, legend_height), 
                colorMap[color_idx], 1);
    }
    
    // Insert legend image into main image
    cv::Mat legend_roi = projection_image(cv::Rect(legend_x, legend_y, legend_width, legend_height));
    legend_image.copyTo(legend_roi);
    
    // Add legend border
    cv::rectangle(projection_image, 
                 cv::Rect(legend_x, legend_y, legend_width, legend_height), 
                 cv::Scalar(255, 255, 255), 1);
    
    // Add legend text
    cv::putText(projection_image, "Distance", cv::Point(legend_x + 90, legend_y - 10), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(projection_image, std::to_string(int(min_distance*10)/10.0) + "m", 
                cv::Point(legend_x - 30, legend_y + legend_height/2 + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(projection_image, std::to_string(int(max_distance*10)/10.0) + "m", 
                cv::Point(legend_x + legend_width + 5, legend_y + legend_height/2 + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // Create OpenCV display (for debugging and visualization)
    cv::imshow("LiDAR to Camera Projection", projection_image);
    cv::waitKey(1);
    
    // Publish projection image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projection_image).toImageMsg();
    msg->header.stamp = current_time;
    image_pub_.publish(msg);
}

void LidarCameraCalibration::setupCalibrationGUI() {
    // Create calibration window
    cv::namedWindow(calibration_window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(calibration_window_name_, 600, 400);
    
    // Camera position trackbars (millimeter units, -500cm ~ +500cm)
    int cam_x_initial = static_cast<int>(cam_x_ * POS_SCALE_);
    int cam_y_initial = static_cast<int>(cam_y_ * POS_SCALE_);
    int cam_z_initial = static_cast<int>(cam_z_ * POS_SCALE_);
    
    // Camera rotation trackbars (degree units, -180° ~ +180°)
    int cam_roll_initial = static_cast<int>(cam_roll_ * ROT_SCALE_ * 180.0 / M_PI);
    int cam_pitch_initial = static_cast<int>(cam_pitch_ * ROT_SCALE_ * 180.0 / M_PI);
    int cam_yaw_initial = static_cast<int>(cam_yaw_ * ROT_SCALE_ * 180.0 / M_PI);
    
    // LiDAR position trackbars
    int lidar_x_initial = static_cast<int>(lidar_x_ * POS_SCALE_);
    int lidar_y_initial = static_cast<int>(lidar_y_ * POS_SCALE_);
    int lidar_z_initial = static_cast<int>(lidar_z_ * POS_SCALE_);
    
    // LiDAR rotation trackbars
    int lidar_roll_initial = static_cast<int>(lidar_roll_ * ROT_SCALE_ * 180.0 / M_PI);
    int lidar_pitch_initial = static_cast<int>(lidar_pitch_ * ROT_SCALE_ * 180.0 / M_PI);
    int lidar_yaw_initial = static_cast<int>(lidar_yaw_ * ROT_SCALE_ * 180.0 / M_PI);
    
    // Create trackbars for each parameter (with independent callback functions)
    cv::createTrackbar("Camera X (cm)", calibration_window_name_, nullptr, 500, onCameraXChanged);
    cv::setTrackbarMin("Camera X (cm)", calibration_window_name_, -500);
    cv::setTrackbarPos("Camera X (cm)", calibration_window_name_, cam_x_initial);
    
    cv::createTrackbar("Camera Y (cm)", calibration_window_name_, nullptr, 500, onCameraYChanged);
    cv::setTrackbarMin("Camera Y (cm)", calibration_window_name_, -500);
    cv::setTrackbarPos("Camera Y (cm)", calibration_window_name_, cam_y_initial);
    
    cv::createTrackbar("Camera Z (cm)", calibration_window_name_, nullptr, 500, onCameraZChanged);
    cv::setTrackbarMin("Camera Z (cm)", calibration_window_name_, -500);
    cv::setTrackbarPos("Camera Z (cm)", calibration_window_name_, cam_z_initial);
    
    cv::createTrackbar("Camera Roll (deg)", calibration_window_name_, nullptr, 180, onCameraRollChanged);
    cv::setTrackbarMin("Camera Roll (deg)", calibration_window_name_, -180);
    cv::setTrackbarPos("Camera Roll (deg)", calibration_window_name_, cam_roll_initial);
    
    cv::createTrackbar("Camera Pitch (deg)", calibration_window_name_, nullptr, 180, onCameraPitchChanged);
    cv::setTrackbarMin("Camera Pitch (deg)", calibration_window_name_, -180);
    cv::setTrackbarPos("Camera Pitch (deg)", calibration_window_name_, cam_pitch_initial);
    
    cv::createTrackbar("Camera Yaw (deg)", calibration_window_name_, nullptr, 180, onCameraYawChanged);
    cv::setTrackbarMin("Camera Yaw (deg)", calibration_window_name_, -180);
    cv::setTrackbarPos("Camera Yaw (deg)", calibration_window_name_, cam_yaw_initial);
    
    cv::createTrackbar("LiDAR X (cm)", calibration_window_name_, nullptr, 500, onLidarXChanged);
    cv::setTrackbarMin("LiDAR X (cm)", calibration_window_name_, -500);
    cv::setTrackbarPos("LiDAR X (cm)", calibration_window_name_, lidar_x_initial);
    
    cv::createTrackbar("LiDAR Y (cm)", calibration_window_name_, nullptr, 500, onLidarYChanged);
    cv::setTrackbarMin("LiDAR Y (cm)", calibration_window_name_, -500);
    cv::setTrackbarPos("LiDAR Y (cm)", calibration_window_name_, lidar_y_initial);
    
    cv::createTrackbar("LiDAR Z (cm)", calibration_window_name_, nullptr, 500, onLidarZChanged);
    cv::setTrackbarMin("LiDAR Z (cm)", calibration_window_name_, -500);
    cv::setTrackbarPos("LiDAR Z (cm)", calibration_window_name_, lidar_z_initial);
    
    cv::createTrackbar("LiDAR Roll (deg)", calibration_window_name_, nullptr, 180, onLidarRollChanged);
    cv::setTrackbarMin("LiDAR Roll (deg)", calibration_window_name_, -180);
    cv::setTrackbarPos("LiDAR Roll (deg)", calibration_window_name_, lidar_roll_initial);
    
    cv::createTrackbar("LiDAR Pitch (deg)", calibration_window_name_, nullptr, 180, onLidarPitchChanged);
    cv::setTrackbarMin("LiDAR Pitch (deg)", calibration_window_name_, -180);
    cv::setTrackbarPos("LiDAR Pitch (deg)", calibration_window_name_, lidar_pitch_initial);
    
    cv::createTrackbar("LiDAR Yaw (deg)", calibration_window_name_, nullptr, 180, onLidarYawChanged);
    cv::setTrackbarMin("LiDAR Yaw (deg)", calibration_window_name_, -180);
    cv::setTrackbarPos("LiDAR Yaw (deg)", calibration_window_name_, lidar_yaw_initial);
    
    cv::createTrackbar("Save Parameters", calibration_window_name_, nullptr, 1, onSaveButtonChanged);
    cv::setTrackbarPos("Save Parameters", calibration_window_name_, 0);
    
    ROS_INFO("Calibration GUI setup completed.");
}

void LidarCameraCalibration::setupFilterGUI() {
    // Create filter settings window
    cv::namedWindow(filter_window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(filter_window_name_, 500, 200);
    
    // Voxel size trackbar (1cm-20cm)
    int voxel_size_initial = static_cast<int>(voxel_size_ * 100);
    cv::createTrackbar("Voxel Size (cm)", filter_window_name_, nullptr, 20, onVoxelSizeChanged);
    cv::setTrackbarMin("Voxel Size (cm)", filter_window_name_, 1);
    cv::setTrackbarPos("Voxel Size (cm)", filter_window_name_, voxel_size_initial);
    
    // Maximum distance trackbar (1m-50m, 10cm units)
    int max_distance_initial = static_cast<int>(max_distance_ * 10);
    cv::createTrackbar("Max Distance (10cm)", filter_window_name_, nullptr, 500, onMaxDistanceChanged);
    cv::setTrackbarMin("Max Distance (10cm)", filter_window_name_, 10);
    cv::setTrackbarPos("Max Distance (10cm)", filter_window_name_, max_distance_initial);
    
    // Point skipping trackbar (1-10)
    cv::createTrackbar("Point Skip", filter_window_name_, nullptr, 10, onSkipPointsChanged);
    cv::setTrackbarMin("Point Skip", filter_window_name_, 1);
    cv::setTrackbarPos("Point Skip", filter_window_name_, point_skip_);
    
    ROS_INFO("Point cloud filter GUI setup completed.");
}

// Camera parameter update
void LidarCameraCalibration::updateCameraParameter(int value, int param_type, int axis) {
    if (param_type == 0) {  // Position
        // Convert cm to m
        double pos_value = value / POS_SCALE_;
        
        switch (axis) {
            case 0:  // X
                cam_x_ = pos_value;
                ROS_INFO("Camera X updated to %.2f m", cam_x_);
                break;
            case 1:  // Y
                cam_y_ = pos_value;
                ROS_INFO("Camera Y updated to %.2f m", cam_y_);
                break;
            case 2:  // Z
                cam_z_ = pos_value;
                ROS_INFO("Camera Z updated to %.2f m", cam_z_);
                break;
        }
    } else if (param_type == 1) {  // Rotation
        // Convert degrees to radians
        double rot_value = value * M_PI / (ROT_SCALE_ * 180.0);
        
        switch (axis) {
            case 0:  // Roll
                cam_roll_ = rot_value;
                ROS_INFO("Camera Roll updated to %.2f deg", value / ROT_SCALE_);
                break;
            case 1:  // Pitch
                cam_pitch_ = rot_value;
                ROS_INFO("Camera Pitch updated to %.2f deg", value / ROT_SCALE_);
                break;
            case 2:  // Yaw
                cam_yaw_ = rot_value;
                ROS_INFO("Camera Yaw updated to %.2f deg", value / ROT_SCALE_);
                break;
        }
    }
    
    // Recalculate transformation matrix
    calculateTransformMatrix();
}

// LiDAR parameter update
void LidarCameraCalibration::updateLidarParameter(int value, int param_type, int axis) {
    if (param_type == 0) {  // Position
        // Convert cm to m
        double pos_value = value / POS_SCALE_;
        
        switch (axis) {
            case 0:  // X
                lidar_x_ = pos_value;
                ROS_INFO("LiDAR X updated to %.2f m", lidar_x_);
                break;
            case 1:  // Y
                lidar_y_ = pos_value;
                ROS_INFO("LiDAR Y updated to %.2f m", lidar_y_);
                break;
            case 2:  // Z
                lidar_z_ = pos_value;
                ROS_INFO("LiDAR Z updated to %.2f m", lidar_z_);
                break;
        }
    } else if (param_type == 1) {  // Rotation
        // Convert degrees to radians
        double rot_value = value * M_PI / (ROT_SCALE_ * 180.0);
        
        switch (axis) {
            case 0:  // Roll
                lidar_roll_ = rot_value;
                ROS_INFO("LiDAR Roll updated to %.2f deg", value / ROT_SCALE_);
                break;
            case 1:  // Pitch
                lidar_pitch_ = rot_value;
                ROS_INFO("LiDAR Pitch updated to %.2f deg", value / ROT_SCALE_);
                break;
            case 2:  // Yaw
                lidar_yaw_ = rot_value;
                ROS_INFO("LiDAR Yaw updated to %.2f deg", value / ROT_SCALE_);
                break;
        }
    }
    
    // Recalculate transformation matrix
    calculateTransformMatrix();
}

// 캘리브레이션 파라미터 저장
void LidarCameraCalibration::saveCalibrationParameters() {
    // 현재 파라미터 값으로 YAML 파일 작성
    std::ofstream yaml_file;
    std::string file_path = ros::package::getPath("lidar_camera_calibration") + "/config/calibration_params.yaml";
    yaml_file.open(file_path);
    
    if (yaml_file.is_open()) {
        // 현재 시간 추가
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
        
        // Write file in exact format
        yaml_file << "# Calibration parameters - Saved at: " << buffer << std::endl;
        yaml_file << "# Camera parameter settings" << std::endl;
        yaml_file << "camera:" << std::endl;
        yaml_file << "  width: " << image_width_ << "          # Image width (pixels)" << std::endl;
        yaml_file << "  height: " << image_height_ << "         # Image height (pixels)" << std::endl;
        yaml_file << "  fov: " << (int)fov_ << "           # Field of View - degrees" << std::endl;
        
        // Format values with correct precision
        char x_str[64], y_str[64], z_str[64];
        char roll_str[64], pitch_str[64], yaw_str[64];
        
        // Format camera position values
        sprintf(x_str, "%.2f", cam_x_);
        sprintf(y_str, "%.2f", cam_y_);
        sprintf(z_str, "%.2f", cam_z_);
        sprintf(roll_str, "%.0f", cam_roll_);
        sprintf(pitch_str, "%.0f", cam_pitch_);
        sprintf(yaw_str, "%.0f", cam_yaw_);
        
        yaml_file << "  x: " << x_str << "              # Vehicle-frame X position (meters)" << std::endl;
        yaml_file << "  y: " << y_str << "              # Vehicle-frame Y position (meters)" << std::endl;
        yaml_file << "  z: " << z_str << "              # Vehicle-frame Z position (meters)" << std::endl;
        yaml_file << "  roll: " << roll_str << "           # Roll angle (radians) - X-axis rotation" << std::endl;
        yaml_file << "  pitch: " << pitch_str << "          # Pitch angle (radians) - Y-axis rotation" << std::endl;
        yaml_file << "  yaw: " << yaw_str << "            # Yaw angle (radians) - Z-axis rotation" << std::endl;
        
        yaml_file << std::endl;
        yaml_file << "# LiDAR parameter settings" << std::endl;
        yaml_file << "lidar:" << std::endl;
        
        // Format LiDAR position values
        sprintf(x_str, "%.2f", lidar_x_);
        sprintf(y_str, "%.2f", lidar_y_);
        sprintf(z_str, "%.2f", lidar_z_);
        sprintf(roll_str, "%.0f", lidar_roll_);
        sprintf(pitch_str, "%.0f", lidar_pitch_);
        sprintf(yaw_str, "%.0f", lidar_yaw_);
        
        yaml_file << "  x: " << x_str << "              # Vehicle-frame X position (meters)" << std::endl;
        yaml_file << "  y: " << y_str << "              # Vehicle-frame Y position (meters)" << std::endl;
        yaml_file << "  z: " << z_str << "             # Vehicle-frame Z position (meters)" << std::endl;
        yaml_file << "  roll: " << roll_str << "           # Roll angle (radians) - X-axis rotation" << std::endl;
        yaml_file << "  pitch: " << pitch_str << "          # Pitch angle (radians) - Y-axis rotation" << std::endl;
        yaml_file << "  yaw: " << yaw_str << "           # Yaw angle (radians) - Z-axis rotation" << std::endl;
        
        yaml_file.close();
        ROS_INFO("Calibration parameters saved to %s", file_path.c_str());
    } else {
        ROS_ERROR("Failed to open file for saving calibration parameters: %s", file_path.c_str());
    }
}

Eigen::Matrix3d LidarCameraCalibration::getRotationMatrix(double roll, double pitch, double yaw) {
    // Calculate cosine and sine values for each angle
    double cos_roll = cos(roll);
    double cos_pitch = cos(pitch);
    double cos_yaw = cos(yaw);
    double sin_roll = sin(roll);
    double sin_pitch = sin(pitch);
    double sin_yaw = sin(yaw);
    
    // Create rotation matrix for each axis
    Eigen::Matrix3d rot_roll;
    rot_roll << 1, 0, 0,
                0, cos_roll, -sin_roll,
                0, sin_roll, cos_roll;
    
    Eigen::Matrix3d rot_pitch;
    rot_pitch << cos_pitch, 0, sin_pitch,
                 0, 1, 0,
                 -sin_pitch, 0, cos_pitch;
    
    Eigen::Matrix3d rot_yaw;
    rot_yaw << cos_yaw, -sin_yaw, 0,
               sin_yaw, cos_yaw, 0,
               0, 0, 1;
    
    // Total rotation matrix = Yaw * Pitch * Roll (matrix multiplication order is important)
    return rot_yaw * rot_pitch * rot_roll;
}

void LidarCameraCalibration::calculateTransformMatrix() {
    // Apply additional rotation to camera pose (for camera coordinate system transformation)
    double adjusted_cam_roll = cam_roll_ - M_PI/2;  // -90 degrees
    double adjusted_cam_pitch = cam_pitch_;
    double adjusted_cam_yaw = cam_yaw_ - M_PI/2;    // -90 degrees
    
    // Calculate rotation matrices
    Eigen::Matrix3d cam_rot = getRotationMatrix(adjusted_cam_roll, adjusted_cam_pitch, adjusted_cam_yaw);
    Eigen::Matrix3d lidar_rot = getRotationMatrix(lidar_roll_, lidar_pitch_, lidar_yaw_);
    
    // Create transformation matrices
    Eigen::Matrix4d Tr_cam_to_vehicle = Eigen::Matrix4d::Identity();
    Tr_cam_to_vehicle.block<3, 3>(0, 0) = cam_rot;
    Tr_cam_to_vehicle(0, 3) = cam_x_;
    Tr_cam_to_vehicle(1, 3) = cam_y_;
    Tr_cam_to_vehicle(2, 3) = cam_z_;
    
    Eigen::Matrix4d Tr_lidar_to_vehicle = Eigen::Matrix4d::Identity();
    Tr_lidar_to_vehicle.block<3, 3>(0, 0) = lidar_rot;
    Tr_lidar_to_vehicle(0, 3) = lidar_x_;
    Tr_lidar_to_vehicle(1, 3) = lidar_y_;
    Tr_lidar_to_vehicle(2, 3) = lidar_z_;
    
    // Calculate LiDAR-to-camera transformation matrix: (camera-to-vehicle)^-1 * (lidar-to-vehicle)
    transform_lidar_to_cam_ = Tr_cam_to_vehicle.inverse() * Tr_lidar_to_vehicle;
    
    ROS_INFO_STREAM("Transform matrix from LiDAR to camera: \n" << transform_lidar_to_cam_);
}

void LidarCameraCalibration::calculateCameraMatrix() {
    // Calculate focal length: width / (2 * tan(FOV/2))
    double focal_length = image_width_ / (2 * tan(fov_ * M_PI / 360.0));  // Convert FOV to radians
    
    // Principal point coordinates - typically image center
    double cx = image_width_ / 2.0;
    double cy = image_height_ / 2.0;
    
    // Create camera intrinsic parameter matrix
    camera_matrix_ = Eigen::Matrix3d::Identity();
    camera_matrix_(0, 0) = focal_length;  // fx
    camera_matrix_(1, 1) = focal_length;  // fy
    camera_matrix_(0, 2) = cx;            // cx
    camera_matrix_(1, 2) = cy;            // cy
    
    ROS_INFO_STREAM("Camera intrinsic matrix: \n" << camera_matrix_);
}

Eigen::Vector3d LidarCameraCalibration::transformLidarToCamera(const Eigen::Vector4d& pc_lidar) {
    // Transform LiDAR coordinates to camera coordinates using the transformation matrix
    Eigen::Vector4d pc_cam_homogeneous = transform_lidar_to_cam_ * pc_lidar;
    
    // Convert homogeneous coordinates to 3D coordinates
    return pc_cam_homogeneous.head<3>();
}

cv::Point2i LidarCameraCalibration::transformCameraToImage(const Eigen::Vector3d& pc_camera) {
    // Ignore points behind the camera
    if (pc_camera(2) <= 0) {
        return cv::Point2i(-1, -1);  // Invalid point
    }
    
    // Use camera intrinsic parameter matrix for 3D->2D transformation
    Eigen::Vector3d pc_image = camera_matrix_ * pc_camera;
    
    // Homogenization (divide by z)
    int u = static_cast<int>(pc_image(0) / pc_image(2));
    int v = static_cast<int>(pc_image(1) / pc_image(2));
    
    // Ignore points outside the image area
    if (u < 0 || u >= image_width_ || v < 0 || v >= image_height_) {
        return cv::Point2i(-1, -1);  // Invalid point
    }
    
    return cv::Point2i(u, v);
}

} // namespace lidar_camera_calibration