#include <lidar_camera_calibration/lidar_camera_calibration.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace lidar_camera_calibration {

LidarCameraCalibration::LidarCameraCalibration() : it_(nh_) {
    // 파라미터 로딩
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
    
    // 포인트 클라우드 초기화
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    
    // 변환 행렬 계산
    calculateTransformMatrix();
    calculateCameraMatrix();
    
    // 시간 동기화를 위한 설정
    setupTimeSynchronization();
    
    // 투영 이미지 발행자 설정
    image_pub_ = it_.advertise("/lidar_camera_calibration/projection", 1);
    
    ROS_INFO("LiDAR-Camera calibration node initialized.");
}

LidarCameraCalibration::~LidarCameraCalibration() {
    ROS_INFO("LiDAR-Camera calibration node terminated.");
}

void LidarCameraCalibration::setupTimeSynchronization() {
    // 동기화된 구독자 설정
    lidar_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/velodyne_points", 1));
    image_sub_.reset(new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh_, "/image_jpeg/compressed", 1));
    
    // 근사 시간 동기화 정책 설정 (10ms 타임 윈도우)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> SyncPolicy;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *lidar_sub_, *image_sub_));
    
    // 콜백 등록
    sync_->registerCallback(boost::bind(&LidarCameraCalibration::syncCallback, this, _1, _2));
    
    ROS_INFO("Time synchronization for LiDAR and camera set up with 10ms window");
}

void LidarCameraCalibration::syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, 
                                          const sensor_msgs::CompressedImageConstPtr& img_msg) {
    // LiDAR 데이터 처리
    pcl::fromROSMsg(*cloud_msg, *cloud_);
    ROS_INFO_THROTTLE(1, "Synchronized LiDAR data received: %zu points, stamp: %.3f", 
                      cloud_->size(), cloud_msg->header.stamp.toSec());
    
    // 이미지 데이터 처리
    try {
        cv::Mat compressed_img = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);
        image_ = compressed_img.clone();
        ROS_INFO_THROTTLE(1, "Synchronized image data received: %dx%d, stamp: %.3f", 
                         image_.cols, image_.rows, img_msg->header.stamp.toSec());
        
        // 데이터가 유효하면 투영 수행
        if (!image_.empty() && !cloud_->empty()) {
            ROS_INFO_THROTTLE(1, "Time difference between messages: %.3f ms", 
                     std::abs((cloud_msg->header.stamp - img_msg->header.stamp).toSec() * 1000.0));
            projectPointsToImage();
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Image conversion error: %s", e.what());
    }
}

void LidarCameraCalibration::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 이 함수는 시간 동기화 사용 시 호출되지 않음 (하위 호환성을 위해 유지)
    pcl::fromROSMsg(*cloud_msg, *cloud_);
    
    ROS_INFO_THROTTLE(1, "LiDAR data received: %zu points (not synchronized)", cloud_->size());
    
    // 포인트 클라우드 및 이미지가 존재하면 투영 수행
    if (!image_.empty()) {
        projectPointsToImage();
    }
}

void LidarCameraCalibration::imageCallback(const sensor_msgs::CompressedImageConstPtr& img_msg) {
    // 이 함수는 시간 동기화 사용 시 호출되지 않음 (하위 호환성을 위해 유지)
    try {
        // 압축된 이미지 디코딩
        cv::Mat compressed_img = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);
        image_ = compressed_img.clone();
        
        ROS_INFO_THROTTLE(1, "Image data received: %dx%d (not synchronized)", image_.cols, image_.rows);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Image conversion error: %s", e.what());
    }
}

Eigen::Matrix3d LidarCameraCalibration::getRotationMatrix(double roll, double pitch, double yaw) {
    // 각 각도의 코사인, 사인 값 계산
    double cos_roll = cos(roll);
    double cos_pitch = cos(pitch);
    double cos_yaw = cos(yaw);
    double sin_roll = sin(roll);
    double sin_pitch = sin(pitch);
    double sin_yaw = sin(yaw);
    
    // 각 축별 회전 행렬 생성
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
    
    // 전체 회전 행렬 = Yaw * Pitch * Roll (행렬 곱셈 순서 중요)
    return rot_yaw * rot_pitch * rot_roll;
}

void LidarCameraCalibration::calculateTransformMatrix() {
    // 카메라 자세에 추가 회전 적용 (카메라 좌표계 변환을 위한 보정)
    double adjusted_cam_roll = cam_roll_ - M_PI/2;  // -90도
    double adjusted_cam_pitch = cam_pitch_;
    double adjusted_cam_yaw = cam_yaw_ - M_PI/2;    // -90도
    
    // 회전 행렬 계산
    Eigen::Matrix3d cam_rot = getRotationMatrix(adjusted_cam_roll, adjusted_cam_pitch, adjusted_cam_yaw);
    Eigen::Matrix3d lidar_rot = getRotationMatrix(lidar_roll_, lidar_pitch_, lidar_yaw_);
    
    // 변환 행렬 생성
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
    
    // 라이다-카메라 간 변환 행렬 계산: (카메라-차량)^-1 * (라이다-차량)
    transform_lidar_to_cam_ = Tr_cam_to_vehicle.inverse() * Tr_lidar_to_vehicle;
    
    ROS_INFO_STREAM("Transform matrix from LiDAR to camera: \n" << transform_lidar_to_cam_);
}

void LidarCameraCalibration::calculateCameraMatrix() {
    // 초점 거리(focal length) 계산: width / (2 * tan(FOV/2))
    double focal_length = image_width_ / (2 * tan(fov_ * M_PI / 360.0));  // fov를 라디안으로 변환
    
    // 주점(principal point) 좌표 - 일반적으로 이미지 중심
    double cx = image_width_ / 2.0;
    double cy = image_height_ / 2.0;
    
    // 카메라 내부 파라미터 행렬 생성
    camera_matrix_ = Eigen::Matrix3d::Identity();
    camera_matrix_(0, 0) = focal_length;  // fx
    camera_matrix_(1, 1) = focal_length;  // fy
    camera_matrix_(0, 2) = cx;            // cx
    camera_matrix_(1, 2) = cy;            // cy
    
    ROS_INFO_STREAM("Camera intrinsic matrix: \n" << camera_matrix_);
}

Eigen::Vector3d LidarCameraCalibration::transformLidarToCamera(const Eigen::Vector4d& pc_lidar) {
    // 변환 행렬을 이용해 라이다 좌표를 카메라 좌표로 변환
    Eigen::Vector4d pc_cam_homogeneous = transform_lidar_to_cam_ * pc_lidar;
    
    // 동차 좌표를 3D 좌표로 변환
    return pc_cam_homogeneous.head<3>();
}

cv::Point2i LidarCameraCalibration::transformCameraToImage(const Eigen::Vector3d& pc_camera) {
    // 카메라 뒤에 있는 포인트는 무시
    if (pc_camera(2) <= 0) {
        return cv::Point2i(-1, -1);  // 무효한 포인트
    }
    
    // 카메라 내부 파라미터 행렬을 이용해 3D->2D 변환
    Eigen::Vector3d pc_image = camera_matrix_ * pc_camera;
    
    // 동차화 (z로 나누기)
    int u = static_cast<int>(pc_image(0) / pc_image(2));
    int v = static_cast<int>(pc_image(1) / pc_image(2));
    
    // 이미지 영역을 벗어난 포인트 무시
    if (u < 0 || u >= image_width_ || v < 0 || v >= image_height_) {
        return cv::Point2i(-1, -1);  // 무효한 포인트
    }
    
    return cv::Point2i(u, v);
}

void LidarCameraCalibration::projectPointsToImage() {
    // 이미지가 없으면 처리하지 않음
    if (image_.empty() || cloud_->empty()) {
        return;
    }
    
    // 투영 이미지 생성
    cv::Mat projection_image = image_.clone();
    
    // 포인트 클라우드 필터링 및 투영
    std::vector<cv::Point2i> image_points;
    
    int total_points = 0;
    int valid_points = 0;
    int filtered_points = 0;
    int behind_camera = 0;
    int outside_image = 0;
    
    for (const auto& point : cloud_->points) {
        total_points++;
        
        // 필터링: 관심 영역 내의 포인트만 선택 (조건 완화)
        // 원래 조건: point.x < 0 || point.x > 10 || point.z < -1.2
        // 모든 포인트를 시도해봅니다 (필터링 제거)
        //if (point.x < -5 || point.x > 30 || point.z < -3) {
        //    filtered_points++;
        //    continue;  // 필터링 조건
        //}
        
        // 라이다 포인트를 동차 좌표로 변환
        Eigen::Vector4d lidar_point(point.x, point.y, point.z, 1.0);
        
        // 라이다 좌표를 카메라 좌표로 변환
        Eigen::Vector3d camera_point = transformLidarToCamera(lidar_point);
        
        // 카메라 뒤에 있는 포인트 확인
        if (camera_point(2) <= 0) {
            behind_camera++;
            continue;
        }
        
        // 카메라 3D 좌표를 이미지 2D 픽셀 좌표로 변환
        cv::Point2i image_point = transformCameraToImage(camera_point);
        
        // 유효한 이미지 포인트만 추가
        if (image_point.x != -1 && image_point.y != -1) {
            image_points.push_back(image_point);
            valid_points++;
        } else {
            outside_image++;
        }
    }
    
    // 포인트 개수 로깅
    ROS_INFO_THROTTLE(1, "Point statistics: Total=%d, Valid=%d, Filtered=%d, Behind Camera=%d, Outside Image=%d",
               total_points, valid_points, filtered_points, behind_camera, outside_image);
    
    // 이미지에 포인트 그리기
    for (const auto& point : image_points) {
        cv::circle(projection_image, point, 2, cv::Scalar(0, 255, 0), -1);
    }
    
    // 포인트 수 텍스트 출력
    std::string info_text = "Valid points: " + std::to_string(valid_points) + " / " + std::to_string(total_points);
    cv::putText(projection_image, info_text, cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    
    // 타임스탬프 텍스트 출력
    ros::Time current_time = ros::Time::now();
    std::string time_text = "Time: " + std::to_string(current_time.toSec());
    cv::putText(projection_image, time_text, cv::Point(20, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    
    // OpenCV 디스플레이 생성 (디버깅 및 시각화용)
    cv::imshow("LiDAR to Camera Projection", projection_image);
    cv::waitKey(1);
    
    // 투영 이미지 발행
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projection_image).toImageMsg();
    msg->header.stamp = current_time;
    image_pub_.publish(msg);
}

} // namespace lidar_camera_calibration