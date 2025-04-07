#ifndef LIDAR_CAMERA_CALIBRATION_H
#define LIDAR_CAMERA_CALIBRATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace lidar_camera_calibration {

/**
 * @brief 라이다-카메라 캘리브레이션 및 포인트 클라우드 투영 클래스
 */
class LidarCameraCalibration {
public:
    /**
     * @brief 생성자
     */
    LidarCameraCalibration();

    /**
     * @brief 소멸자
     */
    ~LidarCameraCalibration();

    /**
     * @brief GUI에서 카메라 파라미터 업데이트 시 호출되는 함수
     * @param value 새 파라미터 값
     * @param param_type 파라미터 유형 (위치/회전)
     * @param axis 회전 축 (0=x, 1=y, 2=z)
     */
    void updateCameraParameter(int value, int param_type, int axis);

    /**
     * @brief GUI에서 라이다 파라미터 업데이트 시 호출되는 함수
     * @param value 새 파라미터 값
     * @param param_type 파라미터 유형 (위치/회전)
     * @param axis 회전 축 (0=x, 1=y, 2=z)
     */
    void updateLidarParameter(int value, int param_type, int axis);

    /**
     * @brief 캘리브레이션 파라미터를 저장하는 함수
     */
    void saveCalibrationParameters();

    /**
     * @brief 복셀 크기 설정 함수
     * @param size 복셀 크기 (미터)
     */
    void setVoxelSize(float size);

    /**
     * @brief 최대 거리 설정 함수
     * @param dist 최대 거리 (미터)
     */
    void setMaxDistance(float dist);

    /**
     * @brief 포인트 건너뛰기 수 설정 함수
     * @param skip 건너뛸 포인트 수
     */
    void setPointSkip(int skip);

    // GUI 창 이름 접근자
    std::string getCalibrationWindowName() const { return calibration_window_name_; }

private:
    /**
     * @brief 시간 동기화 설정 함수
     */
    void setupTimeSynchronization();

    /**
     * @brief 동기화된 메시지 콜백 함수
     * @param cloud_msg 라이다 포인트 클라우드 메시지
     * @param img_msg 압축된 이미지 메시지
     */
    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, 
                      const sensor_msgs::CompressedImageConstPtr& img_msg);

    /**
     * @brief 라이다 포인트 클라우드 콜백 함수 (비동기식)
     * @param cloud_msg 라이다 포인트 클라우드 메시지
     */
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    /**
     * @brief 압축된 이미지 콜백 함수 (비동기식)
     * @param img_msg 압축된 이미지 메시지
     */
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& img_msg);

    /**
     * @brief Roll-Pitch-Yaw 각도를 회전 행렬로 변환하는 함수
     * @param roll Roll 각도 (라디안)
     * @param pitch Pitch 각도 (라디안)
     * @param yaw Yaw 각도 (라디안)
     * @return 3x3 회전 행렬
     */
    Eigen::Matrix3d getRotationMatrix(double roll, double pitch, double yaw);

    /**
     * @brief 라이다에서 카메라로의 변환 행렬을 계산하는 함수
     */
    void calculateTransformMatrix();

    /**
     * @brief 카메라 내부 파라미터 행렬을 계산하는 함수
     */
    void calculateCameraMatrix();

    /**
     * @brief 라이다 좌표계의 포인트를 카메라 좌표계로 변환하는 함수
     * @param pc_lidar 라이다 좌표계의 포인트 벡터
     * @return 카메라 좌표계로 변환된 포인트 벡터
     */
    Eigen::Vector3d transformLidarToCamera(const Eigen::Vector4d& pc_lidar);

    /**
     * @brief 카메라 3D 좌표를 이미지 2D 픽셀 좌표로 변환하는 함수
     * @param pc_camera 카메라 좌표계의 포인트
     * @return 이미지 픽셀 좌표 (x, y)
     */
    cv::Point2i transformCameraToImage(const Eigen::Vector3d& pc_camera);

    /**
     * @brief 이미지에 포인트 클라우드를 투영하는 함수
     */
    void projectPointsToImage();

    /**
     * @brief 캘리브레이션 파라미터 조정용 GUI를 설정하는 함수
     */
    void setupCalibrationGUI();

    /**
     * @brief 포인트 클라우드 필터링 옵션 GUI를 설정하는 함수
     */
    void setupFilterGUI();

    /**
     * @brief 포인트 클라우드 필터링 함수
     */
    void filterPointCloud();

    // ROS 관련 변수
    ros::NodeHandle nh_;                      ///< ROS 노드 핸들
    image_transport::ImageTransport it_;      ///< 이미지 전송 핸들러
    image_transport::Publisher image_pub_;    ///< 투영 이미지 발행자

    // 시간 동기화 관련 변수
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> LidarSubscriber;
    typedef message_filters::Subscriber<sensor_msgs::CompressedImage> ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    boost::shared_ptr<LidarSubscriber> lidar_sub_;     ///< 라이다 데이터 동기화 구독자
    boost::shared_ptr<ImageSubscriber> image_sub_;     ///< 이미지 데이터 동기화 구독자
    boost::shared_ptr<Synchronizer> sync_;             ///< 메시지 동기화 객체

    // 포인트 클라우드 및 이미지 데이터
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; ///< 원본 라이다 포인트 클라우드
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_; ///< 필터링된 라이다 포인트 클라우드
    cv::Mat image_;                             ///< 카메라 이미지

    // 카메라 파라미터
    int image_width_;                ///< 이미지 너비
    int image_height_;               ///< 이미지 높이
    double fov_;                     ///< 시야각 (Field of View)
    double cam_x_, cam_y_, cam_z_;   ///< 카메라 위치
    double cam_roll_, cam_pitch_, cam_yaw_; ///< 카메라 자세 (RPY)

    // 라이다 파라미터
    double lidar_x_, lidar_y_, lidar_z_;    ///< 라이다 위치
    double lidar_roll_, lidar_pitch_, lidar_yaw_; ///< 라이다 자세 (RPY)
    
    // 필터링 파라미터
    float voxel_size_;              ///< 복셀 크기 (미터)
    float max_distance_;            ///< 최대 거리 (미터)
    int point_skip_;                ///< 건너뛸 포인트 수
    
    // 변환 행렬
    Eigen::Matrix4d transform_lidar_to_cam_; ///< 라이다에서 카메라로의 변환 행렬
    Eigen::Matrix3d camera_matrix_;         ///< 카메라 내부 파라미터 행렬

    // GUI 관련 변수
    std::string calibration_window_name_;  ///< 캘리브레이션 GUI 창 이름
    std::string filter_window_name_;       ///< 필터링 GUI 창 이름
    
    // 파라미터 배율 (트랙바 값 변환용)
    static constexpr double POS_SCALE_ = 100.0;  ///< 위치 값 스케일링 (cm 단위 표시)
    static constexpr double ROT_SCALE_ = 100.0;  ///< 회전 값 스케일링 (라디안->각도)
};

} // namespace lidar_camera_calibration

#endif // LIDAR_CAMERA_CALIBRATION_H 