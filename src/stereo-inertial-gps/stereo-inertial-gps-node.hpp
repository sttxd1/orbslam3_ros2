#ifndef __STEREO_INERTIAL_GPS_NODE_HPP__
#define __STEREO_INERTIAL_GPS_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// #include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "WGS84toCartesian.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class StereoInertialGPSNode : public rclcpp::Node
{
public:
    StereoInertialGPSNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual);
    ~StereoInertialGPSNode();

private:
    using ImuMsg = sensor_msgs::msg::Imu;
    using ImageMsg = sensor_msgs::msg::Image;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using TfPoseStampedMsg = geometry_msgs::msg::TransformStamped;
    // using MarkerMsg = visualization_msgs::msg::Marker;
    using OdomMsg = nav_msgs::msg::Odometry;
    using PathMsg = nav_msgs::msg::Path;
    using GPSMsg = sensor_msgs::msg::NavSatFix;

    // const GPSMsg::SharedPtr GPSmsg_new;


    void initial();
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    void GrabGPS(const GPSMsg::SharedPtr GPSmsg);
    int num_GPS_msg = 0;
    bool GPS_inital;
    std::array<double, 2> ENU_origin;
    double ENU_height;

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;
    rclcpp::Subscription<GPSMsg>::SharedPtr gps_publisher;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;
    ORB_SLAM3::System::eSensor sensor_type;
    
    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    //GPS
    queue<GPSMsg::SharedPtr> gpsBuf_;
    std::mutex bufMutexgps_;

    // Image
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

    rclcpp::Publisher<OdomMsg>::SharedPtr odom_publisher;
    rclcpp::Publisher<PathMsg>::SharedPtr path_publisher;
    rclcpp::Publisher<PathMsg>::SharedPtr gps_path_publisher;
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr camera_pose_publisher;
    // rclcpp::Publisher<TfPoseStampedMsg>::SharedPtr body_pose_publisher;
    // rclcpp::Publisher<PointCloud2Msg>::SharedPtr key_point_cloud_publisher;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr tracked_point_cloud_publisher;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr all_point_cloud_publisher;
    // rclcpp::Publisher<MarkerMsg>::SharedPtr key_markers_publisher;

    // Tf
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> world2orbmap_static_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> orb2body_static_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> body2gps_static_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> world2enu_static_broadcaster; 
    std::shared_ptr<tf2_ros::TransformBroadcaster> orbmap2orb_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> gps2enu_broadcaster;
    PathMsg orb_path_msg;
    PathMsg GPS_path_msg;

    void TF_Static_Broadcaster(std::shared_ptr<tf2_ros::StaticTransformBroadcaster> name,
                                std::string frame_id,
                                std::string child_frame_id,
                                Eigen::Vector3d position,
                                Eigen::Quaterniond quaternion);
    void Publish_Camera_Pose(Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time);
    void Pulish_All_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time);
    void Pulish_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time);
    sensor_msgs::msg::PointCloud2 Feature_Points_to_Point_cloud(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time);

};

#endif
