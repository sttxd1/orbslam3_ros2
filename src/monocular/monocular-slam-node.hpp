#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Vector3.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();


private:
    using ImageMsg = sensor_msgs::msg::Image;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;

    // void TF_Static_Broadcaster(std::string frame_id, std::string child_frame_id);
    // void TF_Broadcaster(std::string frame_id, std::string child_frame_id, Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time);
    void GrabImage(const ImageMsg::SharedPtr msg);
    void Publish_Camera_Pose(Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time);
    void Pulish_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time);
    sensor_msgs::msg::PointCloud2 Feature_Points_to_Point_cloud(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time);
    
    ORB_SLAM3::System* m_SLAM;
    ORB_SLAM3::System::eSensor sensor_type;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr camera_pose_publisher;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr point_cloud_publisher;
    // rclcpp::Publisher<PathMsg>::SharedPtr camera_path_publisher;
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> cam2base_static_broadcaster;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> cam2world_broadcaster;
};

#endif
