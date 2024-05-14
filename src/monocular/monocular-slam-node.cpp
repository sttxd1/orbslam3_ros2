#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;
std::string world_map_id, camera_map_id, scooty_frame_id, cam_frame_id, imu_frame_id;
Sophus::SE3f Tc0w = Sophus::SE3f();

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    // std::string node_name = rclcpp::Node::get_node_names();
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/stereo/left/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    camera_pose_publisher = this->create_publisher<PoseStampedMsg>("/camera/frame/pose", 10);
    point_cloud_publisher = this->create_publisher<PointCloud2Msg>("/camera/frame/pointcloud", 10);
    // cam2world_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // cam2base_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // TF_Static_Broadcaster("camera","base");
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

// void MonocularSlamNode::TF_Static_Broadcaster(std::string frame_id, std::string child_frame_id){
//     geometry_msgs::msg::TransformStamped static_transform;
//     static_transform.header.stamp = this->get_clock()->now();
//     static_transform.header.frame_id = frame_id;  // Parent frame
//     static_transform.child_frame_id = child_frame_id; // Child frame
//     static_transform.transform.translation.x = 0.0;  // meters
//     static_transform.transform.translation.y = 0.0;
//     static_transform.transform.translation.z = 0.0;
//     tf2::Quaternion q;
//     q.setRPY(-1.57, 0, -1.57);  // Rotation in radians
//     static_transform.transform.rotation.x = q.x();
//     static_transform.transform.rotation.y = q.y();
//     static_transform.transform.rotation.z = q.z();
//     static_transform.transform.rotation.w = q.w();
    
//     cam2base_static_broadcaster->sendTransform(static_transform);
// }

// void MonocularSlamNode::TF_Broadcaster(std::string frame_id, std::string child_frame_id, Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time){
//     geometry_msgs::msg::TransformStamped dy_transform;
//     dy_transform.header.stamp = msg_time;
//     dy_transform.header.frame_id = frame_id;  // Parent frame
//     dy_transform.child_frame_id = child_frame_id; // Child frame

//     dy_transform.transform.translation.x = Twc_SE3f.translation().x();
//     dy_transform.transform.translation.y = Twc_SE3f.translation().y();
//     dy_transform.transform.translation.z = Twc_SE3f.translation().z();

//     dy_transform.transform.rotation.w = Twc_SE3f.unit_quaternion().coeffs().w();
//     dy_transform.transform.rotation.x = Twc_SE3f.unit_quaternion().coeffs().x();
//     dy_transform.transform.rotation.y = Twc_SE3f.unit_quaternion().coeffs().y();
//     dy_transform.transform.rotation.z = Twc_SE3f.unit_quaternion().coeffs().z();

//     cam2world_broadcaster->sendTransform(dy_transform);
// }

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // std::cout<<"one frame has been sent"<<std::endl;
    Sophus::SE3f Tcc0 = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();

    rclcpp::Time msg_time = msg->header.stamp;
    this->Publish_Camera_Pose(Twc, msg_time);
    this->Pulish_Feature_Points(m_SLAM->GetTrackedMapPoints(), msg_time);
    RCLCPP_INFO(this->get_logger(), "This is an informational message.");
    // RCLCPP_INFO(this->get_logger(), m_SLAM->GetTrackedMapPoints().size());
    // This snippet assumes it's placed inside a method of a class that inherits from rclcpp::Node
    RCLCPP_INFO(this->get_logger(), "Tracked Map Points: %zu", m_SLAM->GetTrackedMapPoints().size());

    // TF_Broadcaster("world", "camera", Twc, msg_time)
}

void MonocularSlamNode::Publish_Camera_Pose(Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time){
    PoseStampedMsg pose_msg;

    pose_msg.header.frame_id = "camera_frame";
    pose_msg.header.stamp = msg_time;

    pose_msg.pose.position.x = Twc_SE3f.translation().x();
    pose_msg.pose.position.y = Twc_SE3f.translation().y();
    pose_msg.pose.position.z = Twc_SE3f.translation().z();

    pose_msg.pose.orientation.w = Twc_SE3f.unit_quaternion().coeffs().w();
    pose_msg.pose.orientation.x = Twc_SE3f.unit_quaternion().coeffs().x();
    pose_msg.pose.orientation.y = Twc_SE3f.unit_quaternion().coeffs().y();
    pose_msg.pose.orientation.z = Twc_SE3f.unit_quaternion().coeffs().z();

    camera_pose_publisher->publish(pose_msg);
}

void MonocularSlamNode::Pulish_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
    PointCloud2Msg pointcloud = Feature_Points_to_Point_cloud(feature_points, msg_time);
    point_cloud_publisher->publish(pointcloud);
}

sensor_msgs::msg::PointCloud2 MonocularSlamNode::Feature_Points_to_Point_cloud(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
    const int num_channels = 3;
    if (feature_points.size() == 0){
        RCLCPP_INFO(this->get_logger(), "Empty feature points.");
    }

    sensor_msgs::msg::PointCloud2 point_cloud2;
    point_cloud2.header.frame_id = "camera_map_frame";
    point_cloud2.header.stamp = msg_time;
    point_cloud2.height = 1;
    point_cloud2.width = feature_points.size();
    point_cloud2.is_bigendian = false;
    point_cloud2.is_dense = true;
    point_cloud2.point_step = num_channels * sizeof(float);
    point_cloud2.row_step = point_cloud2.point_step * point_cloud2.width;
    point_cloud2.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        point_cloud2.fields[i].name = channel_id[i];
        point_cloud2.fields[i].offset = i * sizeof(float);
        point_cloud2.fields[i].count = 1;
        point_cloud2.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }
    
    point_cloud2.data.resize(point_cloud2.row_step * point_cloud2.height);

    unsigned char *cloud_ptr = &(point_cloud2.data[0]);

    for (unsigned int i = 0; i < point_cloud2.width; i++)
    {
        if (feature_points[i])
        {
            // Original data
            Eigen::Vector3f pMPw = feature_points[i]->GetWorldPos();

            // Apply world frame orientation for non-IMU cases
            if (sensor_type == ORB_SLAM3::System::MONOCULAR || sensor_type == ORB_SLAM3::System::STEREO)
            {
                Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), pMPw);
                Sophus::SE3f Twmp = Tc0w.inverse() * Tc0mp;
                pMPw = Twmp.translation();
            }

            tf2::Vector3 point_translation(pMPw.x(), pMPw.y(), pMPw.z());

            float data_array[num_channels] = {
                point_translation.x(),
                point_translation.y(),
                point_translation.z()
            };

            // Copying point data into the cloud data buffer at the correct offset
            std::memcpy(cloud_ptr + (i * point_cloud2.point_step), data_array, num_channels * sizeof(float));
        }
    }

    return point_cloud2;
}
