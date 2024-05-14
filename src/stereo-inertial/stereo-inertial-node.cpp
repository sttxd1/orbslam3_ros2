#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <WGS84toCartesian.hpp>
#define EARTH_RADIUS 6378137.0;

using std::placeholders::_1;
Sophus::SE3f Tc0w = Sophus::SE3f();

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    this->initial();
    // subImu_ = this->create_subscription<ImuMsg>("/imu/data_raw", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    // subImgLeft_ = this->create_subscription<ImageMsg>("/stereo/left/image_raw", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    // subImgRight_ = this->create_subscription<ImageMsg>("/stereo/right/image_raw", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    RCLCPP_INFO(this->get_logger(), "Stop this node.");
    // Delete sync thread
    // syncThread_->join();
    // delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("/home/peterstone/Datasets/map/KeyFrameTrajectory.txt");

    syncThread_->join();
    delete syncThread_;
    RCLCPP_INFO(this->get_logger(), "Done.");
}

void StereoInertialNode::initial(){
    subImu_ = this->create_subscription<ImuMsg>("/imu/data_raw", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("/stereo/left/image_raw", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("/stereo/right/image_raw", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    gps_publisher = this->create_subscription<GPSMsg>("/gps/fix", 100, std::bind(&StereoInertialNode::GrabGPS, this, _1));


    odom_publisher = this->create_publisher<OdomMsg>("/orbslam3/odom", 10);
    path_publisher = this->create_publisher<PathMsg>("/orbslam3/path", 10);
    gps_path_publisher = this->create_publisher<PathMsg>("/orbslam3/gps_path", 10);
    camera_pose_publisher = this->create_publisher<PoseStampedMsg>("/orbslam3/camera_pose", 10);
    // body_pose_publisher = this->create_publisher<TfPoseStampedMsg>("/orbslam3/body_pose", 10);
    tracked_point_cloud_publisher = this->create_publisher<PointCloud2Msg>("/orbslam3/tracked_pc", 10);
    all_point_cloud_publisher = this->create_publisher<PointCloud2Msg>("/orbslam3/all_pc", 10);
    // key_markers_publisher = this->create_publisher<MarkerMsg>("/orbslam3/kf_markers", 10);

    local2cammap_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    cam2body_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    cammap2cam_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    local2enu_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    Eigen::Vector3d local2cammap_position;
    Eigen::Quaterniond local2cammap_orientation;

    local2cammap_position.x() = 0.0;
    local2cammap_position.y() = 0.0;
    local2cammap_position.z() = 0.0;
    local2cammap_orientation.w() = 1.0;
    local2cammap_orientation.x() = 0.0;
    local2cammap_orientation.y() = 0.0;
    local2cammap_orientation.z() = 0.0;

    Eigen::Vector3d cam2body_position;
    Eigen::Quaterniond cam2body_orientation;

    cam2body_position.x() = 0.0;
    cam2body_position.y() = 0.0;
    cam2body_position.z() = 0.0;
    cam2body_orientation.w() = 1.0;
    cam2body_orientation.x() = 0.0;
    cam2body_orientation.y() = 0.0;
    cam2body_orientation.z() = 0.0;

    this->TF_Static_Broadcaster(local2cammap_static_broadcaster, "local_frame", "camera_map_frame", local2cammap_position, local2cammap_orientation);
    this->TF_Static_Broadcaster(cam2body_static_broadcaster, "camera_frame", "body_frame", cam2body_position, cam2body_orientation);
}   

void StereoInertialNode::TF_Static_Broadcaster(std::shared_ptr<tf2_ros::StaticTransformBroadcaster> name,std::string frame_id, std::string child_frame_id, Eigen::Vector3d position, Eigen::Quaterniond quaternion){
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->get_clock()->now();
    static_transform.header.frame_id = frame_id;  // Parent frame
    static_transform.child_frame_id = child_frame_id; // Child f rame
    static_transform.transform.translation.x = position.x();  // meters
    static_transform.transform.translation.y = position.y();
    static_transform.transform.translation.z = position.z();
    static_transform.transform.rotation.x = quaternion.x();
    static_transform.transform.rotation.y = quaternion.y();
    static_transform.transform.rotation.z = quaternion.z();
    static_transform.transform.rotation.w = quaternion.w();
    name->sendTransform(static_transform);
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

void StereoInertialNode::GrabGPS(const GPSMsg::SharedPtr GPSmsg){
    num_GPS_msg = num_GPS_msg + 1;
    double ENU_x, ENU_y, ENU_z;
    TfPoseStampedMsg local2enu_pose_msg;
    PoseStampedMsg temp_msg;


    if (num_GPS_msg == 1) {
        ENU_origin[0] = GPSmsg->latitude;
        ENU_origin[1] = GPSmsg->longitude;
        ENU_height = GPSmsg->altitude;
        GPS_inital = false;
    } else {
        std::array<double, 2> temp;
        temp[0] = GPSmsg->latitude;
        temp[1] = GPSmsg->longitude;
        std::array<double, 2> ENU_xy = wgs84::toCartesian(ENU_origin, temp);
        ENU_x = ENU_xy[0];
        ENU_y = ENU_xy[1];
        ENU_z = GPSmsg->altitude - ENU_height;
        local2enu_pose_msg.header.frame_id = "local_frame";
        local2enu_pose_msg.child_frame_id = "gps_frame";
        local2enu_pose_msg.header.stamp = GPSmsg->header.stamp;
        local2enu_pose_msg.transform.translation.x = ENU_y;
        local2enu_pose_msg.transform.translation.y = -ENU_x;
        local2enu_pose_msg.transform.translation.z = ENU_z;
        local2enu_broadcaster->sendTransform(local2enu_pose_msg);

        temp_msg.header.frame_id = "gps_frame";
        temp_msg.header.stamp = GPSmsg->header.stamp;
        temp_msg.pose.position.x = ENU_y;
        temp_msg.pose.position.y = -ENU_x;
        temp_msg.pose.position.z = ENU_z;


        GPS_path_msg.header.frame_id = "local_frame";
        GPS_path_msg.header.stamp = GPSmsg->header.stamp;
        GPS_path_msg.poses.push_back(temp_msg);
        gps_path_publisher->publish(GPS_path_msg);
    }
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.1;

    while (1)
    {
        cv::Mat imLeft, imRight, imRight_new, imLeft_new;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            // tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            rclcpp::Time msg_time = imgLeftBuf_.front()->header.stamp;
            tImLeft = Utility::StampToSec(msg_time);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                msg_time = imgLeftBuf_.front()->header.stamp;
                tImLeft = Utility::StampToSec(msg_time);
                // tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;
            

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft_new);
                clahe_->apply(imRight, imRight_new);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft_new, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight_new, M1r_, M2r_, cv::INTER_LINEAR);
            }

            Sophus::SE3f Tcc0 = SLAM_->TrackStereo(imLeft_new, imRight_new, tImLeft, vImuMeas);

            Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();

            this->Publish_Camera_Pose(Twc, msg_time);
            this->Pulish_Feature_Points(SLAM_->GetTrackedMapPoints(), msg_time);
            this->Pulish_All_Feature_Points(SLAM_->GetAllMapPoints(), msg_time);


            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void StereoInertialNode::Publish_Camera_Pose(Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time){
    TfPoseStampedMsg cammap2cam_pose_msg;
    PoseStampedMsg temp_msg;

    cammap2cam_pose_msg.header.frame_id = "camera_map_frame";
    cammap2cam_pose_msg.child_frame_id = "camera_frame";
    cammap2cam_pose_msg.header.stamp = msg_time;

    cammap2cam_pose_msg.transform.translation.x = Twc_SE3f.translation().x();
    cammap2cam_pose_msg.transform.translation.y = Twc_SE3f.translation().y();
    cammap2cam_pose_msg.transform.translation.z = Twc_SE3f.translation().z();

    cammap2cam_pose_msg.transform.rotation.w = Twc_SE3f.unit_quaternion().coeffs().w();
    cammap2cam_pose_msg.transform.rotation.x = Twc_SE3f.unit_quaternion().coeffs().x();
    cammap2cam_pose_msg.transform.rotation.y = Twc_SE3f.unit_quaternion().coeffs().y();
    cammap2cam_pose_msg.transform.rotation.z = Twc_SE3f.unit_quaternion().coeffs().z();

    temp_msg.header.frame_id = "camera_map_frame";
    // temp_msg.header.stamp = msg_time;

    temp_msg.pose.position.x = Twc_SE3f.translation().x();
    temp_msg.pose.position.y = Twc_SE3f.translation().y();
    temp_msg.pose.position.z = Twc_SE3f.translation().z();

    temp_msg.pose.orientation.w = Twc_SE3f.unit_quaternion().coeffs().w();
    temp_msg.pose.orientation.x = Twc_SE3f.unit_quaternion().coeffs().x();
    temp_msg.pose.orientation.y = Twc_SE3f.unit_quaternion().coeffs().y();
    temp_msg.pose.orientation.z = Twc_SE3f.unit_quaternion().coeffs().z();

    cammap2cam_broadcaster->sendTransform(cammap2cam_pose_msg);

    cammap_path_msg.header.frame_id = "camera_map_frame";
    cammap_path_msg.header.stamp = msg_time;
    cammap_path_msg.poses.push_back(temp_msg);

    camera_pose_publisher->publish(temp_msg);

    path_publisher->publish(cammap_path_msg);
}

void StereoInertialNode::Pulish_All_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
    PointCloud2Msg pointcloud = Feature_Points_to_Point_cloud(feature_points, msg_time);
    all_point_cloud_publisher->publish(pointcloud);
}

void StereoInertialNode::Pulish_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
    PointCloud2Msg pointcloud = Feature_Points_to_Point_cloud(feature_points, msg_time);
    tracked_point_cloud_publisher->publish(pointcloud);
}

sensor_msgs::msg::PointCloud2 StereoInertialNode::Feature_Points_to_Point_cloud(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
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
