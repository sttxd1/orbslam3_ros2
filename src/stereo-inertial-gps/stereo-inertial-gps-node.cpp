#include "stereo-inertial-gps-node.hpp"

#include <opencv2/core/core.hpp>
#include <WGS84toCartesian.hpp>
#define EARTH_RADIUS 6378137.0;

using std::placeholders::_1;
Sophus::SE3f Tc0w = Sophus::SE3f();

StereoInertialGPSNode::StereoInertialGPSNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
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

    syncThread_ = new std::thread(&StereoInertialGPSNode::SyncWithImu, this);
}

StereoInertialGPSNode::~StereoInertialGPSNode()
{
    RCLCPP_INFO(this->get_logger(), "Stop this node.");
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("/home/peterstone/Datasets/map/KeyFrameTrajectory.txt");

    // syncThread_->join();
    // delete syncThread_;
    RCLCPP_INFO(this->get_logger(), "Done.");
}

void StereoInertialGPSNode::initial(){
    subImu_ = this->create_subscription<ImuMsg>("/imu/data_raw", 1000, std::bind(&StereoInertialGPSNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("/stereo/left/image_raw", 100, std::bind(&StereoInertialGPSNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("/stereo/right/image_raw", 100, std::bind(&StereoInertialGPSNode::GrabImageRight, this, _1));
    gps_publisher = this->create_subscription<GPSMsg>("/gps/fix", 100, std::bind(&StereoInertialGPSNode::GrabGPS, this, _1));


    odom_publisher = this->create_publisher<OdomMsg>("/orbslam3/odom", 10);
    path_publisher = this->create_publisher<PathMsg>("/orbslam3/path", 10);
    gps_path_publisher = this->create_publisher<PathMsg>("/orbslam3/gps_path", 10);
    camera_pose_publisher = this->create_publisher<PoseStampedMsg>("/orbslam3/camera_pose", 10);
    // body_pose_publisher = this->create_publisher<TfPoseStampedMsg>("/orbslam3/body_pose", 10);
    tracked_point_cloud_publisher = this->create_publisher<PointCloud2Msg>("/orbslam3/tracked_pc", 100);
    all_point_cloud_publisher = this->create_publisher<PointCloud2Msg>("/orbslam3/all_pc", 100);
    // key_markers_publisher = this->create_publisher<MarkerMsg>("/orbslam3/kf_markers", 10);

    world2orbmap_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    orb2body_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    body2gps_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    world2enu_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    orbmap2orb_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    gps2enu_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    Eigen::Vector3d world2orbmap_position;
    Eigen::Quaterniond world2orbmap_orientation;

    world2orbmap_position.x() = 0.0;
    world2orbmap_position.y() = 0.0;
    world2orbmap_position.z() = 0.0; 
    world2orbmap_orientation.w() = std::sqrt(2.0)/2;
    world2orbmap_orientation.x() = 0.0;
    world2orbmap_orientation.y() = 0.0;
    world2orbmap_orientation.z() = -std::sqrt(2.0)/2;

    Eigen::Vector3d orb2body_position;
    Eigen::Quaterniond orb2body_orientation;

    orb2body_position.x() = 0.0;
    orb2body_position.y() = 0.0;
    orb2body_position.z() = 0.0;
    orb2body_orientation.w() = 0.5;
    orb2body_orientation.x() = 0.5;
    orb2body_orientation.y() = -0.5;
    orb2body_orientation.z() = 0.5;

    Eigen::Vector3d world2enu_position;
    Eigen::Quaterniond world2enu_orientation;

    world2enu_position.x() = 0.0;
    world2enu_position.y() = 0.0;
    world2enu_position.z() = 0.0;
    world2enu_orientation.w() = 0.0;
    world2enu_orientation.x() = 0.0;
    world2enu_orientation.y() = 0.0;
    world2enu_orientation.z() = 1.0;

    // Eigen::Vector3d body2gps_position;
    // Eigen::Quaterniond body2gps_orientation;

    // body2gps_position.x() = 0.0;
    // body2gps_position.y() = 0.0;
    // body2gps_position.z() = 25.0;
    // body2gps_orientation.w() = 1.0;
    // body2gps_orientation.x() = 0.0;
    // body2gps_orientation.y() = 0.0;
    // body2gps_orientation.z() = 0.0;

    this->TF_Static_Broadcaster(world2orbmap_static_broadcaster, "world_frame", "orb_map_frame", world2orbmap_position, world2orbmap_orientation);
    this->TF_Static_Broadcaster(orb2body_static_broadcaster, "orb_frame", "body_frame", orb2body_position, orb2body_orientation);
    // this->TF_Static_Broadcaster(body2gps_static_broadcaster, "body_frame", "gps_frame", body2gps_position, body2gps_orientation);
    this->TF_Static_Broadcaster(world2enu_static_broadcaster, "world_frame", "enu_frame", world2enu_position, world2enu_orientation);
}   

void StereoInertialGPSNode::TF_Static_Broadcaster(std::shared_ptr<tf2_ros::StaticTransformBroadcaster> name,std::string frame_id, std::string child_frame_id, Eigen::Vector3d position, Eigen::Quaterniond quaternion){
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

void StereoInertialGPSNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialGPSNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialGPSNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

void StereoInertialGPSNode::GrabGPS(const GPSMsg::SharedPtr GPSmsg){
    num_GPS_msg = num_GPS_msg + 1;
    double ENU_x, ENU_y, ENU_z;
    TfPoseStampedMsg gps2enu_pose_msg;
    PoseStampedMsg temp_msg;

    if (num_GPS_msg == 1) {
        ENU_origin[0] = GPSmsg->latitude;
        ENU_origin[1] = GPSmsg->longitude;
        ENU_height = GPSmsg->altitude;
        // GPS_inital = false;
    } else {
        std::array<double, 2> temp;
        temp[0] = GPSmsg->latitude;
        temp[1] = GPSmsg->longitude;
        std::array<double, 2> ENU_xy = wgs84::toCartesian(ENU_origin, temp);
        ENU_x = ENU_xy[0];
        ENU_y = ENU_xy[1];
        ENU_z = GPSmsg->altitude - ENU_height;
        gps2enu_pose_msg.header.frame_id = "enu_frame";
        gps2enu_pose_msg.child_frame_id = "gps_frame";
        gps2enu_pose_msg.header.stamp = GPSmsg->header.stamp;
        gps2enu_pose_msg.transform.translation.x = ENU_x;
        gps2enu_pose_msg.transform.translation.y = ENU_y;
        gps2enu_pose_msg.transform.translation.z = ENU_z;
        gps2enu_broadcaster->sendTransform(gps2enu_pose_msg);

        temp_msg.header.frame_id = "gps_frame";
        temp_msg.header.stamp = GPSmsg->header.stamp;
        // temp_msg.pose.position.x = ENU_y;
        // temp_msg.pose.position.y = -ENU_x;
        // temp_msg.pose.position.z = ENU_z;
        temp_msg.pose.position.x = ENU_x;
        temp_msg.pose.position.y = ENU_y;
        temp_msg.pose.position.z = ENU_z;

        // GPSmsg_new->header.stamp = GPSmsg->header.stamp;
        GPSmsg->latitude = ENU_y;
        GPSmsg->longitude = -ENU_x;
        GPSmsg->altitude = ENU_z;


        GPS_path_msg.header.frame_id = "enu_frame";
        GPS_path_msg.header.stamp = GPSmsg->header.stamp;
        GPS_path_msg.poses.push_back(temp_msg);
        gps_path_publisher->publish(GPS_path_msg);
    }

    bufMutexgps_.lock();
    if (!gpsBuf_.empty())
        gpsBuf_.pop();
    gpsBuf_.push(GPSmsg);
    bufMutexgps_.unlock();
}

cv::Mat StereoInertialGPSNode::GetImage(const ImageMsg::SharedPtr msg)
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

void StereoInertialGPSNode::SyncWithImu()
{
    const double maxTimeDiff = 0.1;

    while (1)
    {
        cv::Mat imLeft, imRight, imRight_new, imLeft_new;
        double tImLeft = 0, tImRight = 0, tGPS = -1;
        // RCLCPP_INFO(this->get_logger(), "Hello0");
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
            vector<ORB_SLAM3::GPS::Data> vGPS;
            
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
            float gpsx = -1, gpsy = -1;
            bufMutexgps_.lock();
            if(!gpsBuf_.empty()){
                tGPS = Utility::StampToSec(gpsBuf_.front()->header.stamp);
                // TO BE CHANGED
                if(tImLeft - tGPS < 0.1 && tImLeft > tGPS) {
                    vGPS.clear();
                    vGPS.push_back(ORB_SLAM3::GPS::Data(gpsBuf_.front()->latitude, gpsBuf_.front()->longitude, gpsBuf_.front()->altitude, Utility::StampToSec(gpsBuf_.front()->header.stamp)));
                    gpsx = gpsBuf_.front()->latitude;
                    gpsy = gpsBuf_.front()->longitude;
                    gpsBuf_.pop(); 
                }
            }
            bufMutexgps_.unlock();


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

            // RCLCPP_INFO(this->get_logger(), "Stereo: %f, GPS time: %f", tImLeft, tGPS);
            Sophus::SE3f Tcc0 = SLAM_->TrackStereoGPS(imLeft_new, imRight_new, tImLeft, vImuMeas, vGPS);
            // RCLCPP_INFO(this->get_logger(), "Hello2");
            Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();

            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, gpsx: %f, gpsy: %f", Twc.translation().x(), Twc.translation().y(), gpsx, gpsy);
            this->Publish_Camera_Pose(Twc, msg_time);
            this->Pulish_Feature_Points(SLAM_->GetTrackedMapPoints(), msg_time);
            this->Pulish_All_Feature_Points(SLAM_->GetAllMapPoints(), msg_time);


            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void StereoInertialGPSNode::Publish_Camera_Pose(Sophus::SE3f Twc_SE3f, rclcpp::Time msg_time){
    TfPoseStampedMsg orbmap2orb_pose_msg;
    PoseStampedMsg temp_msg;

    orbmap2orb_pose_msg.header.frame_id = "orb_map_frame";
    orbmap2orb_pose_msg.child_frame_id = "orb_frame";
    orbmap2orb_pose_msg.header.stamp = msg_time;

    orbmap2orb_pose_msg.transform.translation.x = Twc_SE3f.translation().x();
    orbmap2orb_pose_msg.transform.translation.y = Twc_SE3f.translation().y();
    orbmap2orb_pose_msg.transform.translation.z = Twc_SE3f.translation().z();

    orbmap2orb_pose_msg.transform.rotation.w = Twc_SE3f.unit_quaternion().coeffs().w();
    orbmap2orb_pose_msg.transform.rotation.x = Twc_SE3f.unit_quaternion().coeffs().x();
    orbmap2orb_pose_msg.transform.rotation.y = Twc_SE3f.unit_quaternion().coeffs().y();
    orbmap2orb_pose_msg.transform.rotation.z = Twc_SE3f.unit_quaternion().coeffs().z();

    temp_msg.header.frame_id = "orb_map_frame";
    // temp_msg.header.stamp = msg_time;

    temp_msg.pose.position.x = Twc_SE3f.translation().x();
    temp_msg.pose.position.y = Twc_SE3f.translation().y();
    temp_msg.pose.position.z = Twc_SE3f.translation().z();

    temp_msg.pose.orientation.w = Twc_SE3f.unit_quaternion().coeffs().w();
    temp_msg.pose.orientation.x = Twc_SE3f.unit_quaternion().coeffs().x();
    temp_msg.pose.orientation.y = Twc_SE3f.unit_quaternion().coeffs().y();
    temp_msg.pose.orientation.z = Twc_SE3f.unit_quaternion().coeffs().z();

    orbmap2orb_broadcaster->sendTransform(orbmap2orb_pose_msg);

    orb_path_msg.header.frame_id = "orb_map_frame";
    orb_path_msg.header.stamp = msg_time;
    orb_path_msg.poses.push_back(temp_msg);
 
    camera_pose_publisher->publish(temp_msg);

    path_publisher->publish(orb_path_msg);
}

void StereoInertialGPSNode::Pulish_All_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
    PointCloud2Msg pointcloud = Feature_Points_to_Point_cloud(feature_points, msg_time);
    all_point_cloud_publisher->publish(pointcloud);
}

void StereoInertialGPSNode::Pulish_Feature_Points(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
    PointCloud2Msg pointcloud = Feature_Points_to_Point_cloud(feature_points, msg_time);
    tracked_point_cloud_publisher->publish(pointcloud);
}

sensor_msgs::msg::PointCloud2 StereoInertialGPSNode::Feature_Points_to_Point_cloud(std::vector<ORB_SLAM3::MapPoint*> feature_points, rclcpp::Time msg_time){
    const int num_channels = 3;
    if (feature_points.size() == 0){
        RCLCPP_INFO(this->get_logger(), "Empty feature points.");
    }

    sensor_msgs::msg::PointCloud2 point_cloud2;
    point_cloud2.header.frame_id = "orb_map_frame";
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
                static_cast<float>(point_translation.x()),
                static_cast<float>(point_translation.y()),
                static_cast<float>(point_translation.z())
            };

            // Copying point data into the cloud data buffer at the correct offset
            std::memcpy(cloud_ptr + (i * point_cloud2.point_step), data_array, num_channels * sizeof(float));
        }
    }

    return point_cloud2;
}
