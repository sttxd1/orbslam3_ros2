#include "rgbd-inertial-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    subImu = this->create_subscription<ImuMsg>("/fliped/imu", 1000, std::bind(&RgbdSlamNode::GrabImu, this, _1));
    subRGB = this->create_subscription<ImageMsg>("/camera/color/image_raw", 100, std::bind(&RgbdSlamNode::GrabImageRGB, this, _1));
    subD = this->create_subscription<ImageMsg>("/camera/depth/image_rect_raw", 100, std::bind(&RgbdSlamNode::GrabImageD, this, _1));
    syncThread = new std::thread(&RgbdSlamNode::SyncTwoImages, this);
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabImu(const ImuMsg::SharedPtr msgImu){
    bufMutexImu.lock();
    imuBuf.push(msgImu);
    bufMutexImu.unlock();
}

void RgbdSlamNode::GrabImageRGB(const ImageMsg::SharedPtr msgRGB){
    bufMutexRGB.lock();

    if (!imgRGBBuf.empty())
        imgRGBBuf.pop();
    imgRGBBuf.push(msgRGB);

    bufMutexRGB.unlock();
}

void RgbdSlamNode::GrabImageD(const ImageMsg::SharedPtr msgD){
    bufMutexD.lock();
    
    if (!imgDBuf.empty())
        imgDBuf.pop();
    imgDBuf.push(msgD);

    bufMutexD.unlock();
}

cv::Mat RgbdSlamNode::GetRGBImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {   
        // std::cerr << msg->encoding << std::endl;
        if (msg->encoding == "rgb8"){
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            // std::cerr << "Error image type_123" << std::endl;
            // std::cout << cv_ptr->image.type() << std::endl;
        }
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 16)
    {
        return cv_ptr->image.clone();
    }
    else
    {   
        std::cout << cv_ptr->image.type() << std::endl;
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

cv::Mat RgbdSlamNode::GetDImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {   
        if (msg->encoding == "16UC1"){
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            // std::cerr << "Error image type_2" << std::endl;
            // std::cout << cv_ptr->image.type() << std::endl;
        }
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 2)
    {
        return cv_ptr->image.clone();
    }
    else
    {   
        std::cout << cv_ptr->image.type() << std::endl;
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}



void RgbdSlamNode::SyncTwoImages()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imgRGB, imgD;
        double tImLeft = 0, tImRight = 0, tImu = 0;
        if (!imgRGBBuf.empty() && !imgDBuf.empty() && !imuBuf.empty())
        {
            tImLeft = Utility::StampToSec(imgRGBBuf.front()->header.stamp);
            tImRight = Utility::StampToSec(imgDBuf.front()->header.stamp);
            tImu = Utility::StampToSec(imuBuf.front()->header.stamp);

            bufMutexD.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgDBuf.size() > 1)
            {
                imgDBuf.pop();
                tImRight = Utility::StampToSec(imgDBuf.front()->header.stamp);
            }
            bufMutexD.unlock();

            bufMutexRGB.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgRGBBuf.size() > 1)
            {
                imgRGBBuf.pop();
                tImLeft = Utility::StampToSec(imgRGBBuf.front()->header.stamp);
            }
            bufMutexRGB.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Before continue");
            if (tImLeft > Utility::StampToSec(imuBuf.back()->header.stamp))
                continue;

            bufMutexRGB.lock();
            imgRGB = GetRGBImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            bufMutexRGB.unlock();

            bufMutexD.lock();
            imgD = GetDImage(imgDBuf.front());
            imgDBuf.pop();
            bufMutexD.unlock();
            RCLCPP_INFO(this->get_logger(), "Before IMU");
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutexImu.lock();
            if (!imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf.empty() && Utility::StampToSec(imuBuf.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf.front()->header.stamp);
                    cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf.pop();
                }
            }
            bufMutexImu.unlock();

            m_SLAM->TrackRGBD(imgRGB, imgD, tImLeft, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
