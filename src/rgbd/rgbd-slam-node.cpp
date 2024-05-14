#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    subRGB = this->create_subscription<ImageMsg>("/camera/color/image_raw", 1000, std::bind(&RgbdSlamNode::GrabImageRGB, this, _1));
    subD = this->create_subscription<ImageMsg>("/camera/depth/image_rect_raw", 1000, std::bind(&RgbdSlamNode::GrabImageD, this, _1));
    syncThread = new std::thread(&RgbdSlamNode::SyncTwoImages, this);
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
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
        double tImLeft = 0, tImRight = 0;
        if (!imgRGBBuf.empty() && !imgDBuf.empty())
        {
            tImLeft = Utility::StampToSec(imgRGBBuf.front()->header.stamp);
            tImRight = Utility::StampToSec(imgDBuf.front()->header.stamp);

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

            bufMutexRGB.lock();
            imgRGB = GetRGBImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            bufMutexRGB.unlock();

            bufMutexD.lock();
            imgD = GetDImage(imgDBuf.front());
            imgDBuf.pop();
            bufMutexD.unlock();

            m_SLAM->TrackRGBD(imgRGB, imgD, tImLeft);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
