#include "mono-inertial-node.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonoIMUSlamNode::MonoIMUSlamNode(ORB_SLAM3::System *pSLAM) : Node("ORB_SLAM3_ROS2")
{
    mSLAM_ = pSLAM;

    subImu_ = this->create_subscription<ImuMsg>("/camera/imu", 1000, std::bind(&MonoIMUSlamNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("/camera/color/image_raw", 100, std::bind(&MonoIMUSlamNode::GrabImage, this, _1));

    syncThread_ = new std::thread(&MonoIMUSlamNode::SyncWithImu, this);
}

MonoIMUSlamNode::~MonoIMUSlamNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;
    // Stop all threads
    mSLAM_->Shutdown();
    // Save camera trajectory
    mSLAM_->SaveKeyFrameTrajectoryTUM("/home/peter/KeyFrameTrajectory_imu.txt");
}

void MonoIMUSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufIMUMutex_.lock();
    imuBuf_.push(msg);
    bufIMUMutex_.unlock();
}

void MonoIMUSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufImgMutex_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msg);

    bufImgMutex_.unlock();
}

cv::Mat MonoIMUSlamNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    // if (cv_ptr->image.type() == 1)
    // {
    return cv_ptr->image.clone();
    // }
    // else
    // {
    //     std::cerr << "Error image type" << std::endl;
    //     return cv_ptr->image.clone();
    // }
}

void MonoIMUSlamNode::SyncWithImu()
{

    while (1)
    {
        cv::Mat img;
        double tImg = 0;
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);

            if (tImg > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;
            {
                bufImgMutex_.lock();
                img = GetImage(imgBuf_.front());
                imgBuf_.pop();
                bufImgMutex_.unlock();
            }

            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "aaaa image with header: a = '%f',  b ='%f", Utility::StampToSec(imuBuf_.front()->header.stamp), tImg);
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "len: '%d", imuBuf_.empty());
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufIMUMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                // RCLCPP_INFO(
                //         this->get_logger(),
                //         "Received image with header: a = '%f',  b ='%f", Utility::StampToSec(imuBuf_.front()->header.stamp), tImg);
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImg)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    // std::cout << "Acceleration: ("
                    //     << acc.x << ", "
                    //     << acc.y << ", "
                    //     << acc.z << ")" << std::endl;
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                    // std::cerr << "nice2" << std::endl;
                }
            }
            bufIMUMutex_.unlock();
            mSLAM_->TrackMonocular(img, tImg, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
