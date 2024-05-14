#ifndef __MONO_INERTIAL_NODE_HPP__
#define __MONO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonoIMUSlamNode : public rclcpp::Node
{
public:
    MonoIMUSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonoIMUSlamNode();

private:

    using ImuMsg = sensor_msgs::msg::Imu;
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;

    ORB_SLAM3::System *mSLAM_;
    std::thread *syncThread_;
    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufIMUMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufImgMutex_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
