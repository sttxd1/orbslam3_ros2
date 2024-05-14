#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class RgbdSlamNode : public rclcpp::Node
{
public:
    RgbdSlamNode(ORB_SLAM3::System* pSLAM);

    ~RgbdSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    void GrabImageRGB(const ImageMsg::SharedPtr msgRGB);
    void GrabImageD(const ImageMsg::SharedPtr msgD);
    cv::Mat GetRGBImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetDImage(const ImageMsg::SharedPtr msg);
    void SyncTwoImages();

    rclcpp::Subscription<ImageMsg>::SharedPtr subRGB;
    rclcpp::Subscription<ImageMsg>::SharedPtr subD;

    ORB_SLAM3::System* m_SLAM;
    std::thread* syncThread;

    queue<ImageMsg::SharedPtr> imgRGBBuf, imgDBuf;
    std::mutex bufMutexRGB, bufMutexD;
    // typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> approximate_sync_policy;

    // void GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD);

    // ORB_SLAM3::System* m_SLAM;

    // cv_bridge::CvImageConstPtr cv_ptrRGB;
    // cv_bridge::CvImageConstPtr cv_ptrD;

    // std::shared_ptr<message_filters::Subscriber<ImageMsg> > rgb_sub;
    // std::shared_ptr<message_filters::Subscriber<ImageMsg> > depth_sub;

    // std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
};

#endif
