#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>

#include "Frame.h"
#include "Map.h"
#include "System.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class StereoInertialNode : public rclcpp::Node {
public:
  StereoInertialNode(ORB_SLAM3::System *pSLAM, const string &strSettingsFile,
                     const string &strDoRectify, const string &strDoEqual);
  ~StereoInertialNode();

private:
  void GrabImu(const ImuMsg::SharedPtr msg);
  void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
  void GrabImageRight(const ImageMsg::SharedPtr msgRight);
  cv::Mat GetImage(const ImageMsg::SharedPtr msg);
  void SyncWithImu();

  rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
  rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
  rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;

  ORB_SLAM3::System *SLAM_;
  std::thread *syncThread_;

  // IMU
  queue<ImuMsg::SharedPtr> imuBuf_;
  std::mutex bufMutex_;

  // Image
  queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
  std::mutex bufMutexLeft_, bufMutexRight_;

  bool doRectify_;
  bool doEqual_;
  cv::Mat M1l_, M2l_, M1r_, M2r_;

  bool bClahe_;
  cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

  // outputs of features
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLandmarks_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubDesc_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  tf2_ros::StaticTransformBroadcaster staticBroadcaster_;
  Sophus::SE3f T_baselink_camera_;
};

#endif
