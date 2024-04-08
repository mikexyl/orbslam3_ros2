#include "stereo-inertial-node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM,
                                       const string &strSettingsFile,
                                       const string &strDoRectify,
                                       const string &strDoEqual)
    : Node("ORB_SLAM3_ROS2"), SLAM_(SLAM), tfBroadcaster_(this),
      staticBroadcaster_(this) {
  stringstream ss_rec(strDoRectify);
  ss_rec >> boolalpha >> doRectify_;

  stringstream ss_eq(strDoEqual);
  ss_eq >> boolalpha >> doEqual_;

  bClahe_ = doEqual_;
  std::cout << "Rectify: " << doRectify_ << std::endl;
  std::cout << "Equal: " << doEqual_ << std::endl;

  cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    assert(0);
  }
  cv::Mat T_baselink_camera_mat;
  fsSettings["IMU.T_b_c1"] >> T_baselink_camera_mat;
  Eigen::Matrix3f R_baselink_camera_eigen = Eigen::Matrix3f::Identity();
  R_baselink_camera_eigen(0, 0) = T_baselink_camera_mat.at<float>(0, 0);
  R_baselink_camera_eigen(0, 1) = T_baselink_camera_mat.at<float>(0, 1);
  R_baselink_camera_eigen(0, 2) = T_baselink_camera_mat.at<float>(0, 2);
  R_baselink_camera_eigen(1, 0) = T_baselink_camera_mat.at<float>(1, 0);
  R_baselink_camera_eigen(1, 1) = T_baselink_camera_mat.at<float>(1, 1);
  R_baselink_camera_eigen(1, 2) = T_baselink_camera_mat.at<float>(1, 2);
  R_baselink_camera_eigen(2, 0) = T_baselink_camera_mat.at<float>(2, 0);
  R_baselink_camera_eigen(2, 1) = T_baselink_camera_mat.at<float>(2, 1);
  R_baselink_camera_eigen(2, 2) = T_baselink_camera_mat.at<float>(2, 2);
  Eigen::Vector3f t_baselink_camera_eigen;
  t_baselink_camera_eigen(0) = T_baselink_camera_mat.at<float>(0, 3);
  t_baselink_camera_eigen(1) = T_baselink_camera_mat.at<float>(1, 3);
  t_baselink_camera_eigen(2) = T_baselink_camera_mat.at<float>(2, 3);

  T_baselink_camera_.setRotationMatrix(R_baselink_camera_eigen);
  T_baselink_camera_.translation().x() = t_baselink_camera_eigen(0);
  T_baselink_camera_.translation().y() = t_baselink_camera_eigen(1);
  T_baselink_camera_.translation().z() = t_baselink_camera_eigen(2);

  // dynamic reconfigure rpy
  rpy_odom_origin_ = {0, 0, 0};
  this->declare_parameter<double>("roll", rpy_odom_origin_[0]);
  this->declare_parameter<double>("pitch", rpy_odom_origin_[1]);
  this->declare_parameter<double>("yaw", rpy_odom_origin_[2]);

  auto param_change_callback =
      [this](std::vector<rclcpp::Parameter> parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          if (parameter.get_name() == "roll") {
            rpy_odom_origin_[0] = parameter.as_double() * M_PI / 180;
          } else if (parameter.get_name() == "pitch") {
            rpy_odom_origin_[1] = parameter.as_double() * M_PI / 180;
          } else if (parameter.get_name() == "yaw") {
            rpy_odom_origin_[2] = parameter.as_double() * M_PI / 180;
          }
        }
        return result;
      };

  paramCbHandle_ = this->add_on_set_parameters_callback(param_change_callback);

  if (doRectify_) {

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

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
        R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
      cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
           << endl;
      assert(0);
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l,
                                P_l.rowRange(0, 3).colRange(0, 3),
                                cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
    cv::initUndistortRectifyMap(K_r, D_r, R_r,
                                P_r.rowRange(0, 3).colRange(0, 3),
                                cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
  }

  subImu_ = this->create_subscription<ImuMsg>(
      "imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
  subImgLeft_ = this->create_subscription<ImageMsg>(
      "camera/left", 100,
      std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
  subImgRight_ = this->create_subscription<ImageMsg>(
      "camera/right", 100,
      std::bind(&StereoInertialNode::GrabImageRight, this, _1));

  pubLandmarks_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("landmarks", 1000);
  pubDesc_ =
      this->create_publisher<sensor_msgs::msg::Image>("descriptors", 1000);
  pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);
  pubPath_ = this->create_publisher<nav_msgs::msg::Path>("path", 1000);

  syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode() {
  // Delete sync thread
  syncThread_->join();
  delete syncThread_;

  // Stop all threads
  SLAM_->Shutdown();

  // Save camera trajectory
  SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg) {
  bufMutex_.lock();
  imuBuf_.push(msg);
  bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft) {
  bufMutexLeft_.lock();

  if (!imgLeftBuf_.empty())
    imgLeftBuf_.pop();
  imgLeftBuf_.push(msgLeft);

  bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight) {
  bufMutexRight_.lock();

  if (!imgRightBuf_.empty())
    imgRightBuf_.pop();
  imgRightBuf_.push(msgRight);

  bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0) {
    return cv_ptr->image.clone();
  } else {
    std::cerr << "Error image type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void StereoInertialNode::SyncWithImu() {
  const double maxTimeDiff = 0.01;

  while (1) {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    rclcpp::Time tImLeftRos;
    std::string sImLeftFrame, sImRightFrame;
    if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty()) {
      tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
      tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

      tImLeftRos = imgLeftBuf_.front()->header.stamp;

      sImLeftFrame = imgLeftBuf_.front()->header.frame_id;
      sImRightFrame = imgRightBuf_.front()->header.frame_id;

      bufMutexRight_.lock();
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1) {
        imgRightBuf_.pop();
        tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
      }
      bufMutexRight_.unlock();

      bufMutexLeft_.lock();
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1) {
        imgLeftBuf_.pop();
        tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
      }
      bufMutexLeft_.unlock();

      if ((tImLeft - tImRight) > maxTimeDiff ||
          (tImRight - tImLeft) > maxTimeDiff) {
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
      if (!imuBuf_.empty()) {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!imuBuf_.empty() &&
               Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft) {
          double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
          cv::Point3f acc(imuBuf_.front()->linear_acceleration.x,
                          imuBuf_.front()->linear_acceleration.y,
                          imuBuf_.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf_.front()->angular_velocity.x,
                          imuBuf_.front()->angular_velocity.y,
                          imuBuf_.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          imuBuf_.pop();
        }
      }
      bufMutex_.unlock();

      if (bClahe_) {
        clahe_->apply(imLeft, imLeft);
        clahe_->apply(imRight, imRight);
      }

      if (doRectify_) {
        cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
      }

      SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

      Eigen::Matrix3f R_color_color_optical;
      R_color_color_optical << 0, 0, 1, -1, 0, 0, 0, -1, 0;

      // only publish when the system is "OK"
      auto status = SLAM_->GetTrackingState();
      if (status != ORB_SLAM3::Tracking::eTrackingState::OK)
        continue;

      sensor_msgs::msg::PointCloud2::SharedPtr landmark_cloud_msg(
          new sensor_msgs::msg::PointCloud2);
      landmark_cloud_msg->header.stamp = tImLeftRos;
      landmark_cloud_msg->header.frame_id = "map";
      landmark_cloud_msg->height = 1;

      sensor_msgs::PointCloud2Modifier modifier(*landmark_cloud_msg);
      // clang-format off
      modifier.setPointCloud2Fields(8, // no format
                                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "id_low", 1, sensor_msgs::msg::PointField::UINT32,
                                "id_high", 1, sensor_msgs::msg::PointField::UINT32,
                                "u", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "v", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "keypoint_id", 1, sensor_msgs::msg::PointField::UINT32
                               );
      // clang-format on

      // publish features
      auto const &landmarks = SLAM_->GetTrackedMapPoints();

      modifier.resize(landmarks.size());
      sensor_msgs::PointCloud2Iterator<float> x_it(*landmark_cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> y_it(*landmark_cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> z_it(*landmark_cloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<uint32_t> id_low_it(*landmark_cloud_msg,
                                                           "id_low");
      sensor_msgs::PointCloud2Iterator<uint32_t> id_high_it(*landmark_cloud_msg,
                                                            "id_high");
      sensor_msgs::PointCloud2Iterator<float> u_it(*landmark_cloud_msg, "u");
      sensor_msgs::PointCloud2Iterator<float> v_it(*landmark_cloud_msg, "v");
      sensor_msgs::PointCloud2Iterator<uint32_t> keypoint_id_it(
          *landmark_cloud_msg, "keypoint_id");

      std::set<long unsigned int> landmark_ids;

      // descriptor as image
      cv::Mat desc(landmarks.size(), 32, CV_8UC1);
      size_t nGoodLandmarks = 0;

      for (size_t i = 0; i < landmarks.size(); i++) {
        auto const &landmark = landmarks[i];
        if (landmark == nullptr or landmark->isBad() or
            landmark_ids.count(landmark->mnId))
          continue;
        *x_it = landmark->GetWorldPos().x();
        *y_it = landmark->GetWorldPos().y();
        *z_it = landmark->GetWorldPos().z();

        *id_low_it = landmark->mnId & 0xFFFFFFFF;
        *id_high_it = landmark->mnId >> 32;
        *u_it = -1; // since landmarks only save its observation in keyframes,
                    // we don't have u, v
        *v_it = -1;
        *keypoint_id_it = nGoodLandmarks;

        // copy to row nGoodLandmarks
        memcpy(desc.ptr<uchar>(nGoodLandmarks), landmark->GetDescriptor().data,
               32);
        assert(landmark->GetDescriptor().cols == 32);
        nGoodLandmarks++;

        ++x_it;
        ++y_it;
        ++z_it;
        ++id_low_it;
        ++id_high_it;
        ++u_it;
        ++v_it;
        ++keypoint_id_it;
      }

      modifier.resize(nGoodLandmarks);

      // resize the descriptor image
      cv::Mat valid_desc_image = desc.rowRange(0, nGoodLandmarks);

      sensor_msgs::msg::Image desc_msg;
      cv_bridge::CvImage desc_cv;
      desc_cv.image = valid_desc_image;
      desc_cv.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      desc_cv.toImageMsg(desc_msg);
      desc_msg.header.stamp = tImLeftRos;
      desc_msg.header.frame_id = sImLeftFrame;

      pubLandmarks_->publish(*landmark_cloud_msg);
      pubDesc_->publish(desc_msg);

      auto se3ToTF = [](const Sophus::SE3f &T) {
        geometry_msgs::msg::Transform tf;
        tf.translation.x = T.translation().x();
        tf.translation.y = T.translation().y();
        tf.translation.z = T.translation().z();
        tf.rotation.x = T.unit_quaternion().x();
        tf.rotation.y = T.unit_quaternion().y();
        tf.rotation.z = T.unit_quaternion().z();
        tf.rotation.w = T.unit_quaternion().w();
        return tf;
      };

      geometry_msgs::msg::TransformStamped T_camera_camera_optical_tf;
      T_camera_camera_optical_tf.header.stamp = tImLeftRos;
      T_camera_camera_optical_tf.header.frame_id = sImLeftFrame + "_vio";
      T_camera_camera_optical_tf.child_frame_id = sImLeftFrame;
      T_camera_camera_optical_tf.transform = se3ToTF(
          Sophus::SE3f(R_color_color_optical, Eigen::Vector3f(0, 0, 0)));

      staticBroadcaster_.sendTransform(T_camera_camera_optical_tf);

      std::map<double, Sophus::SE3f> optimized_trajectory_optical;
      Sophus::SE3f Two;
      SLAM_->GetTrajectory(optimized_trajectory_optical, Two);

      auto optical_to_camera =
          [](const std::map<double, Sophus::SE3f> &optimized_trajectory) {
            std::map<double, Sophus::SE3f> Twc;
            for (auto &[timestamp, Twc_optical] : optimized_trajectory) {
              Eigen::Matrix3f R_color_color_optical;
              R_color_color_optical << 0, 0, 1, -1, 0, 0, 0, -1, 0;
              Twc.emplace(timestamp,
                          Twc_optical *
                              Sophus::SE3f(R_color_color_optical.inverse(),
                                           Eigen::Vector3f(0, 0, 0)));
            }
            return Twc;
          };

      auto optimized_trajectory =
          optical_to_camera(optimized_trajectory_optical);

      nav_msgs::msg::Path path_msg;
      for (auto const &[timestamp, Twc] : optimized_trajectory) {
        geometry_msgs::msg::PoseStamped pose_msg;
        rclcpp::Time t(timestamp * 1e9);
        pose_msg.header.stamp = t;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.position.x = Twc.translation().x();
        pose_msg.pose.position.y = Twc.translation().y();
        pose_msg.pose.position.z = Twc.translation().z();

        pose_msg.pose.orientation.x = Twc.unit_quaternion().x();
        pose_msg.pose.orientation.y = Twc.unit_quaternion().y();
        pose_msg.pose.orientation.z = Twc.unit_quaternion().z();
        pose_msg.pose.orientation.w = Twc.unit_quaternion().w();

        path_msg.poses.push_back(pose_msg);
      }

      auto Toc_current = optimized_trajectory.rbegin()->second;

      path_msg.header.stamp = tImLeftRos;
      path_msg.header.frame_id = "odom";
      pubPath_->publish(path_msg);

      geometry_msgs::msg::TransformStamped T_map_odom_tf;
      T_map_odom_tf.header.stamp = tImLeftRos;
      T_map_odom_tf.header.frame_id = "map";
      T_map_odom_tf.child_frame_id = "odom";
      T_map_odom_tf.transform = se3ToTF(Two);

      staticBroadcaster_.sendTransform(T_map_odom_tf);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = tImLeftRos;
      odom_msg.header.frame_id = "odom_origin";
      odom_msg.child_frame_id = sImLeftFrame + "_vio";
      odom_msg.pose.pose.position.x = Toc_current.translation().x();
      odom_msg.pose.pose.position.y = Toc_current.translation().y();
      odom_msg.pose.pose.position.z = Toc_current.translation().z();

      odom_msg.pose.pose.orientation.x = Toc_current.unit_quaternion().x();
      odom_msg.pose.pose.orientation.y = Toc_current.unit_quaternion().y();
      odom_msg.pose.pose.orientation.z = Toc_current.unit_quaternion().z();
      odom_msg.pose.pose.orientation.w = Toc_current.unit_quaternion().w();

      pubOdom_->publish(odom_msg);

      geometry_msgs::msg::TransformStamped T_odom_camera_tf;
      T_odom_camera_tf.header.stamp = tImLeftRos;
      T_odom_camera_tf.header.frame_id = "odom";
      T_odom_camera_tf.child_frame_id = sImLeftFrame + "_vio";
      T_odom_camera_tf.transform = se3ToTF(Toc_current);
      tfBroadcaster_.sendTransform(T_odom_camera_tf);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}
