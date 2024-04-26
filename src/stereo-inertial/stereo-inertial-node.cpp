#include "stereo-inertial-node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM,
                                       const string &strSettingsFile)
    : Node("ORB_SLAM3_ROS2"), SLAM_(SLAM), tfBroadcaster_(this),
      staticBroadcaster_(this) {
  // get T_baselink_camera.
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

  leftCameraInfo_.k.at(0) = fsSettings["Camera1.fx"];
  leftCameraInfo_.k.at(2) = fsSettings["Camera1.cx"];
  leftCameraInfo_.k.at(4) = fsSettings["Camera1.fy"];
  leftCameraInfo_.k.at(5) = fsSettings["Camera1.cy"];
  leftCameraInfo_.k.at(8) = 1;

  leftCameraInfo_.d.resize(4);
  leftCameraInfo_.d.at(0) = fsSettings["Camera1.k1"];
  leftCameraInfo_.d.at(1) = fsSettings["Camera1.k2"];
  leftCameraInfo_.d.at(2) = fsSettings["Camera1.p1"];
  leftCameraInfo_.d.at(3) = fsSettings["Camera1.p2"];

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
  pubFrame_ = this->create_publisher<aria_msg::msg::Frame>("frame", 1000);

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

      SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

      Eigen::Matrix3f R_color_color_optical;
      R_color_color_optical << 0, 0, 1, -1, 0, 0, 0, -1, 0;

      // only publish when the system is "OK"
      auto status = SLAM_->GetTrackingState();
      if (status != ORB_SLAM3::Tracking::eTrackingState::OK)
        continue;

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

      auto Toc_optical = optimized_trajectory_optical.rbegin()->second;

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

      aria_msg::msg::Frame frame_msg;

      frame_msg.header.stamp = tImLeftRos;
      frame_msg.header.frame_id = sImLeftFrame;

      auto bow_vec = SLAM_->GetCurrentFrameBowVec();
      for (auto [word_id, word_weight] : bow_vec) {
        frame_msg.global_descriptor.ids.push_back(word_id);
        frame_msg.global_descriptor.values.push_back(word_weight);
      }

      cv_bridge::CvImage cv_image;
      cv_image.image = imLeft;
      cv_image.encoding = sensor_msgs::image_encodings::MONO8;
      cv_image.toImageMsg(frame_msg.left_image);

      frame_msg.landmarks.header.stamp = tImLeftRos;
      frame_msg.landmarks.header.frame_id = "odom";
      frame_msg.landmarks.height = 1;

      sensor_msgs::PointCloud2Modifier modifier(frame_msg.landmarks);
      // clang-format off
      modifier.setPointCloud2Fields(11, // no format
                                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "id_low", 1, sensor_msgs::msg::PointField::UINT32,
                                "id_high", 1, sensor_msgs::msg::PointField::UINT32,
                                "keypoint_id", 1, sensor_msgs::msg::PointField::UINT32,
                                "max_distance", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "min_distance", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "norm_x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "norm_y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "norm_z", 1, sensor_msgs::msg::PointField::FLOAT32
                               );
      // clang-format on

      // publish features
      auto const &landmarks = SLAM_->GetTrackedMapPoints();
      auto const &keypoints = SLAM_->GetTrackedKeyPointsUn();
      cv::Mat local_descriptors = SLAM_->GetDescriptors();
      if (local_descriptors.empty()) {
        std::cout << "empty descriptors! skipping frame" << std::endl;
        continue;
      }

      modifier.resize(landmarks.size());
      sensor_msgs::PointCloud2Iterator<float> x_it(frame_msg.landmarks, "x");
      sensor_msgs::PointCloud2Iterator<float> y_it(frame_msg.landmarks, "y");
      sensor_msgs::PointCloud2Iterator<float> z_it(frame_msg.landmarks, "z");
      sensor_msgs::PointCloud2Iterator<uint32_t> id_low_it(frame_msg.landmarks,
                                                           "id_low");
      sensor_msgs::PointCloud2Iterator<uint32_t> id_high_it(frame_msg.landmarks,
                                                            "id_high");
      sensor_msgs::PointCloud2Iterator<uint32_t> keypoint_id_it(
          frame_msg.landmarks, "keypoint_id");
      sensor_msgs::PointCloud2Iterator<float> max_distance_it(
          frame_msg.landmarks, "max_distance");
      sensor_msgs::PointCloud2Iterator<float> min_distance_it(
          frame_msg.landmarks, "min_distance");
      sensor_msgs::PointCloud2Iterator<float> norm_x_it(frame_msg.landmarks,
                                                        "norm_x");
      sensor_msgs::PointCloud2Iterator<float> norm_y_it(frame_msg.landmarks,
                                                        "norm_y");
      sensor_msgs::PointCloud2Iterator<float> norm_z_it(frame_msg.landmarks,
                                                        "norm_z");

      std::set<long unsigned int> landmark_ids;

      // descriptor as image
      size_t nGoodLandmarks = 0;

      auto id_split = [](uint64_t id) {
        return std::make_pair(static_cast<uint32_t>(id),
                              static_cast<uint32_t>(id >> 32));
      };

      assert(landmarks.size() == keypoints.size());

      cv::Mat landmark_descriptors(landmarks.size(), local_descriptors.cols,
                                   local_descriptors.type());

      for (uint32_t i = 0; i < landmarks.size(); i++) {
        auto const &landmark = landmarks[i];
        if (landmark == nullptr or landmark->isBad() or
            landmark_ids.count(landmark->mnId))
          continue;
        landmark_ids.insert(landmark->mnId);
        auto T_o_landmark = Two.inverse() * landmark->GetWorldPos();
        *x_it = T_o_landmark.x();
        *y_it = T_o_landmark.y();
        *z_it = T_o_landmark.z();

        auto const &[id_low, id_high] = id_split(landmark->mnId);

        *id_low_it = id_low;
        *id_high_it = id_high;
        *keypoint_id_it = i;

        landmark->GetDescriptor().row(i).copyTo(
            landmark_descriptors.row(nGoodLandmarks));

        nGoodLandmarks++;

        *max_distance_it = landmark->mfMaxDistance;
        *min_distance_it = landmark->mfMinDistance;
        *norm_x_it = landmark->GetNormal().x();
        *norm_y_it = landmark->GetNormal().y();
        *norm_z_it = landmark->GetNormal().z();

        ++x_it;
        ++y_it;
        ++z_it;
        ++id_low_it;
        ++id_high_it;
        ++keypoint_id_it;
        ++max_distance_it;
        ++min_distance_it;
        ++norm_x_it;
        ++norm_y_it;
        ++norm_z_it;
      }

      modifier.resize(nGoodLandmarks);

      landmark_descriptors.resize(nGoodLandmarks);
      cv_bridge::CvImage landmark_desc_cv;
      landmark_desc_cv.image = landmark_descriptors;
      landmark_desc_cv.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      landmark_desc_cv.toImageMsg(frame_msg.landmark_local_descriptors);

      frame_msg.keypoints.resize(keypoints.size() * 3);
      for (size_t i = 0; i < keypoints.size(); i++) {
        frame_msg.keypoints[i * 3] = keypoints[i].pt.x;
        frame_msg.keypoints[i * 3 + 1] = keypoints[i].pt.y;
        frame_msg.keypoints[i * 3 + 2] = keypoints[i].octave;
      }

      cv_bridge::CvImage desc_cv;
      desc_cv.image = local_descriptors;
      desc_cv.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      desc_cv.toImageMsg(frame_msg.local_descriptors);
      frame_msg.local_descriptors.header.stamp = tImLeftRos;
      frame_msg.local_descriptors.header.frame_id = sImLeftFrame;

      // save feature vector of current frame
      frame_msg.local_feature_vector.node_ids.clear();
      frame_msg.local_feature_vector.feature_ids.clear();
      for (auto const &[node_id, feature_ids] :
           SLAM_->GetCurrentFrameFeatVec()) {
        frame_msg.local_feature_vector.node_ids.push_back(node_id);
        std_msgs::msg::UInt32MultiArray feature_ids_msg;
        feature_ids_msg.data = feature_ids;
        frame_msg.local_feature_vector.feature_ids.push_back(feature_ids_msg);
      }

      frame_msg.log_scale_factor = SLAM_->GetFrame()->mfLogScaleFactor;
      frame_msg.scale_levels = SLAM_->GetFrame()->mnScaleLevels;
      frame_msg.scale_factors = SLAM_->GetFrame()->mvScaleFactors;

      pubLandmarks_->publish(frame_msg.landmarks);
      pubDesc_->publish(frame_msg.local_descriptors);

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

      path_msg.header.stamp = tImLeftRos;
      path_msg.header.frame_id = "odom";
      pubPath_->publish(path_msg);

      geometry_msgs::msg::TransformStamped T_map_odom_tf;
      T_map_odom_tf.header.stamp = tImLeftRos;
      T_map_odom_tf.header.frame_id = "map";
      T_map_odom_tf.child_frame_id = "odom";
      T_map_odom_tf.transform = se3ToTF(Two);

      tfBroadcaster_.sendTransform(T_map_odom_tf);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = tImLeftRos;
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = sImLeftFrame + "_vio";
      odom_msg.pose.pose.position.x = Toc_current.translation().x();
      odom_msg.pose.pose.position.y = Toc_current.translation().y();
      odom_msg.pose.pose.position.z = Toc_current.translation().z();

      odom_msg.pose.pose.orientation.x = Toc_current.unit_quaternion().x();
      odom_msg.pose.pose.orientation.y = Toc_current.unit_quaternion().y();
      odom_msg.pose.pose.orientation.z = Toc_current.unit_quaternion().z();
      odom_msg.pose.pose.orientation.w = Toc_current.unit_quaternion().w();

      frame_msg.camera_pose.position.x = Toc_optical.translation().x();
      frame_msg.camera_pose.position.y = Toc_optical.translation().y();
      frame_msg.camera_pose.position.z = Toc_optical.translation().z();
      frame_msg.camera_pose.orientation.x = Toc_optical.unit_quaternion().x();
      frame_msg.camera_pose.orientation.y = Toc_optical.unit_quaternion().y();
      frame_msg.camera_pose.orientation.z = Toc_optical.unit_quaternion().z();
      frame_msg.camera_pose.orientation.w = Toc_optical.unit_quaternion().w();

      frame_msg.camera_info = leftCameraInfo_;
      frame_msg.camera_info.header = frame_msg.header;
      frame_msg.camera_info.height = imLeft.rows;
      frame_msg.camera_info.width = imLeft.cols;

      frame_msg.level_sigma2 = SLAM_->GetLevelSigma2();

      pubFrame_->publish(frame_msg);

      pubOdom_->publish(odom_msg);

      geometry_msgs::msg::TransformStamped T_odom_camera_tf;
      T_odom_camera_tf.header.stamp = tImLeftRos;
      T_odom_camera_tf.header.frame_id = "odom";
      T_odom_camera_tf.child_frame_id = sImLeftFrame + "_vio";
      T_odom_camera_tf.transform = se3ToTF(Toc_current);
      tfBroadcaster_.sendTransform(T_odom_camera_tf);

      // publish tf from camera to baselink
      geometry_msgs::msg::TransformStamped T_baselink_camera_tf;
      T_baselink_camera_tf.header.stamp = tImLeftRos;
      T_baselink_camera_tf.header.frame_id = sImLeftFrame + "_vio";
      T_baselink_camera_tf.child_frame_id = "baselink";
      T_baselink_camera_tf.transform = se3ToTF(T_baselink_camera_);
      tfBroadcaster_.sendTransform(T_baselink_camera_tf);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}
