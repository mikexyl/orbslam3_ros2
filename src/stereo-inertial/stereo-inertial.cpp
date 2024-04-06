#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "stereo-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary "
                 "path_to_settings do_rectify [do_equalize]"
              << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  if (argc == 4) {
    argv[4] = "false";
  }

  rclcpp::init(argc, argv);

  // malloc error using new.. try shared ptr
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.

  bool visualization = true;
  std::stringstream ss_vis(argv[5]);
  ss_vis >> std::boolalpha >> visualization;
  ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO,
                          visualization);

  auto node =
      std::make_shared<StereoInertialNode>(&pSLAM, argv[2], argv[3], argv[4]);
  std::cout << "============================" << std::endl;

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
