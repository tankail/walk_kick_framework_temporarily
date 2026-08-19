#include "dvision/zed_camera.hpp"
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <cmath>
#include <cstdlib>
#include <string>

namespace dvision {

ZedCamera::ZedCamera() {}

ZedCamera::~ZedCamera() {
  std::lock_guard<std::mutex> lk(zed_mutex_);
  zed.close();
}

bool ZedCamera::initialize() {
  std::lock_guard<std::mutex> lk(zed_mutex_);
  sl::InitParameters init_params;
  init_params.camera_resolution = sl::RESOLUTION::VGA;  // 640x480
  init_params.camera_fps = 30;
  const char* depth_mode_env = std::getenv("DVISION_ZED_DEPTH_MODE");
  std::string depth_mode = depth_mode_env ? depth_mode_env : "QUALITY";
  if (depth_mode == "PERFORMANCE" || depth_mode == "performance") {
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    depth_mode = "PERFORMANCE";
  } else {
    init_params.depth_mode = sl::DEPTH_MODE::QUALITY;
    depth_mode = "QUALITY";
  }
  ROS_INFO("[zed_camera] depth_mode=%s", depth_mode.c_str());
  init_params.sdk_verbose = true;
  // RIGHT_HANDED_Z_UP: SDK X=right, Y=forward, Z=up in camera frame
  init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
  // Point cloud / depth in METERS. SDK default is MILLIMETER; the ball
  // localization (sampleBallFLU validity, FK, downstream kick distances)
  // all assume meters, so this MUST be set explicitly.
  init_params.coordinate_units = sl::UNIT::METER;
  // Disable IMU depth stabilization to keep coordinates camera-relative
  init_params.depth_stabilization = 0;

  sl::ERROR_CODE err = zed.open(init_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    ROS_ERROR("ZED camera open failed: %s", sl::toString(err).c_str());
    return false;
  }

  return true;
}

Frame ZedCamera::capture() {
  std::lock_guard<std::mutex> lk(zed_mutex_);
  if (zed.grab() != sl::ERROR_CODE::SUCCESS) {
    ROS_ERROR("ZED grab failed!");
    return Frame();
  }

  // 获取左侧图像 (BGRA)
  zed.retrieveImage(image_, sl::VIEW::LEFT, sl::MEM::CPU, sl::Resolution(0, 0));

  cv::Mat cv_image_bgra = cv::Mat(image_.getHeight(), image_.getWidth(), CV_8UC4, image_.getPtr<sl::uchar1>());
  cv::Mat cv_image_bgr;
  cv::cvtColor(cv_image_bgra, cv_image_bgr, cv::COLOR_BGRA2BGR);

  // 获取点云 (camera frame, depth_stabilization=0 保证相机相对坐标)
  // SDK 坐标系 RIGHT_HANDED_Z_UP: X 右, Y 前, Z 上
  zed.retrieveMeasure(point_cloud_, sl::MEASURE::XYZ, sl::MEM::CPU, sl::Resolution(0, 0));

  return Frame(cv_image_bgr);
}

cv::Mat ZedCamera::getPointCloudOptical() const {
  int h = point_cloud_.getHeight();
  int w = point_cloud_.getWidth();
  if (h <= 0 || w <= 0) {
    return cv::Mat();
  }
  // sl::float4 = {x, y, z, confidence}, 内存布局与 CV_32FC4 一致
  cv::Mat pc(h, w, CV_32FC4, point_cloud_.getPtr<sl::uchar1>(sl::MEM::CPU));
  return pc;
}

bool ZedCamera::sampleFLU(int u, int v, float& flu_x, float& flu_y, float& flu_z) const {
  std::lock_guard<std::mutex> lk(zed_mutex_);
  return sampleFLUFromLatestPointCloud(u, v, flu_x, flu_y, flu_z);
}

bool ZedCamera::sampleCurrentFLU(int u, int v, float& flu_x, float& flu_y,
                                 float& flu_z) const {
  return sampleFLUFromLatestPointCloud(u, v, flu_x, flu_y, flu_z);
}

bool ZedCamera::sampleFLUFromLatestPointCloud(int u, int v, float& flu_x,
                                              float& flu_y, float& flu_z) const {
  int h = point_cloud_.getHeight();
  int w = point_cloud_.getWidth();
  if (u < 0 || u >= w || v < 0 || v >= h) {
    return false;
  }

  sl::float4 pt;
  sl::ERROR_CODE err = point_cloud_.getValue(u, v, &pt, sl::MEM::CPU);
  if (err != sl::ERROR_CODE::SUCCESS) {
    return false;
  }

  if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ||
      std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) {
    return false;
  }
  // SDK RIGHT_HANDED_Z_UP (depth_stabilization=0) camera frame:
  //   SDK X = 相机右, SDK Y = 相机前, SDK Z = 相机上
  // 转换为 FLU (X 前, Y 左, Z 上):
  flu_x =  pt.y;   // SDK Y (前) -> FLU X
  flu_y = -pt.x;   // -SDK X (右) -> FLU Y (左)
  flu_z =  pt.z;   // SDK Z (上) -> FLU Z
  return true;
}

}  // namespace dvision
