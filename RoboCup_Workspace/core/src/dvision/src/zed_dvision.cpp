/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 *
 * Modified: ball position now in camera FLU frame (X forward, Y left, Z up)
 * using ZED SDK point cloud depth instead of IPM field-plane projection.
 */

/**
 * \file dvision.cpp
 * \brief Main class of vision module
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-12
 */

#include "dvision/zed_dvision.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

using namespace dancer_geometry;

namespace dvision {
namespace {
bool envFlag(const char *name, bool default_value) {
  const char *value = std::getenv(name);
  if (!value || !*value) return default_value;
  std::string s(value);
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return !(s == "0" || s == "false" || s == "off" || s == "no");
}

int envInt(const char *name, int default_value, int min_value, int max_value) {
  const char *value = std::getenv(name);
  if (!value || !*value) return default_value;
  char *end = nullptr;
  long parsed = std::strtol(value, &end, 10);
  if (end == value) return default_value;
  parsed = std::max<long>(min_value, std::min<long>(max_value, parsed));
  return static_cast<int>(parsed);
}

double envDouble(const char *name, double default_value, double min_value,
                 double max_value) {
  const char *value = std::getenv(name);
  if (!value || !*value) return default_value;
  char *end = nullptr;
  double parsed = std::strtod(value, &end);
  if (end == value || !std::isfinite(parsed)) return default_value;
  return std::max(min_value, std::min(max_value, parsed));
}

std::string envString(const char *name, const std::string &default_value) {
  const char *value = std::getenv(name);
  return (value && *value) ? std::string(value) : default_value;
}

std::string captureDataDir() {
  std::string path = envString("DVISION_CAPTURE_DIR", "/tmp/dvision/hum_data");
  if (!path.empty() && path.back() != '/') path += '/';
  return path;
}

double distance3(const geometry_msgs::Vector3 &a,
                 const geometry_msgs::Vector3 &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool sendAll(int fd, const unsigned char *data, size_t len) {
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
    if (n <= 0) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool sendAll(int fd, const std::string &data) {
  return sendAll(fd, reinterpret_cast<const unsigned char *>(data.data()),
                 data.size());
}

void drawOutlinedText(cv::Mat &img, const std::string &text, cv::Point origin,
                      double scale, const cv::Scalar &color) {
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  cv::putText(img, text, origin, font, scale, cv::Scalar(0, 0, 0), 5,
              cv::LINE_AA);
  cv::putText(img, text, origin, font, scale, color, 2, cv::LINE_AA);
}

std::string fmt3(double value) {
  std::ostringstream ss;
  ss << std::fixed << std::showpos << std::setprecision(3) << value;
  return ss.str();
}

std::string fmt1(double value) {
  std::ostringstream ss;
  ss << std::fixed << std::showpos << std::setprecision(1) << value;
  return ss.str();
}

bool sampleBallFluFromLatestZedFrame(const ZedCamera* camera,
                                     const BBox& ball_position,
                                     cv::Point3f& flu) {
  if (!camera || ball_position.class_label_prob < 0.3f) {
    return false;
  }

  const float left = std::min(ball_position.left_top.x,
                              ball_position.right_bottom.x);
  const float right = std::max(ball_position.left_top.x,
                               ball_position.right_bottom.x);
  const float top = std::min(ball_position.left_top.y,
                             ball_position.right_bottom.y);
  const float bottom = std::max(ball_position.left_top.y,
                                ball_position.right_bottom.y);
  const float width = std::max(1.0f, right - left);
  const float height = std::max(1.0f, bottom - top);
  const int cx = static_cast<int>((left + right) * 0.5f);
  const int cy = static_cast<int>((top + bottom) * 0.5f);
  const int step_x = std::max(2, static_cast<int>(std::round(width / 8.0f)));
  const int step_y = std::max(2, static_cast<int>(std::round(height / 8.0f)));

  std::vector<cv::Point3f> samples;
  samples.reserve(25);
  for (int gy = -2; gy <= 2; ++gy) {
    for (int gx = -2; gx <= 2; ++gx) {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      if (!camera->sampleCurrentFLU(cx + gx * step_x, cy + gy * step_y, x, y, z)) {
        continue;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          x <= 0.05f || x > 20.0f) {
        continue;
      }
      samples.emplace_back(x, y, z);
    }
  }

  if (samples.size() < 2) {
    return false;
  }

  std::vector<float> depths;
  depths.reserve(samples.size());
  for (const auto& sample : samples) {
    depths.push_back(sample.x);
  }
  std::sort(depths.begin(), depths.end());
  const float median_depth = depths[depths.size() / 2];

  std::vector<float> flu_x;
  std::vector<float> flu_y;
  std::vector<float> flu_z;
  flu_x.reserve(samples.size());
  flu_y.reserve(samples.size());
  flu_z.reserve(samples.size());

  const float depth_gate = std::max(0.12f, median_depth * 0.18f);
  for (const auto& sample : samples) {
    if (std::abs(sample.x - median_depth) > depth_gate) {
      continue;
    }
    flu_x.push_back(sample.x);
    flu_y.push_back(sample.y);
    flu_z.push_back(sample.z);
  }

  if (flu_x.size() < 2) {
    return false;
  }

  std::sort(flu_x.begin(), flu_x.end());
  std::sort(flu_y.begin(), flu_y.end());
  std::sort(flu_z.begin(), flu_z.end());

  const size_t n = flu_x.size();
  flu.x = (n % 2 == 0) ? (flu_x[n / 2 - 1] + flu_x[n / 2]) * 0.5f
                       : flu_x[n / 2];
  flu.y = (n % 2 == 0) ? (flu_y[n / 2 - 1] + flu_y[n / 2]) * 0.5f
                       : flu_y[n / 2];
  flu.z = (n % 2 == 0) ? (flu_z[n / 2 - 1] + flu_z[n / 2]) * 0.5f
                       : flu_z[n / 2];

  return std::isfinite(flu.x) && std::isfinite(flu.y) &&
         std::isfinite(flu.z) && flu.x > 0.05f && flu.x <= 20.0f;
}
}  // namespace

DVision::DVision()
    : profiler_step_1_("Step 1"),
      ball_quality_(0),
      profiler_step_2_("Step 2"),
      profiler_others_("Others") {
  // Initialize various member
  ROS_INFO("[dvision] starting");
  nh_ = new ros::NodeHandle("~");

  parameters.init(nh_);
  projection_.init(nh_);
  projection2_.init(nh_);
  amcl_.Init();
  buffer_gui_image_.Init(4, 30);
  buffer_info_.Init(2, 30);
  tracker_.Init(parameters.camera.extrinsic_para, parameters.camera.fx,
                parameters.camera.fy, parameters.camera.undistCx,
                parameters.camera.undistCy, parameters.camera.centerInUndistX,
                parameters.camera.centerInUndistY);
  if (parameters.ball.velocityDebug) {
    log_file_.open("/home/daiz/dancer-debug/ball.txt");
    log_file_ << "time, flu_x, flu_y, flu_z, vx, vy, vz\n";
  }
  if (!parameters.simulation) {
    obstacle_.Init();
    ball_.Init();
    circle_.Init();
    corner_.Init();
    goal_.Init();
    line_classifier_.Init();
    obj_.Init();

    ROS_INFO("detect init success");
  }

  if (parameters.offline_record) {
    image_transport::ImageTransport pub(*nh_);
    pub_image_ = pub.advertise("cam_image", 1, this);
    ROS_WARN("[DVision] Recording mode");
  } else if (parameters.offline_replay) {
    image_transport::ImageTransport sub(*nh_);
    sub_image_ = sub.subscribe(
        "/dvision_" + std::to_string(parameters.robotId) + "/cam_image", 1,
        &DVision::imageCallback, this);
    ROS_WARN("[DVision] Replaying mode");
  }

  ROS_INFO("[dvision] undist %lf %lf", parameters.camera.centerInUndistX,
           parameters.camera.centerInUndistY);

  InitVisionYaw();

  ball_base_filter_max_missing_sec_ =
      envDouble("DVISION_BALL_KF_MAX_MISSING_SEC", 0.45, 0.0, 3.0);
  ball_base_filter_reacquire_reset_sec_ =
      envDouble("DVISION_BALL_KF_REACQUIRE_RESET_SEC", 0.9, 0.0, 5.0);
  ball_base_filter_gate_base_m_ =
      envDouble("DVISION_BALL_KF_GATE_BASE_M", 0.45, 0.05, 5.0);
  ball_base_filter_gate_speed_mps_ =
      envDouble("DVISION_BALL_KF_GATE_SPEED_MPS", 3.0, 0.0, 20.0);
  ball_base_filter_process_noise_ =
      envDouble("DVISION_BALL_KF_PROCESS_NOISE", 0.08, 1e-5, 10.0);
  ball_base_filter_measurement_noise_ =
      envDouble("DVISION_BALL_KF_MEASUREMENT_NOISE", 0.015, 1e-5, 10.0);
  ball_base_filter_max_dt_ =
      envDouble("DVISION_BALL_KF_MAX_DT", 0.12, 0.01, 1.0);
  ROS_INFO("[dvision] ball base_link KF: max_missing=%.2fs gate=%.2fm+%.2fm/s "
           "q=%.4f r=%.4f",
           ball_base_filter_max_missing_sec_, ball_base_filter_gate_base_m_,
           ball_base_filter_gate_speed_mps_, ball_base_filter_process_noise_,
           ball_base_filter_measurement_noise_);

  mjpeg_enabled_ = envFlag("DVISION_MJPEG_ENABLE", true);
  mjpeg_bind_address_ = envString("DVISION_MJPEG_BIND", "0.0.0.0");
  mjpeg_port_ = envInt("DVISION_MJPEG_PORT", 8080, 1, 65535);
  mjpeg_quality_ = envInt("DVISION_MJPEG_QUALITY", 85, 10, 100);
  mjpeg_fps_ = envInt("DVISION_MJPEG_FPS", 8, 0, 60);
  if (mjpeg_enabled_) {
    ROS_INFO("[dvision] MJPEG frame encoding limited to %d FPS", mjpeg_fps_);
    StartMjpegServer();
  }

  // Initialize receiver and transmitter
  sub_motion_info_ = nh_->subscribe(
      "/dmotion_" + std::to_string(parameters.robotId) + "/MotionInfo", 1,
      &DVision::MotionCallback, this);
  sub_behavior_info_ = nh_->subscribe(
      "/dbehavior_" + std::to_string(parameters.robotId) + "/BehaviorInfo", 1,
      &DVision::BehaviorCallback, this);
  sub_reload_config_ = nh_->subscribe("/humanoid/ReloadVisionConfig", 1,
                                      &DVision::ReloadConfigCallback, this);
  pub_ = nh_->advertise<VisionInfo>("VisionInfo", 1);
  serv_toggle_amcl_ =
      nh_->advertiseService("toggle_amcl", &DVision::toggleAMCL, this);
  serv_reset_particle_touch_line_ = nh_->advertiseService(
      "reset_particles_touch_line", &DVision::ResetParticlesTouchLine, this);
  serv_reset_particle_left_touch_ = nh_->advertiseService(
      "reset_particles_left_touch", &DVision::ResetParticlesLeftTouch, this);
  serv_reset_article_right_touch_ = nh_->advertiseService(
      "reset_particles_right_touch", &DVision::ResetParticlesRightTouch, this);
  serv_reset_particle_point_ = nh_->advertiseService(
      "reset_particles_point", &DVision::ResetParticlesPoint, this);
#ifdef USE_NEO_INFO
  serv_reset_yaw_ =
      nh_->advertiseService("reset_yaw", &DVision::SetInitOrientation, this);
#endif

  if (parameters.simulation) {
    ROS_WARN("[dvision] now in SIMULATION mode");
  } else {
    if (parameters.vision_only_mode) {
      ROS_WARN("[dvision] now in VISION_ONLY mode");
    } else {
      ROS_WARN("[dvision] now in REALWORLD mode");
    }
    if (!parameters.offline_replay) {
        camera_ = new ZedCamera();
        if (!camera_->initialize()) {
            ROS_FATAL("ZED camera init failed!");
            return;
        }
    }

    if (parameters.broadcast_vision_info) {
      transmitter_ = new dtransmit::DTransmit();
      transmitter_->startService();
    }
  }

  // Setup concurrent
  if (!parameters.simulation) {
    concurrent_.push([&]() {
#ifdef USE_GST_CAMERA
      obj_.Detect(frame_.bgr_cuda(), workspace_->guiImg, ball_quality_);
#else
      obj_.Detect(frame_.bgr(), workspace_->guiImg, ball_quality_);
#endif
      workspace_->ballPosition = obj_.ball_position();
      workspace_->goalPosition = obj_.goal_position();
      workspace_->robotPosition = obj_.obstacle_position();
      workspace_->fieldFeatures = obj_.field_features();
      workspace_->hasBallFlu =
          sampleBallFluFromLatestZedFrame(camera_, workspace_->ballPosition,
                                          workspace_->ballFlu);
    });
  } else {
    concurrent_.push([&]() {
      workspace_->visionInfo = m_sim_vision_info;
      workspace_->visionInfo.behaviorInfo = m_behavior_info;
    });
  }
}

DVision::~DVision() {
    StopMjpegServer();
    std::cout << profiler_step_1_ << std::endl;
    std::cout << profiler_step_2_ << std::endl;
    std::cout << profiler_others_ << std::endl;

    if (camera_) {
        delete camera_;
        camera_ = nullptr;
    }
    if (transmitter_) {
        delete transmitter_;
        transmitter_ = nullptr;
    }
}


void DVision::Start() {
  t1_ = std::thread([&]() {
    ros::Rate r(30);
    while (ros::ok()) {
      ros::spinOnce();
      step1();
      r.sleep();
    }
  });

  t2_ = std::thread([&]() {
    ros::Rate r(30);
    while (ros::ok()) {
      step2();
      r.sleep();
    }
  });
}

void DVision::Join() {
  t1_.join();
  t2_.join();
}

void DVision::step1() {
  profiler_step_1_.StartTimer();
  CheckMotionConnection();
  workspace_ = &buffer_info_.WorkerRequest();
  profiler_step_1_.LogAndRestartTimer("[1] request buffer");

  if (!parameters.simulation &&
      (!parameters.offline_replay || parameters.offline_record)) {
      frame_ = camera_->capture();
      if(frame_.bgr().empty()){
        profiler_step_1_.LogAndRestartTimer("[1] capture frame (FAILED)");
        buffer_info_.WorkerRelease();
        return;
      }
      cv::Mat frame_bgr = frame_.bgr();
      if (frame_bgr.channels() == 4) {
          cv::cvtColor(frame_bgr, frame_bgr, cv::COLOR_BGRA2BGR);
      }
      frame_ = Frame(frame_bgr);
      profiler_step_1_.LogAndRestartTimer("[1] capture frame");

  }

  if (!parameters.offline_record && parameters.offline_replay) {
    workspace_->visionInfo = VisionInfo();
    workspace_->guiImg = frame_.bgr();
    workspace_->hsvImg = frame_.hsv();
    workspace_->pitch = plat_pitch_;
    workspace_->yaw = plat_yaw_;
    projection_.updateExtrinsic(plat_pitch_, plat_yaw_);
    profiler_step_1_.LogAndRestartTimer("[1] update extrinsic");

    if (!workspace_->guiImg.empty()) {
      concurrent_.spinOnce();
      concurrent_.join();
      profiler_step_1_.LogAndRestartTimer(
          "[1] detect field & objs & obstacles");
    }
  } else if (parameters.vision_only_mode or
             (motion_connected_ && motion_stable_)) {
    workspace_->pitch = plat_pitch_;
    workspace_->yaw = plat_yaw_;
    projection_.updateExtrinsic(plat_pitch_, plat_yaw_);
    profiler_step_1_.LogAndRestartTimer("[1] update extrinsic");

    if (!parameters.simulation) {
      workspace_->visionInfo = VisionInfo();
      workspace_->guiImg = frame_.bgr_raw();
      workspace_->hsvImg = frame_.hsv();
      profiler_step_1_.LogAndRestartTimer("[1] convert frame");
      if (parameters.offline_record) {
        std_msgs::Header header;
        cv_bridge::CvImage cvmsg(header, "bgr8", frame_.bgr().clone());
        sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();
        pub_image_.publish(msg);
        profiler_step_1_.LogAndRestartTimer("[1] publish frame");
      }
    }

    concurrent_.spinOnce();
    concurrent_.join();
    profiler_step_1_.LogAndRestartTimer("[1] detect field & objs & obstacles");
  }
  buffer_info_.WorkerRelease();
  profiler_step_1_.LogAndRestartTimer("[1] release buffer");
}

void DVision::step2() {
  profiler_step_2_.StartTimer();
  auto &visionSharedInfo = buffer_info_.UserRequest();
  profiler_step_2_.LogAndRestartTimer("[2] request buffer");

  ROS_DEBUG("[dvision] motion_connected_: %d motionStable: %d, pitch: %lf",
            (int)motion_connected_, (int)motion_stable_,
            visionSharedInfo.pitch);

  if (parameters.vision_only_mode or
      (motion_connected_ && motion_stable_)) {
    projection2_.updateExtrinsic(visionSharedInfo.pitch, visionSharedInfo.yaw);
    profiler_step_2_.LogAndRestartTimer("[2] update extrinsic");

    Measurement::LandMark mark;
    measurement_ = Measurement();

    if (!parameters.simulation) {
        cv::Mat gui_img = visionSharedInfo.guiImg;
        if (gui_img.channels() == 4) {
            cv::cvtColor(gui_img, gui_img, cv::COLOR_BGRA2BGR);
        }

        goal_.Detect(visionSharedInfo.goalPosition, projection2_,
                    visionSharedInfo.visionInfo, delta_, measurement_);
        profiler_step_2_.LogAndRestartTimer("[2] detect goal");

        corner_.Detect(visionSharedInfo.fieldFeatures, projection2_,
                    visionSharedInfo.visionInfo, gui_img,
                    measurement_);
        profiler_step_2_.LogAndRestartTimer("[2] detect corner");

        circle_.Detect(corner_.getXIntx(), projection2_,
                    visionSharedInfo.visionInfo, delta_, measurement_);
        profiler_step_2_.LogAndRestartTimer("[2] detect circle");

        // ---- Ball detection: depth-based 3D localization ----
        // Step1 pre-samples the ZED point cloud from the same frame as YOLO.
        // Step2 consumes that value without touching the camera, avoiding
        // zed_mutex waits inside the ROS publishing path.
        ball_.DetectPreSampled(visionSharedInfo.ballPosition, gui_img,
                               visionSharedInfo.hasBallFlu,
                               visionSharedInfo.ballFlu,
                               visionSharedInfo.visionInfo, delta_);
        profiler_step_2_.LogAndRestartTimer("[2] detect ball (pre-sampled)");

        bool ball_has_measurement = ball_.has_raw_measurement();

        // ---- FK: camera FLU -> base_link ----
        if (ball_has_measurement) {
            auto &vf = visionSharedInfo.visionInfo;
            // Camera FLU position (from BallDetector)
            const cv::Point3f raw_flu = ball_.raw_ball_flu();
            double cam_fx = raw_flu.x;
            double cam_fy = raw_flu.y;
            double cam_fz = raw_flu.z;

            // Save camera FLU in ball_global for debug / new consumers
            vf.ball_global.x = cam_fx;
            vf.ball_global.y = cam_fy;
            vf.ball_global.z = cam_fz;

            // Head angles from MotionInfo (degrees -> radians)
            double head_yaw = Degree2Radian(visionSharedInfo.yaw);
            double head_pitch = Degree2Radian(visionSharedInfo.pitch);

            double cy = cos(head_yaw), sy = sin(head_yaw);
            double cp = cos(head_pitch), sp = sin(head_pitch);

            // Rotation: R_base_head_pitch = R_z(yaw) @ R_y(pitch)
            double R00 = cy*cp,  R01 = -sy,  R02 = cy*sp;
            double R10 = sy*cp,  R11 =  cy,  R12 = sy*sp;
            double R20 = -sp,    R21 =   0,  R22 = cp;

            // Camera FLU -> left_eye_optical: X_r=-Y_l, Y_d=-Z_u, Z_f=X_f
            double opt_x = -cam_fy;
            double opt_y = -cam_fz;
            double opt_z =  cam_fx;

            // R_base_left_eye_optical = R_base_head_pitch @ R_cam_optical
            // R_cam_optical = [[0,0,1],[-1,0,0],[0,-1,0]]
            double Ro00 =  sy,       Ro01 = -cy*sp,  Ro02 =  cy*cp;
            double Ro10 = -cy,       Ro11 = -sy*sp,  Ro12 =  sy*cp;
            double Ro20 =  0.0,      Ro21 = -cp,     Ro22 = -sp;

            // Translation: position of left_eye in base_link
            // Chain: base->head_yaw(0.014,-0.000267,0.1981)
            //        ->head_pitch(0,0,0.0275)
            //        ->camera_link(0.048925,0.005352,0.058472)
            //        ->left_eye(0, 0.0315, 0)  // ZED Mini baseline/2
            double tx = 0.014, ty = -0.000267, tz = 0.1981 + 0.0275;  // base->head_pitch
            double cam_lx = 0.048925, cam_ly = 0.005352 + 0.0315, cam_lz = 0.058472;
            tx += R00*cam_lx + R01*cam_ly + R02*cam_lz;
            ty += R10*cam_lx + R11*cam_ly + R12*cam_lz;
            tz += R20*cam_lx + R21*cam_ly + R22*cam_lz;

            // Transform optical point to base_link
            vf.ball_field.x = tx + Ro00*opt_x + Ro01*opt_y + Ro02*opt_z;
            vf.ball_field.y = ty + Ro10*opt_x + Ro11*opt_y + Ro12*opt_z;
            vf.ball_field.z = tz + Ro20*opt_x + Ro21*opt_y + Ro22*opt_z;

            // Velocity is computed from position history below (now in base_link).
        }
        if (FilterBallBaseLink(visionSharedInfo.visionInfo,
                               ball_has_measurement, ros::Time::now())) {
            profiler_step_2_.LogAndRestartTimer("[2] filter ball base_link");
        }

        ShowDebugImg(visionSharedInfo);
        profiler_step_2_.LogAndRestartTimer("[2] show debug image");
    }

    // Correct vision yaw
    std::unique_lock<std::mutex> lk(lock_vision_yaw_);
    projection2_.UpdateHeadingOffset(vision_yaw_);
    auto delta = delta_;
    auto yaw = vision_yaw_;
    lk.unlock();
    profiler_step_2_.LogAndRestartTimer("[2] correct vision yaw");

    // Localization (AMCL — still works with field features)
    amcl_.Process(measurement_, delta, yaw / M_PI * 180.0,
                  visionSharedInfo.visionInfo, crouching_);
    float field_quality = amcl_.GetQuality(),
          field_consistency = amcl_.GetConsistency();
    visionSharedInfo.visionInfo.ball_quality = ball_quality_;
    visionSharedInfo.visionInfo.field_quality = field_quality;
    visionSharedInfo.visionInfo.field_consistency = field_consistency;
    profiler_step_2_.LogAndRestartTimer("[2] localization");

    UpdateViewRange(visionSharedInfo.visionInfo);
    profiler_step_2_.LogAndRestartTimer("[2] update view range");

    // Track Ball — compute head pitch/yaw from camera FLU (stored in ball_global)
    if (visionSharedInfo.visionInfo.see_ball &&
        !ball_base_last_output_predicted_) {
      float flu_x = visionSharedInfo.visionInfo.ball_global.x;
      float flu_y = visionSharedInfo.visionInfo.ball_global.y;
      float flu_z = visionSharedInfo.visionInfo.ball_global.z;

      // Head yaw: angle to ball in horizontal plane
      float head_yaw_rad = std::atan2(flu_y, std::max(flu_x, 0.01f));
      // Head pitch: angle to ball in vertical plane
      float horizontal_dist = std::sqrt(flu_x * flu_x + flu_y * flu_y);
      float head_pitch_rad = std::atan2(flu_z, std::max(horizontal_dist, 0.01f));

      visionSharedInfo.visionInfo.ballTrack.pitch =
          Radian2Degree(head_pitch_rad);
      visionSharedInfo.visionInfo.ballTrack.yaw =
          Radian2Degree(head_yaw_rad);
      profiler_step_2_.LogAndRestartTimer("[2] track ball (FLU)");
    }

    // Track circle
    if (visionSharedInfo.visionInfo.see_circle) {
      auto circleField = visionSharedInfo.visionInfo.circle_field;
      if (tracker_.Process(circleField.x, circleField.y)) {
        visionSharedInfo.visionInfo.circleTrack.pitch =
            Radian2Degree(tracker_.out_pitch());
        visionSharedInfo.visionInfo.circleTrack.yaw =
            Radian2Degree(tracker_.out_yaw());
      }
      profiler_step_2_.LogAndRestartTimer("[2] track circle");
    }

    // Obstacles global
    for (auto &g : visionSharedInfo.visionInfo.obstacles_field) {
      cv::Point2f obstacle_global =
          getOnGlobalCoordinate(visionSharedInfo.visionInfo.robot_pos, g);
      geometry_msgs::Vector3 tmp_obs;
      tmp_obs.x = obstacle_global.x;
      tmp_obs.y = obstacle_global.y;
      visionSharedInfo.visionInfo.obstacles_global.push_back(tmp_obs);
    }

    // Track Goal
    if (visionSharedInfo.visionInfo.see_goal) {
      cv::Point2f goal_global =
          getOnGlobalCoordinate(visionSharedInfo.visionInfo.robot_pos,
                              visionSharedInfo.visionInfo.goal_field);
      visionSharedInfo.visionInfo.goal_global.x = goal_global.x;
      visionSharedInfo.visionInfo.goal_global.y = goal_global.y;

      cv::Point2f goal_field;
      goal_field.x = visionSharedInfo.visionInfo.goal_field.x;
      goal_field.y = visionSharedInfo.visionInfo.goal_field.y;
      if (tracker_.Process(goal_field.x, goal_field.y)) {
        visionSharedInfo.visionInfo.goalTrack.pitch =
            Radian2Degree(tracker_.out_pitch());
        visionSharedInfo.visionInfo.goalTrack.yaw =
            Radian2Degree(tracker_.out_yaw());
      }
    }
    profiler_step_2_.LogAndRestartTimer("[2] track goal");

    // Update ball velocity — 恒速 Kalman (base_link, m/s), 替换窗口差分。
    // 状态 [x, y, vx, vy], 测量 [x, y] (ball_field)。球速更平滑, 抗视觉抖动。
    if (visionSharedInfo.visionInfo.see_ball) {
      const cv::Point2f meas(visionSharedInfo.visionInfo.ball_field.x,
                             visionSharedInfo.visionInfo.ball_field.y);
      const double now = ros::Time::now().toSec();
      if (!ball_vel_kalman_inited_) {
        ball_vel_kalman_ = cv::KalmanFilter(4, 2, 0, CV_32F);
        cv::setIdentity(ball_vel_kalman_.transitionMatrix);  // dt 项每帧更新
        cv::setIdentity(ball_vel_kalman_.measurementMatrix);  // 2x4: [1,0,0,0;0,1,0,0]
        ball_vel_kalman_.statePre.at<float>(0) = meas.x;
        ball_vel_kalman_.statePre.at<float>(1) = meas.y;
        ball_vel_kalman_.statePre.at<float>(2) = 0.0f;  // vx
        ball_vel_kalman_.statePre.at<float>(3) = 0.0f;  // vy
        cv::setIdentity(ball_vel_kalman_.processNoiseCov, cv::Scalar(1e-2));
        cv::setIdentity(ball_vel_kalman_.measurementNoiseCov, cv::Scalar(5e-2));
        cv::setIdentity(ball_vel_kalman_.errorCovPost, cv::Scalar(1e-1));
        ball_vel_kalman_inited_ = true;
        ball_vel_kalman_last_t_ = now;
        ball_velocity_ = cv::Point2f(0.0f, 0.0f);
      } else {
        double dt = now - ball_vel_kalman_last_t_;
        ball_vel_kalman_last_t_ = now;
        dt = dt > 0.1 ? 0.1 : (dt < 0.001 ? 0.02 : dt);  // 钳位到 [1ms, 100ms]
        ball_vel_kalman_.transitionMatrix.at<float>(0, 2) = static_cast<float>(dt);
        ball_vel_kalman_.transitionMatrix.at<float>(1, 3) = static_cast<float>(dt);
        ball_vel_kalman_.predict();
        cv::Mat measurement(2, 1, CV_32F);
        measurement.at<float>(0) = meas.x;
        measurement.at<float>(1) = meas.y;
        ball_vel_kalman_.correct(measurement);
        ball_velocity_.x = ball_vel_kalman_.statePost.at<float>(2);
        ball_velocity_.y = ball_vel_kalman_.statePost.at<float>(3);
      }
      visionSharedInfo.visionInfo.ball_velocity.x = ball_velocity_.x;
      visionSharedInfo.visionInfo.ball_velocity.y = ball_velocity_.y;
      visionSharedInfo.visionInfo.ball_velocity.z = 0.0f;
    } else {
      ball_pos_history_ = std::queue<BallPos>();
      ball_vel_kalman_inited_ = false;  // 断流重置, 下次见球重新初始化
    }
    profiler_step_2_.LogAndRestartTimer("[2] update ball velocity");

    // Publish
    pub_.publish(visionSharedInfo.visionInfo);
    if (parameters.broadcast_vision_info && !(++cycle2_ % 3)) {
      transmitter_->sendRos(
          dconstant::network::robotBroadcastAddressBase + parameters.robotId,
          visionSharedInfo.visionInfo);
    }
    profiler_step_2_.LogAndRestartTimer("[2] pub vision info");
  }

  buffer_info_.UserRelease();
  profiler_step_2_.LogAndRestartTimer("[2] release buffer");
}

void DVision::InitVisionYaw() {
  plat_pitch_ = 0;
  plat_yaw_ = 0;
  imu_roll_ = 0;
  imu_pitch_ = 0;
  imu_yaw_pre_ = M_PI / 2;
  imu_yaw_cur_ = M_PI / 2;
  imu_yaw_delta_ = 0;
  vision_yaw_ = M_PI / 2;
  vision_x_pre_ = 0;
  vision_y_pre_ = 0;
  vision_x_cur_ = 0;
  vision_y_cur_ = 0;
  vision_x_delta_ = 0;
  vision_y_delta_ = 0;
  projection2_.ClearHeadingOffsetBias();
}

bool DVision::CheckMotionConnection() {
  auto now = ros::Time::now();
  auto elapsedSec = now.toSec() - time_last_motion_recv.toSec();
  auto conn_elapsedSec = now.toSec() - time_last_motion_conn.toSec();
  motion_connected_ =
      ((elapsedSec < 1) && (conn_elapsedSec < 1)) || parameters.simulation;
  return motion_connected_;
}

void DVision::InitBallBaseKalman(const geometry_msgs::Vector3& measurement,
                                 const ros::Time& stamp) {
  ball_base_kalman_ = cv::KalmanFilter(6, 3, 0, CV_32F);
  cv::setIdentity(ball_base_kalman_.transitionMatrix);
  ball_base_kalman_.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
  ball_base_kalman_.measurementMatrix.at<float>(0, 0) = 1.0f;
  ball_base_kalman_.measurementMatrix.at<float>(1, 1) = 1.0f;
  ball_base_kalman_.measurementMatrix.at<float>(2, 2) = 1.0f;
  cv::setIdentity(ball_base_kalman_.processNoiseCov,
                  cv::Scalar(ball_base_filter_process_noise_));
  cv::setIdentity(ball_base_kalman_.measurementNoiseCov,
                  cv::Scalar(ball_base_filter_measurement_noise_));
  cv::setIdentity(ball_base_kalman_.errorCovPost, cv::Scalar(0.05));

  ball_base_kalman_.statePost = cv::Mat::zeros(6, 1, CV_32F);
  ball_base_kalman_.statePost.at<float>(0) = measurement.x;
  ball_base_kalman_.statePost.at<float>(1) = measurement.y;
  ball_base_kalman_.statePost.at<float>(2) = measurement.z;
  ball_base_kalman_.statePre = ball_base_kalman_.statePost.clone();

  ball_base_last_update_time_ = stamp;
  ball_base_last_measurement_time_ = stamp;
  ball_base_kalman_initialized_ = true;
  ball_base_last_output_predicted_ = false;
}

bool DVision::FilterBallBaseLink(VisionInfo& info, bool has_measurement,
                                 const ros::Time& stamp) {
  geometry_msgs::Vector3 measurement = info.ball_field;

  if (has_measurement &&
      (!std::isfinite(measurement.x) || !std::isfinite(measurement.y) ||
       !std::isfinite(measurement.z))) {
    has_measurement = false;
  }

  if (!ball_base_kalman_initialized_) {
    if (!has_measurement) {
      info.see_ball = false;
      ball_base_last_output_predicted_ = false;
      return false;
    }
    InitBallBaseKalman(measurement, stamp);
    info.see_ball = true;
    info.ball_field = measurement;
    return true;
  }

  double dt = (stamp - ball_base_last_update_time_).toSec();
  if (!std::isfinite(dt) || dt <= 0.0) dt = 1.0 / 30.0;
  dt = std::min(dt, ball_base_filter_max_dt_);
  ball_base_last_update_time_ = stamp;

  auto& A = ball_base_kalman_.transitionMatrix;
  cv::setIdentity(A);
  A.at<float>(0, 3) = dt;
  A.at<float>(1, 4) = dt;
  A.at<float>(2, 5) = dt;

  const cv::Mat prediction = ball_base_kalman_.predict();
  geometry_msgs::Vector3 predicted;
  predicted.x = prediction.at<float>(0);
  predicted.y = prediction.at<float>(1);
  predicted.z = prediction.at<float>(2);

  const double missing_sec = (stamp - ball_base_last_measurement_time_).toSec();
  if (has_measurement && missing_sec > ball_base_filter_reacquire_reset_sec_) {
    InitBallBaseKalman(measurement, stamp);
    info.see_ball = true;
    info.ball_field = measurement;
    return true;
  }

  bool accepted_measurement = false;
  if (has_measurement) {
    const double gate =
        ball_base_filter_gate_base_m_ + ball_base_filter_gate_speed_mps_ * dt;
    const double innovation = distance3(measurement, predicted);
    accepted_measurement = innovation <= gate;
    if (!accepted_measurement) {
      ROS_WARN_THROTTLE(1.0,
                        "[dvision] reject ball jump: innovation=%.3fm gate=%.3fm "
                        "meas=(%.3f, %.3f, %.3f) pred=(%.3f, %.3f, %.3f)",
                        innovation, gate, measurement.x, measurement.y,
                        measurement.z, predicted.x, predicted.y, predicted.z);
    }
  }

  if (accepted_measurement) {
    cv::Mat z(3, 1, CV_32F);
    z.at<float>(0) = measurement.x;
    z.at<float>(1) = measurement.y;
    z.at<float>(2) = measurement.z;
    const cv::Mat corrected = ball_base_kalman_.correct(z);
    info.see_ball = true;
    info.ball_field.x = corrected.at<float>(0);
    info.ball_field.y = corrected.at<float>(1);
    info.ball_field.z = corrected.at<float>(2);
    ball_base_last_measurement_time_ = stamp;
    ball_base_last_output_predicted_ = false;
    return true;
  }

  const double hold_sec = (stamp - ball_base_last_measurement_time_).toSec();
  if (hold_sec <= ball_base_filter_max_missing_sec_) {
    info.see_ball = true;
    info.ball_field = predicted;
    ball_base_last_output_predicted_ = true;
    return true;
  }

  info.see_ball = false;
  ball_base_kalman_initialized_ = false;
  ball_base_last_output_predicted_ = false;
  return false;
}

void DVision::MotionCallback(const dmsgs::MotionInfo::ConstPtr &motion_msg) {
  profiler_others_.StartTimer();
  time_last_motion_recv = ros::Time::now();
  motion_info_ = *motion_msg;
  crouching_ = false;
  if (motion_info_.lower_board_connected)
    time_last_motion_conn = ros::Time::now();

  std::lock_guard<std::mutex> lk(lock_vision_yaw_);

  auto now = ros::Time::now();
  auto conn_elapsedSec = now.toSec() - time_last_motion_conn.toSec();
  if (conn_elapsedSec < 1 && !motion_connected_) {
    ROS_FATAL("Motion reconnected, init vision_yaw");
    InitVisionYaw();
    dx_sum_ = 0;
    dy_sum_ = 0;
    dt_sum_ = 0;
    imu_inited_ = false;
  }

  motion_stable_ = motion_info_.stable;
  plat_pitch_ = motion_info_.curPlat.pitch;
  plat_yaw_ = motion_info_.curPlat.yaw;

  imu_roll_ = motion_info_.imuRPY.x;
  imu_pitch_ = motion_info_.imuRPY.y;
  if (!imu_inited_) {
    imu_yaw_pre_ = motion_info_.imuRPY.z;
    imu_yaw_cur_ = motion_info_.imuRPY.z;
    previous_delta_ = motion_info_.odometry;
    imu_inited_ = true;
  } else if (motion_stable_) {
    imu_yaw_pre_ = imu_yaw_cur_;
    imu_yaw_cur_ = motion_info_.imuRPY.z;
    imu_yaw_delta_ = imu_yaw_cur_ - imu_yaw_pre_;
    vision_yaw_ += Degree2Radian(imu_yaw_delta_);
  }

  if (vision_yaw_ > M_PI) {
    vision_yaw_ -= 2 * M_PI;
  } else if (vision_yaw_ <= -M_PI) {
    vision_yaw_ += 2 * M_PI;
  }

  {
    auto tmpDelta = motion_info_.odometry;
    auto dx = tmpDelta.x - previous_delta_.x;
    auto dy = tmpDelta.y - previous_delta_.y;
    auto dt = tmpDelta.z - previous_delta_.z;

    if (!motion_stable_) {
      dx = 0;
      dy = 0;
    }

    previous_delta_ = tmpDelta;
    delta_.dx = dx;
    delta_.dy = dy;
    delta_.dt = dt;
    bool start_kick = false;
    if (ros::Time::now().toSec() - time_start_kick_.toSec() > 10 &&
        motion_info_.status == dmsgs::MotionInfo::KICKING) {
      time_start_kick_ = ros::Time::now();
    }
    if (ros::Time::now().toSec() - time_start_kick_.toSec() < 2)
      start_kick = true;
    crouching_ =
        (motion_info_.status == dmsgs::MotionInfo::STANDBY || start_kick);

    if (parameters.calib.calibOdometer) {
      dx_sum_ += dx;
      dy_sum_ += dy;
      dt_sum_ += dt;
      ROS_INFO("[dvision] dx_sum_: %lf", dx_sum_);
      ROS_INFO("[dvision] dy_sum_: %lf", dy_sum_);
      ROS_INFO("[dvision] dt_sum_: %lf", dt_sum_);
    }
  }

  profiler_others_.LogAndRestartTimer("[others] motion callback");
}

void DVision::BehaviorCallback(
    const dmsgs::BehaviorInfo::ConstPtr &behavior_msg) {
  m_behavior_info = *behavior_msg;

  if (!parameters.simulation && m_behavior_info.save_image && !save_data_) {
    save_data_ = true;
    const std::string path = captureDataDir();
    try {
      const boost::filesystem::path dir(path);
      if (boost::filesystem::create_directories(dir)) {
        ROS_INFO("Directory Created for captured frames: %s", path.c_str());
      }
    } catch (const boost::filesystem::filesystem_error &e) {
      ROS_WARN("Failed to prepare capture directory %s: %s", path.c_str(),
               e.what());
      return;
    }
    auto frame = camera_->capture();
    if(frame.bgr().empty()){
      ROS_WARN("[zed_dvision] capture failed in BehaviorCallback");
      return;
    }
    std::string filename = path + std::to_string(frame.time_stamp().toNSec()) + ".jpg";
    frame.save(filename);
  }

  save_data_ = m_behavior_info.save_image;
}

void DVision::ReloadConfigCallback(const std_msgs::String::ConstPtr &) {
  parameters.update();
}

void DVision::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv::Mat img;
  img = cv_bridge::toCvShare(msg, "bgr8")->image;
  frame_.offlineUpdate(img);
}

void DVision::UpdateViewRange(VisionInfo &info) {
  cv::Point2f upperLeft;
  cv::Point2f upperRight;
  cv::Point2f lowerLeft;
  cv::Point2f lowerRight;

  projection2_.getOnRealCoordinate(cv::Point(0, 0), upperLeft);
  projection2_.getOnRealCoordinate(cv::Point(parameters.camera.width - 1, 0),
                                   upperRight);
  projection2_.getOnRealCoordinate(cv::Point(0, parameters.camera.height - 1),
                                   lowerLeft);
  projection2_.getOnRealCoordinate(
      cv::Point(parameters.camera.width - 1, parameters.camera.height - 1),
      lowerRight);

  auto v1 = upperLeft - lowerLeft;
  auto v2 = lowerRight - lowerLeft;
  auto theta = getAngleBetweenVectors(v1, v2);
  if (theta < 180 && theta > 0) {
    upperLeft *= -100;
  }

  v1 = upperRight - lowerRight;
  v2 = -v2;
  theta = getAngleBetweenVectors(v2, v1);
  if (theta < 180 && theta > 0) {
    upperRight *= -100;
  }

  info.viewRange.resize(4);
  info.viewRange[0].x = upperLeft.x;
  info.viewRange[0].y = upperLeft.y;
  info.viewRange[1].x = upperRight.x;
  info.viewRange[1].y = upperRight.y;
  info.viewRange[2].x = lowerRight.x;
  info.viewRange[2].y = lowerRight.y;
  info.viewRange[3].x = lowerLeft.x;
  info.viewRange[3].y = lowerLeft.y;
}

void DVision::ShowDebugImg(VisionSharedInfo &info) {
  if (parameters.simulation) return;

  cv::Mat annotated;
  if (mjpeg_enabled_) {
    const ros::Time now = ros::Time::now();
    const double min_interval =
        mjpeg_fps_ > 0 ? 1.0 / static_cast<double>(mjpeg_fps_) : 0.0;
    const bool should_update =
        mjpeg_fps_ > 0 &&
        (last_mjpeg_update_time_.isZero() ||
         (now - last_mjpeg_update_time_).toSec() >= min_interval);
    if (should_update) {
      annotated = MakeAnnotatedDebugFrame(info);
      if (!annotated.empty()) {
        UpdateMjpegFrame(annotated);
        last_mjpeg_update_time_ = now;
      }
    }
  }

  if (parameters.monitor.use_cv_show) {
    if (info.hsvImg.cols != 0 && info.hsvImg.rows != 0) {
      if (parameters.monitor.update_gui_img) {
        gui_cnt_++;
        if (gui_cnt_ % 10 == 0) {
          cv::namedWindow("gui", CV_WINDOW_NORMAL);
          cv::imshow("gui", annotated.empty() ? info.guiImg : annotated);
          gui_cnt_ = 0;
        }
      }
      cv::waitKey(1);
    }
  }
}

cv::Mat DVision::MakeAnnotatedDebugFrame(const VisionSharedInfo& info) {
  cv::Mat frame;
  if (!info.guiImg.empty()) {
    frame = info.guiImg.clone();
  } else if (!info.hsvImg.empty()) {
    cv::cvtColor(info.hsvImg, frame, cv::COLOR_HSV2BGR);
  } else {
    return frame;
  }

  if (frame.channels() == 4) {
    cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
  } else if (frame.channels() == 1) {
    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
  }

  const double text_scale = 0.85;
  const cv::Scalar white(255, 255, 255);
  const cv::Scalar green(80, 255, 80);
  const cv::Scalar yellow(0, 255, 255);
  const cv::Scalar red(40, 40, 255);
  int y = 36;

  drawOutlinedText(frame,
                   "head yaw=" + fmt1(info.yaw) + "deg  pitch=" +
                       fmt1(info.pitch) + "deg",
                   cv::Point(14, y), text_scale, white);
  y += 34;

  if (info.visionInfo.see_ball) {
    cv::Point p1(std::min(info.ballPosition.left_top.x,
                          info.ballPosition.right_bottom.x),
                 std::min(info.ballPosition.left_top.y,
                          info.ballPosition.right_bottom.y));
    cv::Point p2(std::max(info.ballPosition.left_top.x,
                          info.ballPosition.right_bottom.x),
                 std::max(info.ballPosition.left_top.y,
                          info.ballPosition.right_bottom.y));
    p1.x = std::max(0, std::min(frame.cols - 1, p1.x));
    p1.y = std::max(0, std::min(frame.rows - 1, p1.y));
    p2.x = std::max(0, std::min(frame.cols - 1, p2.x));
    p2.y = std::max(0, std::min(frame.rows - 1, p2.y));
    if (!ball_base_last_output_predicted_ && p2.x > p1.x && p2.y > p1.y) {
      cv::rectangle(frame, p1, p2, green, 3, cv::LINE_AA);
    }

    const auto &ball = info.visionInfo.ball_field;
    drawOutlinedText(frame,
                     ball_base_last_output_predicted_ ?
                         "BALL base_link KF PREDICT (m)" :
                         "BALL base_link KF (m)",
                     cv::Point(14, y), text_scale,
                     ball_base_last_output_predicted_ ? yellow : green);
    y += 34;
    drawOutlinedText(frame,
                     "x=" + fmt3(ball.x) + "  y=" + fmt3(ball.y) +
                         "  z=" + fmt3(ball.z),
                     cv::Point(14, y), text_scale, yellow);
  } else {
    drawOutlinedText(frame, "BALL: not detected", cv::Point(14, y),
                     text_scale, red);
  }

  return frame;
}

void DVision::UpdateMjpegFrame(const cv::Mat& frame) {
  if (!mjpeg_enabled_ || frame.empty()) return;
  std::vector<uchar> encoded;
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, mjpeg_quality_};
  if (!cv::imencode(".jpg", frame, encoded, params)) return;
  std::lock_guard<std::mutex> lk(mjpeg_mutex_);
  mjpeg_frame_.swap(encoded);
}

void DVision::StartMjpegServer() {
  if (mjpeg_running_.exchange(true)) return;
  mjpeg_thread_ = std::thread(&DVision::MjpegServerLoop, this);
}

void DVision::StopMjpegServer() {
  if (!mjpeg_running_.exchange(false)) return;
  int fd = mjpeg_server_fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
  if (mjpeg_thread_.joinable()) {
    mjpeg_thread_.join();
  }
}

void DVision::MjpegServerLoop() {
  int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    ROS_ERROR("[dvision] MJPEG socket failed: %s", std::strerror(errno));
    mjpeg_running_ = false;
    return;
  }

  int opt = 1;
  ::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(mjpeg_port_));
  if (mjpeg_bind_address_ == "0.0.0.0" || mjpeg_bind_address_ == "*") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, mjpeg_bind_address_.c_str(),
                         &addr.sin_addr) != 1) {
    ROS_WARN("[dvision] invalid DVISION_MJPEG_BIND=%s, using 0.0.0.0",
             mjpeg_bind_address_.c_str());
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  }

  if (::bind(server_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR("[dvision] MJPEG bind %s:%d failed: %s",
              mjpeg_bind_address_.c_str(), mjpeg_port_, std::strerror(errno));
    ::close(server_fd);
    mjpeg_running_ = false;
    return;
  }

  if (::listen(server_fd, 4) < 0) {
    ROS_ERROR("[dvision] MJPEG listen failed: %s", std::strerror(errno));
    ::close(server_fd);
    mjpeg_running_ = false;
    return;
  }

  mjpeg_server_fd_ = server_fd;
  ROS_INFO("[dvision] MJPEG visualization: http://%s:%d/",
           mjpeg_bind_address_.c_str(), mjpeg_port_);

  while (mjpeg_running_) {
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd =
        ::accept(server_fd, reinterpret_cast<sockaddr *>(&client_addr),
                 &client_len);
    if (client_fd < 0) {
      if (mjpeg_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      continue;
    }
    std::thread(&DVision::HandleMjpegClient, this, client_fd).detach();
  }

  int fd = mjpeg_server_fd_.exchange(-1);
  if (fd >= 0) {
    ::close(fd);
  }
}

void DVision::HandleMjpegClient(int client_fd) {
  timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;
  ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  char request[1024];
  ssize_t request_len = ::recv(client_fd, request, sizeof(request) - 1, 0);
  if (request_len < 0) request_len = 0;
  request[request_len] = '\0';
  const bool is_head = std::string(request, request_len).find("HEAD ") == 0;

  const std::string header =
      "HTTP/1.0 200 OK\r\n"
      "Server: dvision\r\n"
      "Cache-Control: no-cache, no-store, must-revalidate\r\n"
      "Pragma: no-cache\r\n"
      "Connection: close\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  if (!sendAll(client_fd, header)) {
    ::close(client_fd);
    return;
  }
  if (is_head) {
    ::close(client_fd);
    return;
  }

  while (mjpeg_running_) {
    std::vector<uchar> frame;
    {
      std::lock_guard<std::mutex> lk(mjpeg_mutex_);
      frame = mjpeg_frame_;
    }

    if (!frame.empty()) {
      std::ostringstream part;
      part << "--frame\r\n"
           << "Content-Type: image/jpeg\r\n"
           << "Content-Length: " << frame.size() << "\r\n\r\n";
      if (!sendAll(client_fd, part.str()) ||
          !sendAll(client_fd, frame.data(), frame.size()) ||
          !sendAll(client_fd, "\r\n")) {
        break;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ::close(client_fd);
}

bool DVision::toggleAMCL(dmsgs::ToggleAMCL::Request &req,
                         dmsgs::ToggleAMCL::Response &res) {
  enable_amcl_ = (bool)req.swtich;
  ROS_DEBUG("[dvision] toggle amcl %s", (bool)req.swtich ? "true" : "false");
  return true;
}

bool DVision::ResetParticlesTouchLine(
    dmsgs::ResetParticleTouchLine::Request &req,
    dmsgs::ResetParticleTouchLine::Response &res) {
  auto &side = req.side;
  ROS_DEBUG("[dvision] reset particles touch line");
  amcl_.ResetParticlesTouchLine(side);
  return true;
}

bool DVision::ResetParticlesLeftTouch(
    dmsgs::ResetParticleLeftTouch::Request &req,
    dmsgs::ResetParticleLeftTouch::Response &res) {
  ROS_DEBUG("[dvision] reset particles left touch");
  amcl_.ResetParticlesLeftTouch();
  return true;
}

bool DVision::ResetParticlesRightTouch(
    dmsgs::ResetParticleRightTouch::Request &req,
    dmsgs::ResetParticleRightTouch::Response &res) {
  ROS_DEBUG("[dvision] reset particles right touch");
  amcl_.ResetParticlesRightTouch();
  return true;
}

bool DVision::ResetParticlesPoint(dmsgs::ResetParticlePoint::Request &req,
                                  dmsgs::ResetParticlePoint::Response &res) {
  auto &point = req.point;
  vision_yaw_ = Degree2Radian(point.z);
  amcl_.ResetParticlesPoint(point);
  ROS_DEBUG("[dvision] reset particles at point %lf %lf %lf", point.x, point.y,
            point.z);
  return true;
}

bool DVision::ResetParticlesRandom(dmsgs::ResetParticleRandom::Request &req,
                                   dmsgs::ResetParticleRandom::Response &res) {
  amcl_.ResetParticlesRandom();
  ROS_DEBUG("[dvision] reset particles randomly");
  return true;
}

#ifdef USE_NEO_INFO
bool DVision::SetInitOrientation(dmsgs::SetInitOrientation::Request &req,
                                 dmsgs::SetInitOrientation::Response &res) {
  vision_yaw_ = Degree2Radian(req.yaw);
  amcl_.SetYaw(req.yaw);
  ROS_DEBUG("[dvision] reset init orientation");
  return true;
}
#endif

}  // namespace dvision
