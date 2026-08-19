/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 *
 * Modified: depth-based 3D localization using ZED point cloud.
 * Ball position is now in camera FLU frame (X forward, Y left, Z up) in meters.
 */

#include "dvision/ball_detector.hpp"

#include <algorithm>
#include <cmath>
#include "dancer_geometry/utils.hpp"
#include "dconfig/dconstant.hpp"
#include "dvision/parameters.hpp"

using namespace dancer_geometry;

namespace dvision {

static constexpr float kOpticalInvalid = -9999.0f;

BallDetector::BallDetector()
    : ball_image_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_image_top_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_image_bottom_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_image_radius_(3.0),
      ball_field_(kOpticalInvalid, kOpticalInvalid, kOpticalInvalid),
      ball_field_kalman_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_velocity_flu_(0.0f, 0.0f, 0.0f),
      has_raw_measurement_(false) {}

bool BallDetector::Init() {
  if (!parameters.ball.enable) return false;
  kalmanI_ = new ObjectPosKalmanFilter(ball_field_kalman_,
                                       parameters.ball.kalmanMaxMiss);
  ROS_DEBUG("BallDetector Init (depth-based)");
  return true;
}

bool BallDetector::Update(const Control& delta) {
  if (parameters.ball.useKalman) {
    cv::Point2d pos2d(ball_field_.x, ball_field_.y);
    bool see_ball = kalmanI_->Update(pos2d, delta);
    ball_field_kalman_ = kalmanI_->GetResult();
    // Z 不做卡尔曼滤波，直接透传当前帧测量的相机系 z（仅当本帧有效）
    if (ball_field_.x > -9998.0f) {
      ball_field_z_kalman_ = ball_field_.z;
    }
    return see_ball;
  }
  return ball_field_.x > -9998.0f;
}

void BallDetector::Detect(const BBox& ball_position, cv::Mat& gui_img,
                          const std::function<bool(int, int, cv::Point3f&)>& sample_flu,
                          VisionInfo& vision_info,
                          const Control& delta) {
  Timer t;

  vision_info.see_ball = Process(ball_position, gui_img, sample_flu, delta);
  if (vision_info.see_ball) {
    // Store camera FLU 3D position (X forward, Y left, Z up, meters).
    // zed_dvision step2 applies FK to convert these to base_link.
    cv::Point3f flu = ball_flu();
    vision_info.ball_field.x = flu.x;
    vision_info.ball_field.y = flu.y;
    vision_info.ball_field.z = flu.z;
  }

  ROS_DEBUG("ball detect (depth) used %lf ms", t.elapsedMsec());
}

void BallDetector::DetectPreSampled(const BBox& ball_position, cv::Mat& gui_img,
                                    bool has_flu, const cv::Point3f& flu,
                                    VisionInfo& vision_info,
                                    const Control& delta) {
  Timer t;

  vision_info.see_ball =
      ProcessPreSampled(ball_position, gui_img, has_flu, flu, delta);
  if (vision_info.see_ball) {
    cv::Point3f filtered_flu = ball_flu();
    vision_info.ball_field.x = filtered_flu.x;
    vision_info.ball_field.y = filtered_flu.y;
    vision_info.ball_field.z = filtered_flu.z;
  }

  ROS_DEBUG("ball detect (pre-sampled depth) used %lf ms", t.elapsedMsec());
}

bool BallDetector::Process(const BBox& ball_position, cv::Mat& gui_img,
                           const std::function<bool(int, int, cv::Point3f&)>& sample_flu,
                           const Control& delta) {
  has_raw_measurement_ = false;
  if (!parameters.ball.enable) {
    return false;
  }

  // Check sampler is valid
  if (!sample_flu) {
    ball_field_.x = UNKNOWN_OBJ_POS;
    ball_field_.y = UNKNOWN_OBJ_POS;
    ball_field_.z = UNKNOWN_OBJ_POS;
    return false;
  }

  bool see_ball = false;

  if (ball_position.class_label_prob >= 0.3f) {
    ball_image_.x = static_cast<int>((ball_position.left_top.x + ball_position.right_bottom.x) / 2.0f);
    ball_image_.y = static_cast<int>((ball_position.left_top.y + ball_position.right_bottom.y) / 2.0f);
    ball_image_top_ = cv::Point(ball_position.left_top.x, ball_position.left_top.y);
    ball_image_bottom_ = cv::Point(ball_position.right_bottom.x, ball_position.right_bottom.y);
    ball_image_radius_ = static_cast<float>(std::max(
        std::abs(ball_position.right_bottom.x - ball_position.left_top.x),
        std::abs(ball_position.right_bottom.y - ball_position.left_top.y))) * 0.5f;

    // Sample depth from point cloud at bbox center → FLU coords
    cv::Point3f flu;
    if (sampleBallFLU(sample_flu, ball_image_.x, ball_image_.y, flu)) {
      ball_field_ = flu;
      see_ball = true;
      has_raw_measurement_ = true;

      // Bbox size vs depth consistency check
      // At distance d, expected bbox height in pixels ≈ fx * ball_diameter / d
      // ball diameter = 0.22m for size 5 football
      // float expected_radius_px = parameters.camera.fx * 0.11f / flu.x;
      // float ratio = ball_image_radius_ / std::max(expected_radius_px, 1.0f);
      // if (ratio < 0.3f || ratio > 3.0f) see_ball = false;
    }
  }

  if (!see_ball) {
    ball_field_.x = UNKNOWN_OBJ_POS;
    ball_field_.y = UNKNOWN_OBJ_POS;
    ball_field_.z = UNKNOWN_OBJ_POS;
  }

  // Kalman update
  see_ball = Update(delta);

  // Draw debug
  if (parameters.monitor.update_gui_img && parameters.ball.showResult && see_ball) {
    cv::circle(gui_img, ball_image_, static_cast<int>(ball_image_radius_),
               redColor(), 2);
    // Show FLU coords on image
    char buf[128];
    snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f", ball_field_.x, ball_field_.y, ball_field_.z);
    cv::putText(gui_img, buf, ball_image_ + cv::Point(10, 0),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, redColor(), 1);
  }

  return see_ball;
}

bool BallDetector::ProcessPreSampled(const BBox& ball_position, cv::Mat& gui_img,
                                     bool has_flu, const cv::Point3f& flu,
                                     const Control& delta) {
  has_raw_measurement_ = false;
  if (!parameters.ball.enable) {
    return false;
  }

  bool see_ball = false;
  if (ball_position.class_label_prob >= 0.3f && has_flu) {
    ball_image_.x = static_cast<int>(
        (ball_position.left_top.x + ball_position.right_bottom.x) / 2.0f);
    ball_image_.y = static_cast<int>(
        (ball_position.left_top.y + ball_position.right_bottom.y) / 2.0f);
    ball_image_top_ = cv::Point(ball_position.left_top.x, ball_position.left_top.y);
    ball_image_bottom_ =
        cv::Point(ball_position.right_bottom.x, ball_position.right_bottom.y);
    ball_image_radius_ = static_cast<float>(std::max(
        std::abs(ball_position.right_bottom.x - ball_position.left_top.x),
        std::abs(ball_position.right_bottom.y - ball_position.left_top.y))) * 0.5f;

    if (std::isfinite(flu.x) && std::isfinite(flu.y) &&
        std::isfinite(flu.z) && flu.x > 0.05f && flu.x <= 20.0f) {
      ball_field_ = flu;
      see_ball = true;
      has_raw_measurement_ = true;
    }
  }

  if (!see_ball) {
    ball_field_.x = UNKNOWN_OBJ_POS;
    ball_field_.y = UNKNOWN_OBJ_POS;
    ball_field_.z = UNKNOWN_OBJ_POS;
  }

  see_ball = Update(delta);

  if (parameters.monitor.update_gui_img && parameters.ball.showResult && see_ball) {
    cv::circle(gui_img, ball_image_, static_cast<int>(ball_image_radius_),
               redColor(), 2);
    char buf[128];
    snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f", ball_field_.x, ball_field_.y,
             ball_field_.z);
    cv::putText(gui_img, buf, ball_image_ + cv::Point(10, 0),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, redColor(), 1);
  }

  return see_ball;
}

bool BallDetector::sampleBallFLU(
    const std::function<bool(int, int, cv::Point3f&)>& sample_flu, int cx,
    int cy, cv::Point3f& flu) {
  // Sample cross pattern around bbox center for robustness
  int offsets[][2] = {{0,0}, {2,0}, {-2,0}, {0,2}, {0,-2}};
  constexpr int kNumOffsets = 5;

  std::vector<float> flu_x, flu_y, flu_z;
  flu_x.reserve(kNumOffsets);
  flu_y.reserve(kNumOffsets);
  flu_z.reserve(kNumOffsets);

  for (int i = 0; i < kNumOffsets; ++i) {
    int sx = cx + offsets[i][0];
    int sy = cy + offsets[i][1];
    cv::Point3f sample;
    if (!sample_flu(sx, sy, sample)) continue;

    flu_x.push_back(sample.x);
    flu_y.push_back(sample.y);
    flu_z.push_back(sample.z);
  }

  if (flu_x.size() < 2) {
    return false;
  }

  // Sort each axis independently and take median
  std::sort(flu_x.begin(), flu_x.end());
  std::sort(flu_y.begin(), flu_y.end());
  std::sort(flu_z.begin(), flu_z.end());

  size_t n = flu_x.size();
  flu.x = (n % 2 == 0) ? (flu_x[n/2 - 1] + flu_x[n/2]) * 0.5f : flu_x[n/2];
  flu.y = (n % 2 == 0) ? (flu_y[n/2 - 1] + flu_y[n/2]) * 0.5f : flu_y[n/2];
  flu.z = (n % 2 == 0) ? (flu_z[n/2 - 1] + flu_z[n/2]) * 0.5f : flu_z[n/2];

  // Reject obviously bad points
  if (flu.x <= 0.05f || flu.x > 20.0f) {
    return false;
  }

  return true;
}
}  // namespace dvision
