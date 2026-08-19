/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file ball_detector.hpp
 * \author Yusu Pan <xxdsox@gmail.com>
 * \version 2018
 * \date 2018-02-15
 *
 * Modified: depth-based 3D localization replaces IPM field-plane projection.
 * Ball position is now in camera FLU frame (X forward, Y left, Z up) in meters.
 */

#pragma once
#include <ros/ros.h>
#include <deque>
#include <fstream>
#include <functional>
#include <queue>
#include <string>
#include <vector>
#include "dancer_geometry/utils.hpp"
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/kalman.hpp"
#include "dvision/timer.hpp"

#include "yolo/utils.hpp"


using dmsgs::VisionInfo;
namespace dvision {

struct Control;

struct BallPos {
  BallPos(cv::Point2f p, double t) : pos(p), time(t) {}
  cv::Point2f pos;
  double time;
};

// 3D ball position history entry for FLU velocity computation
struct BallPos3D {
  BallPos3D(float x, float y, float z, double t) : flu_x(x), flu_y(y), flu_z(z), time(t) {}
  float flu_x, flu_y, flu_z;
  double time;
};

class BallDetector : public IDetector {
 public:
  explicit BallDetector();
  ~BallDetector() override = default;
  bool Init() override;

  /**
   * \brief Detect ball position using ZED point cloud depth
   *
   * Samples XYZ from the point cloud at the YOLO bbox center,
   * converts from optical (X right, Y down, Z forward) to
   * camera FLU (X forward, Y left, Z up), and populates
   * vision_info.ball_field with the 3D FLU position.
   *
   * \param ball_position - YOLO bounding box of detected ball
   * \param gui_img - GUI image for drawing detections
   * \param sample_flu - callback sampling one pixel into camera FLU
   * \param vision_info - output VisionInfo message
   * \param delta - robot motion delta (for Kalman)
   */
  void Detect(const BBox& ball_position, cv::Mat& gui_img,
              const std::function<bool(int, int, cv::Point3f&)>& sample_flu,
              VisionInfo& vision_info,
              const Control& delta);

  /**
   * \brief Detect ball using a FLU measurement already sampled in step1.
   *
   * This keeps depth sampling on the same frame as YOLO and avoids step2
   * blocking on the ZED camera mutex.
   */
  void DetectPreSampled(const BBox& ball_position, cv::Mat& gui_img,
                        bool has_flu, const cv::Point3f& flu,
                        VisionInfo& vision_info, const Control& delta);

  //! Get ball position in image coordinate
  inline cv::Point ball_image() { return ball_image_; }

  //! Get ball position in camera FLU frame (X forward, Y left, Z up, meters)
  inline cv::Point3f ball_flu() const {
    if (parameters.ball.useKalman) {
      return cv::Point3f(ball_field_kalman_.x, ball_field_kalman_.y, ball_field_z_kalman_);
    }
    return ball_field_;
  }

  //! Get ball velocity in camera FLU frame (m/s)
  inline cv::Point3f ball_velocity_flu() const { return ball_velocity_flu_; }

  //! Whether the current frame has a real YOLO + depth measurement.
  inline bool has_raw_measurement() const { return has_raw_measurement_; }

  //! Raw current-frame ball position in camera FLU before detector Kalman.
  inline cv::Point3f raw_ball_flu() const { return ball_field_; }

 private:
  //! Center point of ball in image plane
  cv::Point ball_image_;
  //! Top-left of ball bbox in image
  cv::Point ball_image_top_;
  //! Bottom-right of ball bbox in image  
  cv::Point ball_image_bottom_;
  //! Ball radius in image plane (pixels)
  float ball_image_radius_;
  //! Ball position in camera FLU frame (X forward, Y left, Z up, meters)
  cv::Point3f ball_field_;
  //! Kalman-filtered ball XY (FLU X, Y only)
  cv::Point2f ball_field_kalman_;
  //! Kalman-filtered ball Z (FLU Z)
  float ball_field_z_kalman_ = 0.0f;
  //! Ball velocity in camera FLU frame
  cv::Point3f ball_velocity_flu_;
  //! True only when the current frame has a valid measured depth point.
  bool has_raw_measurement_;

  //! Kalman filter instance (for XY only)
  ObjectPosKalmanFilter *kalmanI_;

  //! 3D position history for velocity computation
  std::deque<BallPos3D> ball_pos_history_3d_;

  //! Update Kalman filter prediction
  bool Update(const Control& delta);

  /**
   * \brief Process ball detection with point cloud depth
   *
   * \param ball_position - YOLO bbox
   * \param gui_img - debug image
   * \param sample_flu - callback sampling one pixel into camera FLU
   * \param delta - robot motion
   * \return whether ball is seen
   */
  bool Process(const BBox& ball_position, cv::Mat& gui_img,
               const std::function<bool(int, int, cv::Point3f&)>& sample_flu,
               const Control& delta);

  bool ProcessPreSampled(const BBox& ball_position, cv::Mat& gui_img,
                         bool has_flu, const cv::Point3f& flu,
                         const Control& delta);

  /**
   * \brief Sample FLU position from point cloud at bbox center
   *
   * Takes median of multiple samples around bbox center for robustness.
   *
   * \param point_cloud - ZED point cloud optical frame (CV_32FC4)
   * \param cx - bbox center x (pixel)
   * \param cy - bbox center y (pixel)
   * \param flu - output FLU position
   * \return whether sampling is valid
   */
  bool sampleBallFLU(const std::function<bool(int, int, cv::Point3f&)>& sample_flu,
                     int cx, int cy, cv::Point3f& flu);
};
}  // namespace dvision
