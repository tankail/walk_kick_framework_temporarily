#pragma once
#ifndef YOLOV26_H
#define YOLOV26_H

#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.hpp"

namespace dvision {

/**
 * YOLO26 单类(球) TensorRT engine 推理封装。
 *
 * 与旧的 tensorrtx yolov5(自定义 YoloLayer 插件) 不同:
 *  - engine 由 ultralytics YOLO26 导出 ONNX -> trtexec 构建(无自定义插件)
 *  - 输出是 raw 格式 [1, 5, 8400] = (cx, cy, w, h, class_score), 640 尺度
 *  - class_score 已经过 sigmoid, 在 [0,1]
 *  - 本类负责: letterbox 预处理 -> 推理 -> 解码 -> NMS -> 映射回原图坐标
 *  - 对外接口与旧 yolov5 相同, 使 object_detector.cpp 的 class 0..6 分流
 *    逻辑无需改动 (单类模型只产生 class 0 = 球)
 */
class yolov26 {
 public:
  static constexpr int INPUT_H = 640;   // ultralytics imgsz
  static constexpr int INPUT_W = 640;
  static constexpr int NUM_CLASS = 1;   // 单类球
  static constexpr int NUM_OUT = 5;     // cx cy w h score
  static constexpr int NUM_ANCHORS = 8400;  // 80^2+40^2+20^2

  yolov26() = default;
  ~yolov26();

  void init(std::string engine_name, float threshold_, float nms_threshold_,
            int batch_size_);
  std::vector<std::vector<Yolo::Detection>> detect(std::vector<cv::Mat> images);

 private:
  static float iou(const Yolo::Detection &a, const Yolo::Detection &b);
  void nms_class_agnostic(std::vector<Yolo::Detection> &boxes) const;

  IRuntime *runtime_ = nullptr;
  ICudaEngine *engine_ = nullptr;
  IExecutionContext *context_ = nullptr;
  void *buffers_[2] = {nullptr, nullptr};
  int input_index_ = 0;
  int output_index_ = 1;
  float *host_input_ = nullptr;   // [1*3*640*640]
  float *host_output_ = nullptr;  // [1*5*8400]
  cudaStream_t stream_ = nullptr;
  float threshold_ = 0.3f;
  float nms_threshold_ = 0.5f;
  int batch_size_ = 1;
  static Logger gLogger_;
};

}  // namespace dvision

#endif  // YOLOV26_H
