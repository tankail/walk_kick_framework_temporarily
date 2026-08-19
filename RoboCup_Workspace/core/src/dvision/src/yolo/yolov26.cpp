#include "dvision/yolo/yolov26.hpp"

namespace dvision {

Logger yolov26::gLogger_;

yolov26::~yolov26() {
  if (context_) context_->destroy();
  if (engine_) engine_->destroy();
  if (runtime_) runtime_->destroy();
  if (stream_) cudaStreamDestroy(stream_);
  if (buffers_[0]) cudaFree(buffers_[0]);
  if (buffers_[1]) cudaFree(buffers_[1]);
  delete[] host_input_;
  delete[] host_output_;
}

void yolov26::init(std::string engine_name, float threshold_, float nms_threshold_,
                   int batch_size_) {
  this->threshold_ = threshold_;
  this->nms_threshold_ = nms_threshold_;
  this->batch_size_ = 1;  // 固定 batch 1 (engine 由 trtexec 以 [1,3,640,640] 构建)

  cudaSetDevice(0);
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good()) {
    ROS_ERROR("yolov26 read %s error!", engine_name.c_str());
    return;
  }
  file.seekg(0, std::ios::end);
  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);
  char *trtModelStream = new char[size];
  file.read(trtModelStream, size);
  file.close();

  runtime_ = createInferRuntime(gLogger_);
  engine_ = runtime_->deserializeCudaEngine(trtModelStream, size);
  delete[] trtModelStream;
  if (!engine_) {
    ROS_ERROR("yolov26 deserializeCudaEngine failed!");
    return;
  }
  context_ = engine_->createExecutionContext();
  if (!context_) {
    ROS_ERROR("yolov26 createExecutionContext failed!");
    return;
  }

  // 绑定名来自 ultralytics ONNX: input="images", output="output0"
  int inputIndex = engine_->getBindingIndex("images");
  if (inputIndex < 0) inputIndex = 0;
  int outputIndex = engine_->getBindingIndex("output0");
  if (outputIndex < 0) outputIndex = 1;

  input_index_ = inputIndex;
  output_index_ = outputIndex;
  CUDA_CHECK(cudaMalloc(&buffers_[input_index_], 1 * 3 * INPUT_H * INPUT_W * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&buffers_[output_index_], 1 * NUM_OUT * NUM_ANCHORS * sizeof(float)));
  CUDA_CHECK(cudaStreamCreate(&stream_));
  host_input_ = new float[1 * 3 * INPUT_H * INPUT_W];
  host_output_ = new float[1 * NUM_OUT * NUM_ANCHORS];

  ROS_INFO("yolov26 engine loaded: %s (batch=1, %dx%d, in=%s out=%s)",
           engine_name.c_str(), INPUT_W, INPUT_H,
           engine_->getBindingName(inputIndex), engine_->getBindingName(outputIndex));
  ROS_INFO("yolov26 buffers: in=%p(%d) out=%p(%d)",
           buffers_[input_index_], 1 * 3 * INPUT_H * INPUT_W * sizeof(float),
           buffers_[output_index_], 1 * NUM_OUT * NUM_ANCHORS * sizeof(float));
}

float yolov26::iou(const Yolo::Detection &a, const Yolo::Detection &b) {
  // bbox = [cx, cy, w, h]
  float ax1 = a.bbox[0] - a.bbox[2] / 2, ay1 = a.bbox[1] - a.bbox[3] / 2;
  float ax2 = a.bbox[0] + a.bbox[2] / 2, ay2 = a.bbox[1] + a.bbox[3] / 2;
  float bx1 = b.bbox[0] - b.bbox[2] / 2, by1 = b.bbox[1] - b.bbox[3] / 2;
  float bx2 = b.bbox[0] + b.bbox[2] / 2, by2 = b.bbox[1] + b.bbox[3] / 2;
  float x1 = std::max(ax1, bx1), y1 = std::max(ay1, by1);
  float x2 = std::min(ax2, bx2), y2 = std::min(ay2, by2);
  float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  float area_a = a.bbox[2] * a.bbox[3];
  float area_b = b.bbox[2] * b.bbox[3];
  return inter / (area_a + area_b - inter + 1e-6f);
}

void yolov26::nms_class_agnostic(std::vector<Yolo::Detection> &boxes) const {
  // 按置信度降序
  std::sort(boxes.begin(), boxes.end(),
            [](const Yolo::Detection &x, const Yolo::Detection &y) {
              return x.conf > y.conf;
            });
  std::vector<bool> removed(boxes.size(), false);
  std::vector<Yolo::Detection> kept;
  for (size_t i = 0; i < boxes.size(); ++i) {
    if (removed[i]) continue;
    kept.push_back(boxes[i]);
    for (size_t j = i + 1; j < boxes.size(); ++j) {
      if (!removed[j] && iou(boxes[i], boxes[j]) > nms_threshold_) {
        removed[j] = true;
      }
    }
  }
  boxes = std::move(kept);
}

std::vector<std::vector<Yolo::Detection>> yolov26::detect(std::vector<cv::Mat> images) {
  std::vector<std::vector<Yolo::Detection>> batch_res;
  if (images.empty()) return batch_res;
  cudaSetDevice(0);

  // 只处理第 0 张 (batch=1)
  cv::Mat &img = images[0];
  cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H);  // 640x640 letterbox BGR

  // letterbox 参数 (与 preprocess_img 一致), 用于把 640 坐标映射回原图
  float r_w = INPUT_W / (img.cols * 1.0f);
  float r_h = INPUT_H / (img.rows * 1.0f);
  float scale, off_x, off_y;
  if (r_h > r_w) {
    scale = r_w;
    off_x = 0;
    off_y = (INPUT_H - r_w * img.rows) / 2;
  } else {
    scale = r_h;
    off_x = (INPUT_W - r_h * img.cols) / 2;
    off_y = 0;
  }

  // BGR -> NCHW float RGB /255
  int i = 0;
  for (int row = 0; row < INPUT_H; ++row) {
    uchar *uc = pr_img.data + row * pr_img.step;
    for (int col = 0; col < INPUT_W; ++col) {
      host_input_[i] = static_cast<float>(uc[2]) / 255.0f;
      host_input_[i + INPUT_H * INPUT_W] = static_cast<float>(uc[1]) / 255.0f;
      host_input_[i + 2 * INPUT_H * INPUT_W] = static_cast<float>(uc[0]) / 255.0f;
      uc += 3;
      ++i;
    }
  }

  // 固定形状 engine 也显式设置输入维度(防御性, 无副作用)
  nvinfer1::Dims4 in_dims(1, 3, INPUT_H, INPUT_W);
  if (!context_->setBindingDimensions(input_index_, in_dims)) {
    ROS_ERROR("yolov26 setBindingDimensions failed");
  }
  if (!context_->allInputDimensionsSpecified()) {
    ROS_WARN("yolov26: input dims not specified");
  }

  cudaError_t e_in = cudaMemcpyAsync(buffers_[input_index_], host_input_,
                             1 * 3 * INPUT_H * INPUT_W * sizeof(float),
                             cudaMemcpyHostToDevice, stream_);
  if (e_in != cudaSuccess) {
    ROS_ERROR("yolov26 H2D fail: %s (src=%p dst=%p sz=%d)",
              cudaGetErrorString(e_in), (void*)host_input_,
              buffers_[input_index_], 1 * 3 * INPUT_H * INPUT_W * sizeof(float));
  }
  CUDA_CHECK(cudaStreamSynchronize(stream_));  // 输入就绪后再推理
  if (!context_->executeV2(buffers_)) {
    ROS_WARN("yolov26 executeV2 failed");
  }
  cudaError_t launch_err = cudaGetLastError();
  if (launch_err != cudaSuccess) {
    ROS_ERROR("yolov26 launch error: %s", cudaGetErrorString(launch_err));
  }
  cudaError_t e_out = cudaMemcpy(host_output_, buffers_[output_index_],
                             1 * NUM_OUT * NUM_ANCHORS * sizeof(float),
                             cudaMemcpyDeviceToHost);
  if (e_out != cudaSuccess) {
    ROS_ERROR("yolov26 D2H fail: %s (src=%p dst=%p sz=%d)",
              cudaGetErrorString(e_out), buffers_[output_index_],
              (void*)host_output_, 1 * NUM_OUT * NUM_ANCHORS * sizeof(float));
  }

  // 解码 [1, 5, 8400] = (cx, cy, w, h, score)
  std::vector<Yolo::Detection> dets;
  dets.reserve(64);
  float *out = host_output_;  // 行主序 [5][8400]
  for (int a = 0; a < NUM_ANCHORS; ++a) {
    float cx = out[0 * NUM_ANCHORS + a];
    float cy = out[1 * NUM_ANCHORS + a];
    float w = out[2 * NUM_ANCHORS + a];
    float h = out[3 * NUM_ANCHORS + a];
    float score = out[4 * NUM_ANCHORS + a];
    if (score < threshold_) continue;
    Yolo::Detection d;
    d.bbox[0] = cx;
    d.bbox[1] = cy;
    d.bbox[2] = w;
    d.bbox[3] = h;
    d.conf = score;
    d.class_id = 0;
    dets.push_back(d);
  }

  nms_class_agnostic(dets);

  // 映射回原图坐标 (逆 letterbox)
  for (auto &d : dets) {
    d.bbox[0] = (d.bbox[0] - off_x) / scale;
    d.bbox[1] = (d.bbox[1] - off_y) / scale;
    d.bbox[2] = d.bbox[2] / scale;
    d.bbox[3] = d.bbox[3] / scale;
  }

  batch_res.push_back(std::move(dets));
  return batch_res;
}

}  // namespace dvision
