#pragma once

#include <sl/Camera.hpp>
#include <dvision/icamera.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace dvision {
  //! ZED M 相机捕获流类
  class ZedCamera : public ICamera {
  public:
    //! 构造函数
    ZedCamera();

    //! 析构函数
    ~ZedCamera() override;

    /**
     * \brief 初始化相机
     *
     * \return 初始化结果
     */
    bool initialize();

    /**
     * \brief 捕获相机图像
     *
     * \return 捕获的图像
     */
    Frame capture() override;

    /**
     * \brief 获取当前帧的点云数据 (ZED RIGHT_HANDED_Z_UP camera frame)
     *
     * 返回 CV_32FC4 格式的 cv::Mat，4 通道为:
     *   channel 0: X (相机右方, m)
     *   channel 1: Y (相机前方, m)
     *   channel 2: Z (相机上方, m)
     *   channel 3: packed color / SDK payload, not confidence
     */
    cv::Mat getPointCloudOptical() const;

    /**
     * \brief 在指定像素采样 XYZ 并转换为 camera FLU 坐标系
     *
     * FLU: X 前, Y 左, Z 上。
     * 从 ZED RIGHT_HANDED_Z_UP (X 右, Y 前, Z 上) 转换而来。
     * flu_x = sdk_y (前)
     * flu_y = -sdk_x (左)
     * flu_z = sdk_z (上)
     *
     * \return 采样是否有效 (XYZ finite)
     */
    bool sampleFLU(int u, int v, float& flu_x, float& flu_y, float& flu_z) const;

    /**
     * \brief Sample the latest point cloud into camera FLU coordinates.
     *
     * capture() already holds the ZED mutex; this helper is for code paths that
     * are called inside that lock and must not lock the camera a second time.
     */
    bool sampleCurrentFLU(int u, int v, float& flu_x, float& flu_y, float& flu_z) const;

  private:
    mutable std::mutex zed_mutex_;
    sl::Camera zed;       // ZED 相机对象
    sl::Mat image_;       // 图像数据 (BGRA)
    sl::Mat point_cloud_; // 点云数据 (4 通道 float: X, Y, Z, conf) - optical frame

    bool sampleFLUFromLatestPointCloud(int u, int v, float& flu_x,
                                       float& flu_y, float& flu_z) const;
  };
}
