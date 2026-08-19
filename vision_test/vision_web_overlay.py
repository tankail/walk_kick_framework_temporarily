#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dvision 视觉单独测试 —— Web 叠加查看节点 (walk_kick_framework)

在浏览器通过 http://<robot-ip>:8090 查看机器人相机画面:
  - 实时画面(来自 /dvision_1/cam_image, 需 dvision 以 VISION_ONLY+OfflineRecord 启动)
  - 球: 画一个圆 + 从画面中心到球的连线
  - 左侧文字面板: 球相对"眼睛"(相机) 与 base_link 的坐标

坐标约定(与 单独测试视觉流程.md 一致):
  - base_link: ball_field, 单位 cm; x前/ y左/ z上; 另给 m 值
  - 眼睛(相机): 由 base 坐标减去固定外参偏移得到, 单位 m;
    x前/ y左/ z上。仅在头部 yaw/pitch≈0 时准确
    (VISION_ONLY 模式无 motion 反馈, dvision 本身也按头部 0 计算)。

依赖: 系统 python3 (非 conda): rospy, sensor_msgs, dmsgs, numpy, cv2。
用法: 由 start_vision_test.sh 启动; 也可手动:
    source /opt/ros/noetic/setup.bash
    source $ROS1_WS/lib/devel/setup.bash
    source $ROS1_WS/core/devel/setup.bash
    /usr/bin/python3 vision_web_overlay.py --robot-id 1 --port 8090
"""

from __future__ import annotations

import argparse
import collections
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

try:
    import rospy
    from sensor_msgs.msg import Image as RosImage
    from dmsgs.msg import VisionInfo, MotionInfo
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Run this script in a ROS1 environment with dmsgs sourced: "
        "source /opt/ros/noetic/setup.bash + lib/core devel setup.bash"
    ) from exc

# 相机内参: ZED-M VGA(672x376) 实际值, 用 pyzed 读取 (2026-08-07)。
# 注意: dconfig camera.yml 里的 fx=386.5/cx=328.3 是 640x480 标定值,
#       cam_image 实际是 672x376, 用它直接缩放投影会偏 -> 必须用 ZED 真实内参。
CAM_FX, CAM_FY = 363.26531982421875, 363.26531982421875
CAM_CX, CAM_CY = 336.2613220214844, 190.2655487060547
CAM_BASE_W, CAM_BASE_H = 672, 376
# 相机相对 base_link 的偏移 (米)。来源: 单独测试视觉流程.md §6 快速校验式:
#   base_x_cm ~= 100*(camera_x_m + 0.063) 等
# 注意: 本框架(walk_kick_framework)的 dvision 已改为输出 相机FLU/米,
#       zed_dvision 再 FK 成 base_link/米。所以 VisionInfo.ball_field 已是"米"。
CAM_OFFSET = {"x": 0.063, "y": 0.0366, "z": 0.284}


class OverlayBuffer:
    """线程安全的最新帧 + 最新 VisionInfo 缓存, 供 HTTP 流输出。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg: bytes = b""
        self._seq = 0
        self._vision = None  # VisionInfo

    def set_jpeg(self, jpeg: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg
            self._seq += 1

    def snapshot(self):
        with self._lock:
            return self._jpeg, self._seq

    def set_vision(self, info) -> None:
        with self._lock:
            self._vision = info

    def get_vision(self):
        with self._lock:
            return self._vision


BUF = OverlayBuffer()
HTTP_STREAM_INTERVAL = 0.05  # 每个客户端刷新间隔 (s)


class MjpegHandler(BaseHTTPRequestHandler):
    """根路径 -> multipart MJPEG 流; /status -> 最近一次 VisionInfo 文本。"""

    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # 抑制默认日志刷屏
        pass

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        last_seq = -1
        try:
            while True:
                jpeg, seq = BUF.snapshot()
                if jpeg and seq != last_seq:
                    last_seq = seq
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                time.sleep(HTTP_STREAM_INTERVAL)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            pass

    def do_GET(self):
        if self.path in ("/", "/stream"):
            self._stream_mjpeg()
        elif self.path == "/status":
            body = _status_text().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()


def _status_text() -> str:
    info = BUF.get_vision()
    head_note = "head yaw={:.1f} pitch={:.1f}".format(HEAD_YAW_DEG, HEAD_PITCH_DEG)
    if info is None:
        return "vision: NO VisionInfo yet\n" + head_note + "\n"
    flu = get_camera_flu(info)
    if flu is None:
        return ("vision: see_ball=False\n" + head_note +
                "\n(head 来源: MotionInfo 自动 / rosparam set /vision_head_yaw_deg; "
                "单独测试无反馈默认0)\n")
    bx, by, bz = fk_camera_flu_to_base(flu)
    bvx, bvy, bvz = base_velocity(info)
    evx, evy, evz = eye_velocity()
    return (
        "vision: /dvision_{}/VisionInfo\n"
        "see_ball   : True\n"
        "eye(cam FLU,m): x={:+.2f} y={:+.2f} z={:+.2f}\n"
        "eye vel(m/s)  : x={:+.2f} y={:+.2f} z={:+.2f}\n"
        "base_link  (m): x={:+.2f} y={:+.2f} z={:+.2f}  ({})\n"
        "base vel(m/s) : x={:+.2f} y={:+.2f} z={:+.2f}\n"
        "ball_quality: {:.3f}\n"
    ).format(_ROBOT_ID,
             float(flu.x), float(flu.y), float(flu.z),
             evx, evy, evz,
             bx, by, bz, head_note,
             bvx, bvy, bvz,
             float(info.ball_quality))


# ---------------------------------------------------------------------- #
# 坐标计算
# ---------------------------------------------------------------------- #

def _deg2rad(deg: float) -> float:
    return float(deg) * math.pi / 180.0


# 头部角度(度)。来源优先级: 1) /dmotion_X/MotionInfo 订阅 2) CLI --head-*-deg 3) 默认 0
HEAD_YAW_DEG = 0.0
HEAD_PITCH_DEG = 0.0


def fk_camera_flu_to_base(flu, head_yaw_deg: float = None,
                          head_pitch_deg: float = None) -> tuple:
    """camera FLU 球坐标 + 头部 yaw/pitch -> base_link 坐标(米)。

    与 zed_dvision.cpp step2 的 FK 完全一致:
      链: base->head_yaw(0.014,-0.000267,0.1981) -> head_pitch(0,0,0.0275)
          -> camera_link(0.048925,0.005352,0.058472) -> left_eye(0,0.0315,0)
      R_base_head_pitch = R_z(yaw) @ R_y(pitch)
      R_base_left_eye_optical = R_base_head_pitch @ R_cam_optical
    注意: 单独视觉测试(dvision VISION_ONLY)无 motion 反馈, dvision 按头部=0,
          这里用真实头部角度重算后才是真 base_link。
    """
    cam_fx = float(flu.x)
    cam_fy = float(flu.y)
    cam_fz = float(flu.z)
    hy = _deg2rad(float(head_yaw_deg) if head_yaw_deg is not None else HEAD_YAW_DEG)
    hp = _deg2rad(float(head_pitch_deg) if head_pitch_deg is not None else HEAD_PITCH_DEG)
    cy_, sy_ = math.cos(hy), math.sin(hy)
    cp, sp = math.cos(hp), math.sin(hp)

    R00, R01, R02 = cy_ * cp, -sy_, cy_ * sp
    R10, R11, R12 = sy_ * cp, cy_, sy_ * sp
    R20, R21, R22 = -sp, 0.0, cp

    # camera FLU -> left_eye_optical: X_r=-Y_l, Y_d=-Z_u, Z_f=X_f
    opt_x, opt_y, opt_z = -cam_fy, -cam_fz, cam_fx

    # R_base_left_eye_optical
    Ro00, Ro01, Ro02 = sy_, -cy_ * sp, cy_ * cp
    Ro10, Ro11, Ro12 = -cy_, -sy_ * sp, sy_ * cp
    Ro20, Ro21, Ro22 = 0.0, -cp, -sp

    # translation: left_eye 在 base_link 的位置
    tx, ty, tz = 0.014, -0.000267, 0.1981 + 0.0275
    cam_lx, cam_ly, cam_lz = 0.048925, 0.005352 + 0.0315, 0.058472
    tx += R00 * cam_lx + R01 * cam_ly + R02 * cam_lz
    ty += R10 * cam_lx + R11 * cam_ly + R12 * cam_lz
    tz += R20 * cam_lx + R21 * cam_ly + R22 * cam_lz

    bx = tx + Ro00 * opt_x + Ro01 * opt_y + Ro02 * opt_z
    by = ty + Ro10 * opt_x + Ro11 * opt_y + Ro12 * opt_z
    bz = tz + Ro20 * opt_x + Ro21 * opt_y + Ro22 * opt_z
    return bx, by, bz


def get_camera_flu(info):
    """VisionInfo -> camera FLU 球坐标(米, x前/y左/z上)。无球返回 None。

    ball_global 由 zed_dvision 存成原始 camera FLU(不受头部影响)。
    """
    if info is None or not bool(info.see_ball):
        return None
    return info.ball_global


def project_ball_to_pixel(info, width: int, height: int):
    """用 camera-FLU 球坐标(ball_global)直接投影到像素, 不经 ballTrack, 更准。

    camera FLU: x前 y左 z上 -> 像素 u右/v下:
      u = cx - fx * (FLU_y / FLU_x)
      v = cy - fy * (FLU_z / FLU_x)
    """
    flu = get_camera_flu(info)
    if flu is None:
        return None
    fx = CAM_FX * width / float(CAM_BASE_W)
    fy = CAM_FY * height / float(CAM_BASE_H)
    cx = CAM_CX * width / float(CAM_BASE_W)
    cy = CAM_CY * height / float(CAM_BASE_H)
    fxv = float(flu.x)
    if abs(fxv) < 1e-6:
        return None
    u = cx - fx * (float(flu.y) / fxv)
    v = cy - fy * (float(flu.z) / fxv)
    return int(round(u)), int(round(v))


def ball_radius_px(info) -> int:
    flu = get_camera_flu(info)
    if flu is None:
        return 10
    dist_m = math.hypot(float(flu.x), float(flu.y))
    r = 0.075 * CAM_FY / max(dist_m, 0.3)  # 球半径7.5cm * fy / dist
    return int(max(4, min(60, r)))


# ---------------------------------------------------------------------- #
# 球速度 (相机系 = ball_global 历史差分; base_link = dvision ball_velocity)
# ---------------------------------------------------------------------- #
_FLU_HIST = collections.deque(maxlen=30)  # (t, x, y, z) camera FLU, 用于相机速度差分


def record_flu_history(info) -> None:
    """每个 VisionInfo 回调记录一帧 ball_global(camera FLU) + 时间戳; 无球时记 0。"""
    now = time.time()
    flu = get_camera_flu(info)
    if flu is not None:
        _FLU_HIST.append((now, float(flu.x), float(flu.y), float(flu.z)))
    else:
        _FLU_HIST.append((now, 0.0, 0.0, 0.0))


def eye_velocity() -> tuple:
    """相机系球速度 (m/s, FLU) —— ball_global 历史有限差分。"""
    if len(_FLU_HIST) < 2:
        return 0.0, 0.0, 0.0
    (t0, x0, y0, z0), (t1, x1, y1, z1) = _FLU_HIST[-2], _FLU_HIST[-1]
    dt = t1 - t0
    if dt <= 1e-4:
        return 0.0, 0.0, 0.0
    return ((x1 - x0) / dt, (y1 - y0) / dt, (z1 - z0) / dt)


def base_velocity(info) -> tuple:
    """base_link 球速度 (m/s) —— dvision 计算, VisionInfo.ball_velocity (位置历史窗差分)。"""
    if info is None:
        return 0.0, 0.0, 0.0
    return (float(info.ball_velocity.x), float(info.ball_velocity.y),
            float(info.ball_velocity.z))


# ---------------------------------------------------------------------- #
# 画面绘制
# ---------------------------------------------------------------------- #

def draw_overlay(img_bgr: np.ndarray, info) -> np.ndarray:
    h, w = img_bgr.shape[:2]
    img = img_bgr.copy()

    # 画面中心十字
    cv2.line(img, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 200, 200), 1)
    cv2.line(img, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 200, 200), 1)

    if info is not None and bool(info.see_ball):
        pt = project_ball_to_pixel(info, w, h)
        if pt is not None:
            px, py = pt
            r = ball_radius_px(info)
            cv2.circle(img, (px, py), r, (0, 255, 0), 2)
            cv2.line(img, (w // 2, h // 2), (px, py), (0, 255, 0), 1)
            cv2.putText(img, "ball", (px + r + 6, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 左上文字面板
    _draw_panel(img, info)
    return img


def _draw_panel(img: np.ndarray, info) -> None:
    h, w = img.shape[:2]
    if info is None:
        lines = ["waiting VisionInfo ..."]
        mode = "vision: N/A"
    else:
        flu = get_camera_flu(info)
        bx, by, bz = fk_camera_flu_to_base(flu) if flu is not None else (0.0, 0.0, 0.0)
        ex, ey, ez = ((float(flu.x), float(flu.y), float(flu.z))
                      if flu is not None else (0.0, 0.0, 0.0))
        mode = "VISION_ONLY" if rospy.get_param("/ZJUDancer/VisionOnlyMode", False) else "REALWORLD"
        bvx, bvy, bvz = base_velocity(info)
        evx, evy, evz = eye_velocity()
        lines = [
            "see_ball : {}".format("True" if bool(info.see_ball) else "False"),
            "ball_quality: {:.3f}".format(float(info.ball_quality)),
            "base_link  (m):  x={:+.2f}  y={:+.2f}  z={:+.2f}".format(bx, by, bz),
            "base vel(m/s):  x={:+.2f}  y={:+.2f}  z={:+.2f}".format(bvx, bvy, bvz),
            "eye(camFLU,m):  x={:+.2f}  y={:+.2f}  z={:+.2f}".format(ex, ey, ez),
            "eye vel(m/s):  x={:+.2f}  y={:+.2f}  z={:+.2f}".format(evx, evy, evz),
            "head yaw/pitch: {:.1f}/{:.1f} deg".format(HEAD_YAW_DEG, HEAD_PITCH_DEG),
        ]

    font = cv2.FONT_HERSHEY_SIMPLEX
    fs = 0.55
    th = 1
    x0, y0, pad = 10, 22, 8
    line_h = int(24 * (h / 480.0))
    panel_h = line_h * (len(lines) + 2)
    panel_w = w // 2
    cv2.rectangle(img, (x0 - pad, y0 - line_h + 6), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.putText(img, mode, (x0, y0), font, 0.6, (0, 220, 255), 2)
    yy = y0 + line_h
    for line in lines:
        color = (0, 255, 0) if line.startswith("see_ball") else (255, 255, 255)
        cv2.putText(img, line, (x0, yy), font, fs, color, th)
        yy += line_h


# ---------------------------------------------------------------------- #
# ROS 回调 + 编码线程
# ---------------------------------------------------------------------- #

_ROBOT_ID = 1


def image_cb(msg: RosImage) -> None:
    if msg.encoding not in ("bgr8", "rgb8"):
        return
    arr = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.step))
    img = arr[:, : msg.width * 3].reshape((msg.height, msg.width, 3)).copy()
    info = BUF.get_vision()
    drawn = draw_overlay(img, info)
    ok, jpeg = cv2.imencode(".jpg", drawn, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if ok:
        BUF.set_jpeg(jpeg.tobytes())


def vision_cb(msg: VisionInfo) -> None:
    record_flu_history(msg)
    BUF.set_vision(msg)


def motion_cb(msg) -> None:
    """dancer-io 在跑时, 用真实头部角度覆盖 CLI 默认值。"""
    global HEAD_YAW_DEG, HEAD_PITCH_DEG
    HEAD_YAW_DEG = float(msg.curPlat.yaw)
    HEAD_PITCH_DEG = float(msg.curPlat.pitch)


def head_param_timer_cb(event) -> None:
    """单独测试无 motion 反馈时, 用户可 live 设置头部角度:
        rosparam set /vision_head_yaw_deg <deg>
        rosparam set /vision_head_pitch_deg <deg>
    有 MotionInfo 时 motion_cb 优先覆盖。
    """
    global HEAD_YAW_DEG, HEAD_PITCH_DEG
    try:
        if rospy.has_param("/vision_head_yaw_deg"):
            HEAD_YAW_DEG = float(rospy.get_param("/vision_head_yaw_deg"))
        if rospy.has_param("/vision_head_pitch_deg"):
            HEAD_PITCH_DEG = float(rospy.get_param("/vision_head_pitch_deg"))
    except Exception as exc:  # pragma: no cover
        rospy.logwarn("head param read failed: %s", exc)


def encode_loop(rate: float) -> None:
    """即使没有新图像帧也周期性重编码(取最新缓存信息)。"""
    period = 1.0 / max(rate, 1.0)
    last_img = None
    while not rospy.is_shutdown():
        # 无 rospy 拉取时, 这里不主动订阅缓存图像; 交给 image_cb 驱动。
        # 保留此线程以将来支持"无 cam_image 时显示占位图"。
        time.sleep(period)


def main() -> None:
    global _ROBOT_ID, HEAD_YAW_DEG, HEAD_PITCH_DEG
    p = argparse.ArgumentParser(description="dvision 视觉单独测试 Web 叠加")
    p.add_argument("--robot-id", type=int, default=1)
    p.add_argument("--host", type=str, default="0.0.0.0")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--rate", type=float, default=10.0, help="重编码兜底频率(Hz)")
    p.add_argument("--head-yaw-deg", type=float, default=0.0,
                   help="头部 yaw(度)。单独测试无 motion 反馈时用; 有 MotionInfo 则自动用真实值")
    p.add_argument("--head-pitch-deg", type=float, default=0.0,
                   help="头部 pitch(度)")
    args = p.parse_args()
    _ROBOT_ID = int(args.robot_id)
    HEAD_YAW_DEG = float(args.head_yaw_deg)
    HEAD_PITCH_DEG = float(args.head_pitch_deg)

    rospy.init_node("vision_web_overlay", anonymous=True)
    image_topic = "/dvision_{}/cam_image".format(_ROBOT_ID)
    vision_topic = "/dvision_{}/VisionInfo".format(_ROBOT_ID)
    motion_topic = "/dmotion_{}/MotionInfo".format(_ROBOT_ID)
    rospy.Subscriber(image_topic, RosImage, image_cb, queue_size=1)
    rospy.Subscriber(vision_topic, VisionInfo, vision_cb, queue_size=1)
    # dancer-io 在跑时订阅真实头部角度; 单独测试可用 rosparam 设 /vision_head_{yaw,pitch}_deg
    rospy.Subscriber(motion_topic, MotionInfo, motion_cb, queue_size=1)
    rospy.Timer(rospy.Duration(0.5), head_param_timer_cb)
    msg = "subscribing {} + {} (+{}); head via MotionInfo or rosparam".format(
        image_topic, vision_topic, motion_topic)
    rospy.loginfo(msg)
    print(msg, flush=True)

    threading.Thread(target=encode_loop, args=(args.rate,), daemon=True).start()

    server = ThreadingHTTPServer((args.host, int(args.port)), MjpegHandler)
    msg = "web overlay at http://<robot-ip>:{}  (stream=/, status=/status)".format(
        int(args.port))
    rospy.loginfo(msg)
    print(msg, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
