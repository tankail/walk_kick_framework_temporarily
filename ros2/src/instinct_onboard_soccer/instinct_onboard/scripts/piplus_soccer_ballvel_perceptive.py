from __future__ import annotations

"""
PiPlus Walk + Kick + BALLVEL **HeadFree** Perceptive Node  (22-DOF robot, 20-DOF policy)
========================================================================================

给 **walk+kick mcritics + ballvel + HeadFree** 模型用。
对应训练 cfg: WalkKickMCriticsBallVelHeadFreeEnvCfg
    (pi_plus_walk_kick_mcritics_ballvel_headfree_env_cfg.py)

和 piplus_walk_kick_ballvel_perceptive.py 的唯一本质区别:
    ** 头部不在策略动作空间里。 **
    策略输出 20 维 (非 head 关节), head 由本脚本自己控制 (追球 / 锁死 / 交给别人)。
    训练时 head 由 HeadFreeJointPositionAction 内部平滑随机目标驱动, 策略只是
    "在头乱动的情况下也能走能踢", 所以部署时头怎么动都在训练分布内 —— 前提是
    ** 幅度和速度不超过训练范围 ** (见下面 head 段)。

如果你的模型是 22 维动作的老模型 (head 归策略管), 用
   piplus_walk_kick_ballvel_perceptive.py, 不是本文件。
本脚本启动时会自动核对 ONNX 输入/输出维度, 装错模型会直接报错退出, 不会带病上机。

本文件自包含, 不 import 任何同目录脚本。

命令向量 7 维 (和 ballvel 版完全一致):
    [0:3] vx,vy,wz            walk
    [3:5] ball_x_b,ball_y_b   ball pos (yaw-base, 视觉) —— async 历史喂 encoder
    [5:7] kick_dir_b,kick_spd kick

按键:
    默认    : 走路 (左摇杆 vx/vy, 右摇杆 wz)
    按住 A  : 踢球 (左摇杆方向 = kick 方向, 速度 = --kick_speed)

policy obs (与 headfree env.yaml 逐项对齐, posonly=False):
    base_ang_vel        24  (imu_ang_vel,            hist 8, scale 0.25)
    projected_gravity   24  (imu_projected_gravity,  hist 8)
    soccer_command      50  (soccer_command_async,   hist1; 7维删球位置[3,4]=5列x10)
    joint_pos          176  (** 仍是 22 关节, head 保留 ** 8x22)
    joint_vel          176  (** 仍是 22 关节, head 保留 ** 8x22)
    actions            160  (** 20 维, 无 head ** 8x20)   <- 和 ballvel 版的 176 不同
    ball_pos_b_noisy    20  (ball_pos_b_noisy_hist,      10x2)
    ball_vel_b_noisy    20  (ball_vel_b_noisy_hist_async, 10x2)   <- posonly 时无此项
    ------------------------------------------------------------------
    total = 650  (posonly=True 时 630)
    对照: 非 headfree 的 ballvel 版是 666 / 646。差的 16 = head 2 列 x 8 帧 last_action。

============================ 动作 / 头部的三条铁律 ========================
1) 策略输出 20 维, 按 **sim 关节顺序里剔掉 head 之后的次序** 对应关节:
       sim idx: [1,2,3,4, 6,7,8,9, 10..21]   (head_yaw=0, head_pitch=5 被剔除)
   这个映射不是写死的, 是启动时用 env.yaml 里 actions.joint_pos.joint_names 的
   正则 (`(?!.*head).*`) 按 IsaacLab 的 re.fullmatch 语义在 sim_joint_names 上重解出来的,
   并和 ONNX 输出维度对账。启动日志会打完整对照表, ** 上机前扫一眼 **。

2) last_action obs 喂 20 维 (无 head)。
   老版本里 head_track 覆盖值会污染 last_action -> encoder OOD, 所以有
   --head_obs_overridden 这个开关。** 本任务不存在这个问题 ** (head 根本不在动作里),
   开关保留为 no-op 只是为了老启动脚本不报错。

3) head 的 action_scale 在 headfree 的 env.yaml 里是 **0** (head 不是动作), 直接拿它
   做 (target-offset)/scale 会除零 -> ColdStartAgent 出 inf -> 下发 NaN。
   本脚本给 head 两列补一个 **部署侧自定的** action_scale (--head_action_scale,
   默认 0.0961 = 老任务的 0.25*effort/stiffness), 纯内部换算, 策略看不到。
   head 的 kp/kd 仍然从 env.yaml 的 actuators["head"] 读 (≈7.80 / 0.497), 和训练一致。
=========================================================================

============================ 头部控制 --head_mode ========================
track (默认): 本脚本用视觉把头指向球 (原 --head_track 那套: 死区+迟滞+比例逼近+限速)。
              ** 视觉节点要关掉它自己的头部跟踪 ** (ball_vision_ros2.py --no-head-track),
              否则两边抢头。
lock        : 固定看向 --head_pitch_deg / --head_yaw_deg。
zero_gain   : 头部 kp=kd=0 且目标=当前实测角度, 把头完全让给别的控制器。
              ⚠ 只有当那个控制器 **自己下发 head 的 MotorControlCommand** 时才有意义;
                两个节点交替下发同一路电机会让 kp 在 0 和它的值之间跳变。默认别用。

头部运动范围/速度 从 env.yaml 的 actions.joint_pos.head_pos_range / head_max_speed 读,
读不到才退回 0.9xURDF 限位 (yaw ±1.287, pitch [-0.846, +0.792]) 和 2.0 rad/s。
** 部署侧的头部限位和限速都会被夹到训练范围内 **: 超出训练范围的头部姿态/速度,
对身体策略而言是没见过的惯量扰动, 是 sim2real 差距的直接来源。
=========================================================================

============================ 四个正交的部署旋钮 ============================
--posonly            : encoder 观测是否含球速
    带        -> 630 (只喂球位置, BallEstimatorActorCritic)
    不带(默认) -> 650 (加喂 ball_vel_b_noisy, BallPosVelEstimatorActorCritic)
    ** 必须和加载模型的 env.yaml 一致, 否则启动时维度自检会报错。 **

--ball_frame {camera, base_link}
    camera(默认): /soccer_perception/ball_position (PointStamped, ZED OPTICAL);
                  脚本做 optical -> base_link(头部 FK) -> yaw(IMU detilt)。
    base_link   : /soccer/ball_base_xy_velocity (已在 base 系); 跳过相机变换。

--ball_vel_source {finite_diff, measured}  (只在 non-posonly 时有意义)
    finite_diff(默认): 球位置历史有限差分估计。
    measured         : 直接用视觉话题里的球速 (仅 base_link; camera 会强制回退)。

--ball_base_msg_type {Float32MultiArray, PointStamped, Odometry} : base_link 话题消息类型
=========================================================================

============================ 球 obs 的两层结构 ==========================
(和 ballvel 版完全一致, 一个字没改。阈值见 WalkKickBallVelHeadFreeEncDecAgent.BALL_*)

第 1 层 —— buffer 写什么  (encoder 全程看得到球)
    _soccer_hist / _soccer_vel_hist 里 **永远** 滚动真实球, 走路时也写。
    归 0 规则照抄训练 (observations.py), 只看距离和新鲜度, 和 kick 指令无关:
        dist > BALL_FAR_MASK_DIST (2.2m) -> pos=0 且 vel=0
        hold-last > BALL_HOLD_MAX_S (1.0s) -> 同上
    finite_diff 不跨 mask 边界做差分 (否则 0->0.36m 会算出 3.6 m/s 假球速)。

第 2 层 —— latent 接不接进策略  (kick_speed != 0 才接)
    开门: 按住 A **且** 左摇杆出死区; 松手后再保持 BALL_GATE_RELEASE_HOLD_S (0.5s)。
    门关: obs getter 返回全 0 -> encoder 输出训练里"球在 far_mask 外"的那个 latent。
          ** 不是把 z 置 0 ** —— 实测无球时 ||z||≈28.65 而非 0, 强行置 0 才是 OOD。
=========================================================================

用法 (base_link 视觉 + 测量球速 + 自己追球, 推荐真机):
    python piplus_walk_kick_ballvel_headfree_perceptive.py --logdir <run> --nodryrun \\
        --ball_frame base_link --ball_vel_source measured \\
        --ball_base_topic /soccer/ball_base_xy_velocity --ball_base_msg_type Odometry \\
        --head_mode track --kick_speed 3.5 --kick_dir_mode follow
"""

import collections
import math
import os
import queue
import re
import time
from typing import Optional, Tuple

import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent, OnboardAgent
from instinct_onboard.normalizer import Normalizer
from instinct_onboard.ros_nodes.base import RealNode
from instinct_onboard.ros_nodes.hightorque import HightorqueNode

MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500
MAIN_LOOP_DT = 0.02  # 50 Hz

DEFAULT_BALL_POSITION_CAMERA_TOPIC = "/soccer_perception/ball_position"
DEFAULT_BALL_BASE_TOPIC = "/soccer/ball_base_xy_velocity"

# --------------------------------------------------------------------------- #
# Camera extrinsics / head FK constants (仅 --ball_frame camera 时用到)
# --------------------------------------------------------------------------- #
ZED_MINI_BASELINE_M = 0.063
CAMERA_LINK_POS_HEAD_PITCH = np.array([0.048925, 0.0053519, 0.058472], dtype=np.float64)
LEFT_EYE_POS_CAMERA_LINK = np.array([0.0, 0.5 * ZED_MINI_BASELINE_M, 0.0], dtype=np.float64)
HEAD_YAW_POS_BASE = np.array([0.014, -0.00026667, 0.1981], dtype=np.float64)
HEAD_PITCH_POS_HEAD_YAW = np.array([0.0, 0.0, 0.0275], dtype=np.float64)
R_CAMERA_LINK_BALL_INPUT = np.eye(3, dtype=np.float64)

# --------------------------------------------------------------------------- #
# 视觉管线延迟: 球话题 header.stamp 所指时刻 -> 快门时刻 的差值 (s)
# --------------------------------------------------------------------------- #
# ball_vision_ros2.py:674 填的是 **发布时刻** (get_clock().now()), 不是快门时刻,
# 所以 ZED grab + YOLO + 深度这一整段没有被计进 stamp 里。
#
# 标定方法: 球固定不动, 用手匀速左右转头, 记录 "重建方位角" 和 "头角速度",
#           线性回归的斜率就是 τ。   dβ/dθ̇ = τ
# 填错的代价: 重建误差 = |τ_true - PIPELINE_LATENCY_S| x 头角速度;
#             填 0.0 等价于改动前的行为 (stamp 和 now 只差 ROS 传输那几毫秒)。
#
# 如果以后把视觉端改成用 ZED 图像时间戳填 header (sl.TIME_REFERENCE.IMAGE),
# 或者把 ball_vision_ros2.py:387 那个 self._stamp 透传出来, 这里就填 0.0。
PIPELINE_LATENCY_S = 0.15

# 头部角度历史长度 (control step @50Hz)。要能覆盖 PIPELINE_LATENCY_S。
HEAD_HIST_LEN = 64          # 1.28 s

# Head joint hard limits (rad), from PiPlus_S_12L8A0G2H0W.urdf.
#   head_yaw_joint  : URDF [-1.43, 1.43]
#   head_pitch_joint: URDF [-0.94, 0.88]  (positive = look DOWN, per _rot_y about +y)
HEAD_URDF_LIMITS = {
    "head_yaw_joint":   (-1.43, 1.43),
    "head_pitch_joint": (-0.94, 0.88),
}
# 训练里 head 的活动范围 = 0.9 x URDF (pi_plus_walk_kick_env_cfg._HEAD_RANGE_RATIO)。
# 这只是 env.yaml 读不到 head_pos_range 时的兜底; 正常情况以 yaml 为准。
HEAD_RANGE_RATIO_FALLBACK = 0.9
HEAD_MAX_SPEED_FALLBACK = 2.0        # rad/s, = HeadFreeJointPositionActionCfg.head_max_speed

# head 在 headfree 的 env.yaml 里没有 action_scale (head 不是动作) -> 解析出来是 0。
# 这里补一个部署侧自定的换算系数, 只用于 head 目标角 <-> action 列的互转:
#     action_head = (target_rad - offset_head) / HEAD_ACTION_SCALE
# 取值本身不影响物理 (乘回去就抵消了), 默认沿用非 headfree 任务的 head scale
#     0.25 * effort_limit_sim(3.0) / stiffness_3536(7.8009) = 0.09614
# 好处是 /raw_actions 上 head 两列的数量级和老脚本一致, 对比日志时不用换算。
DEFAULT_HEAD_ACTION_SCALE = 0.09614231747988017

# NOTE: 真正生效的 head 限位是 agent 实例上的 _head_yaw_lo/_head_yaw_hi/_head_pitch_lo/
#       _head_pitch_hi —— 它们在 _parse_action_config() 里从 env.yaml 的
#       actions.joint_pos.head_pos_range 读出来, 读不到才用上面的 0.9xURDF 兜底。
#       这里刻意不留模块级常量, 免得有人改了常量却以为改了限位。


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def resolve_logdir_and_actor(path: str) -> Tuple[str, Optional[str]]:
    path = os.path.abspath(path)
    if path.endswith(".onnx"):
        actor_path = path
        logdir = os.path.dirname(os.path.dirname(path))
        return logdir, actor_path
    return path, None


def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quat_wxyz(q: np.ndarray) -> float:
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle); s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle); s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def _quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w = float(q[0])
    u = np.array([q[1], q[2], q[3]], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64).reshape(3)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def resolve_matching_joint_ids(sim_joint_names, patterns) -> list:
    """复刻 IsaacLab 的 string_utils.resolve_matching_names(preserve_order=False) 语义。

    ** 必须用 re.fullmatch, 不能用 re.search。 ** headfree 的 joint_names 是
    `(?!.*head).*`:
        re.fullmatch(pat, "head_yaw_joint") -> None   (位置 0 的 lookahead 失败) ✅
        re.search  (pat, "head_yaw_joint") -> 命中!   (从位置 1 起 "ead_yaw_joint"
                                                       里没有 "head", lookahead 通过)
    instinct_onboard 的 OnboardAgent._parse_action_config 用的就是 re.search ——
    对这个正则会把 head 也算成"匹配", 只是 scale dict 里没有 head 的 key,
    所以 head 的 scale 恰好留成 0。别照抄那份语义来定动作列。

    preserve_order=False -> 返回值按 sim 关节顺序升序 (IsaacLab 的默认行为),
    也就是策略 action[k] 对应 sim_joint_names[out[k]]。
    """
    out = []
    for i, name in enumerate(sim_joint_names):
        for pat in patterns:
            if re.fullmatch(pat, name):
                out.append(i)
                break
    return out


# --------------------------------------------------------------------------- #
# Agent: walk + kick, ONNX encdec, encoder pos(+vel), camera/base_link 双链路
# --------------------------------------------------------------------------- #
class WalkKickBallVelHeadFreeEncDecAgent(OnboardAgent):
    """ONNX 推理 agent (HeadFree)。命令 7 维 (walk/kick)。

    策略输出 20 维 (无 head), 本类负责:
      - 把 20 维散射回 22 维 sim 关节动作向量;
      - head 两列由自己的控制器 (track / lock / zero_gain) 填;
      - 把 20 维原始输出留给 node 当 last_action obs。
    """

    LSTICK_DEADZONE: float = 0.15
    _LSTICK_DEADZONE_SQ: float = LSTICK_DEADZONE * LSTICK_DEADZONE

    # ---------------- 球 obs 常量 (无 CLI 开关, 要改直接改这里) ----------------
    # 第 1 层: buffer 写入的距离/新鲜度 mask —— 和训练 observations.py 完全对齐
    BALL_FAR_MASK_DIST: float = 1.2      # 训练 WalkKickCommandCfg.ball_far_mask_dist (20260730 策略 env.yaml=1.2, 老 agent 注释写 2.2 已过时)
    BALL_HOLD_MAX_S: float = 1.0         # 视觉断流 hold-last 上限 ≈ 一个 encoder 窗
    # 第 2 层: 门控在松开 A 之后再保持多久才关
    #   踢球摆腿本身 ~0.3s, 松手就立刻抽走球 obs 会在挥腿中途改变 latent -> 半途收腿。
    #   设 0.0 = 松手立刻关门。
    BALL_GATE_RELEASE_HOLD_S: float = 0.5

    def __init__(
        self,
        logdir: str,
        ros_node: RealNode,
        actor_path: Optional[str] = None,
        kick_speed: float = 4.5,
        x_vel_scale: float = 1.0,
        y_vel_scale: float = -0.5,
        yaw_vel_scale: float = -1.2,
        kick_dir_mode: str = "anchor",
        soccer_pub_period: int = 5,
        soccer_history_samples: int = 10,
        ball_topic_timeout: float = 0.2,
        # ---- head: 策略不管头, 头归本脚本 ----
        head_mode: str = "track",              # track | lock | zero_gain
        head_action_scale: float = DEFAULT_HEAD_ACTION_SCALE,
        head_pitch_rad: float = math.radians(45.0),
        head_yaw_rad: float = 0.0,
        # ---- head ball-tracking 参数 (head_mode="track") ----
        head_track_kp: float = 0.35,
        head_track_deadband_rad: float = math.radians(7.0),
        head_track_reacquire_rad: float = math.radians(12.0),
        head_track_smooth: float = 0.4,
        head_track_max_rate_rad_s: float = math.radians(90.0),
        head_track_search_pitch_rad: float = math.radians(45.0),
        head_track_search_rate_rad_s: float = math.radians(30.0),
        head_track_hold_grace_s: float = 0.5,
        head_track_yaw_trim_rad: float = 0.0,
        head_track_pitch_trim_rad: float = 0.0,
        # ---- encoder 球速 & 坐标系 ----
        posonly: bool = False,
        ball_vel_scale: float = 0.25,
        ball_frame: str = "camera",
        ball_vel_source: str = "finite_diff",
        ball_base_detilt: bool = False,
    ):
        super().__init__(logdir, ros_node)
        self._actor_path_override = actor_path

        self._kick_speed_const: float = float(kick_speed)
        self.x_vel_scale = float(x_vel_scale)
        self.y_vel_scale = float(y_vel_scale)
        self.yaw_vel_scale = float(yaw_vel_scale)

        kick_dir_mode = str(kick_dir_mode).lower()
        if kick_dir_mode not in ("follow", "anchor"):
            raise ValueError(f"kick_dir_mode must be 'follow' or 'anchor', got {kick_dir_mode!r}")
        self._kick_dir_mode = kick_dir_mode

        self._pub_period: int = max(1, int(soccer_pub_period))
        self._history_samples: int = max(1, int(soccer_history_samples))
        self._soccer_hist: Optional[np.ndarray] = None      # (H, 7) newest at -1 (10Hz async)
        self._clean_hist: Optional[np.ndarray] = None        # (H, 7) newest at -1 (50Hz clean)
        self._soccer_vel_hist: Optional[np.ndarray] = None   # (H, 2) 测量球速 async (yaw-base, 未 scale)
        self._pub_counter: int = 0
        self._step_id: int = 0
        self._ball_hist_step_id: int = -1
        # (t, head_yaw, head_pitch) 环形缓冲, 每个 control step 采样一次。
        # 用来按图像时间戳回溯头部角度 -> 修掉 "用当前头角度投影 τ 毫秒前的图" 这个正反馈。
        self._head_hist: "collections.deque" = collections.deque(maxlen=HEAD_HIST_LEN)
        self._last_cmd: Optional[np.ndarray] = None

        self._kick_was_active: bool = False
        self._robot_yaw_w_at_anchor: float = 0.0
        self._kick_dir_yaw_w: float = 0.0
        self._last_valid_ball_xy: Optional[Tuple[float, float]] = None
        self._last_valid_ball_vel: Optional[Tuple[float, float]] = None
        # 策略的 20 维原始输出 (无 head)。node 拿它当 last_action obs ——
        # HeadFree 下 head 根本不在动作空间里, 不存在老版本"head 覆盖值污染 last_action"
        # 那个 OOD 问题, 这里留一份只是为了发 /policy_raw_actions 和跨 agent 共享。
        self._raw_policy_action: Optional[np.ndarray] = None

        self._ball_topic_timeout = float(ball_topic_timeout)
        self._head_yaw_joint_idx = self.ros_node.sim_joint_names.index("head_yaw_joint")
        self._head_pitch_joint_idx = self.ros_node.sim_joint_names.index("head_pitch_joint")

        head_mode = str(head_mode).lower()
        if head_mode not in ("track", "lock", "zero_gain"):
            raise ValueError(f"head_mode must be track/lock/zero_gain, got {head_mode!r}")
        self._head_mode = head_mode
        self._head_track = (head_mode == "track")
        self._head_lock = (head_mode == "lock")
        self._head_zero_gain = (head_mode == "zero_gain")
        self._head_action_scale_req = float(head_action_scale)
        if abs(self._head_action_scale_req) < 1e-6:
            raise ValueError("head_action_scale 不能是 0 (会导致 ColdStart 除零 -> NaN 目标)")
        self._head_pitch_target_rad = float(head_pitch_rad)
        self._head_yaw_target_rad = float(head_yaw_rad)
        self._head_lock_action_yaw: Optional[float] = None
        self._head_lock_action_pitch: Optional[float] = None

        # ---- head ball-tracking ----
        self._ht_kp = float(head_track_kp)
        self._ht_deadband = float(head_track_deadband_rad)          # center freeze zone (half-width)
        self._ht_reacquire = max(float(head_track_reacquire_rad), self._ht_deadband + 1e-3)  # hysteresis outer
        self._ht_smooth = float(np.clip(head_track_smooth, 0.05, 1.0))  # EMA weight of new sample
        self._ht_max_rate = float(head_track_max_rate_rad_s)
        # 真正的钳位在 _finalize_head_cfg() 里做 —— 那时才从 env.yaml 读到了训练的
        # head_pos_range / head_max_speed。这里只存原始请求值。
        self._ht_search_pitch_req = float(head_track_search_pitch_rad)
        self._ht_search_pitch = self._ht_search_pitch_req
        self._ht_search_rate = float(head_track_search_rate_rad_s)
        self._ht_hold_grace = float(head_track_hold_grace_s)
        self._ht_yaw_trim = float(head_track_yaw_trim_rad)
        self._ht_pitch_trim = float(head_track_pitch_trim_rad)
        # commanded head state (last sent target); start looking down/centered to search
        self._head_cmd_yaw: float = 0.0
        self._head_cmd_pitch: float = self._ht_search_pitch
        self._head_track_stale_t: float = self._ht_hold_grace
        # EMA-filtered target + hysteresis latch (freeze while ball is in the center zone)
        self._des_yaw_f: Optional[float] = None
        self._des_pitch_f: Optional[float] = None
        # 追球闩锁: yaw / pitch 各自独立 (共用一个会让 pitch 的限位误差把 yaw 也拽开)
        self._yaw_track_active: bool = False
        self._pitch_track_active: bool = False

        # ---- encoder 球速 & 坐标系 ----
        self._posonly: bool = bool(posonly)
        self._ball_vel_scale: float = float(ball_vel_scale)
        self._ball_frame: str = str(ball_frame).lower()
        if self._ball_frame not in ("camera", "base_link"):
            raise ValueError(f"ball_frame must be 'camera' or 'base_link', got {ball_frame!r}")
        self._ball_vel_source: str = str(ball_vel_source).lower()
        if self._ball_vel_source not in ("finite_diff", "measured"):
            raise ValueError(f"ball_vel_source must be 'finite_diff' or 'measured', got {ball_vel_source!r}")
        if self._ball_vel_source == "measured" and self._ball_frame != "base_link":
            self.ros_node.get_logger().warn(
                "[WalkKickBallVelHeadFreeEncDecAgent] ball_vel_source='measured' 只在 base_link 链路可用; "
                "camera 链路已强制回退到 'finite_diff'。"
            )
            self._ball_vel_source = "finite_diff"
        self._ball_base_detilt: bool = bool(ball_base_detilt)

        # ---- 球 obs: 两层, 无 CLI 开关, 阈值见类常量 BALL_* ----
        # 第 1 层 buffer: 一直填真球, 归 0 规则照抄训练 (只看距离/遮挡, 不看 kick 指令)
        # 第 2 层 门控  : 只有 kick 指令真正生效 (kick_speed != 0) 时才把 buffer 交给策略
        self._ball_active: bool = False          # 这一帧球位置是否有效 (距离/新鲜度)
        self._last_valid_ball_t: float = -1e9    # 最近一次拿到新视觉球的时刻
        self._ball_gate_is_open: bool = False    # 这一帧策略能不能看到球
        self._ball_gate_release_t: Optional[float] = None

        self.ort_sessions: dict = dict()
        self._parse_obs_config()
        self._parse_action_config()      # -> _act_sim_ids / head scale 补齐 / head 限位
        self._load_models()
        self._finalize_head_cfg()        # 需要 yaml 里的 head 限位, 必须在 parse 之后
        self._check_onnx_io()            # 装错模型 (22 维/维度不符) 在这里就炸, 不上机

        if self._head_lock:
            self._precompute_head_lock_actions()

    # ================================================================= #
    #  HeadFree 专属: 动作列解析 / head scale 补齐 / 维度自检
    # ================================================================= #
    def _parse_action_config(self) -> None:
        """在基类解析之上做三件 HeadFree 必须的事:

        1) 解出策略 20 维动作 -> sim 关节 id 的映射 (_act_sim_ids);
        2) 给 head 两列补一个非零 action_scale (yaml 里是 0, 会把 ColdStart 除炸);
        3) 从 yaml 读训练时 head 的活动范围/最大速度, 作为部署侧头部限位。
        """
        super()._parse_action_config()

        sim_names = list(self.ros_node.sim_joint_names)
        n_joints = len(sim_names)

        # ---- (1) 动作列 -> sim 关节 id ----
        act_cfg = None
        for _, c in self.cfg["actions"].items():
            if c.get("asset_name") == "robot":
                act_cfg = c
                break
        if act_cfg is None:
            raise RuntimeError("env.yaml 里没有 asset_name=='robot' 的 action term。")

        patterns = list(act_cfg.get("joint_names", []))
        if act_cfg.get("preserve_order", False):
            raise RuntimeError(
                "action term 的 preserve_order=True —— 本脚本按 sim 关节升序还原动作列, "
                "这种情况下列序由正则顺序决定, 会错位。请手工核对后再改这里。")
        self._act_sim_ids = resolve_matching_joint_ids(sim_names, patterns)
        if not self._act_sim_ids:
            # yaml 里正则没写清楚时的兜底: 按名字剔除 head
            self._act_sim_ids = [i for i, n in enumerate(sim_names) if "head" not in n]
            print("[HeadFree] ⚠ joint_names 正则没解出关节, 回退到 '名字里没有 head' 的规则。")

        self._head_sim_ids = [i for i in range(n_joints) if i not in set(self._act_sim_ids)]
        head_names = [sim_names[i] for i in self._head_sim_ids]
        if sorted(head_names) != sorted(HEAD_URDF_LIMITS.keys()):
            raise RuntimeError(
                f"[HeadFree] 动作空间之外的关节是 {head_names}, 期望恰好是两个 head 关节。"
                f" 你多半加载了非 HeadFree 的 run (action joint_names={patterns})。")

        self._act_sim_ids_np = np.asarray(self._act_sim_ids, dtype=np.int64)
        self.NUM_POLICY_ACTIONS = len(self._act_sim_ids)

        # ---- (2) head 的 action_scale 补齐 ----
        # yaml 的 actions.scale 里没有 head 的 key -> 基类解析出来是 0。
        # 0 会让 ColdStartAgent 的 (target-offset)/scale 变成 inf -> 下发 NaN 被 node 丢弃,
        # 冷启动会卡死在"发不出去"。这里补一个部署侧换算系数。
        self._head_action_scale = self._head_action_scale_req
        for i in self._head_sim_ids:
            if abs(float(self._action_scale[i])) > 1e-9:
                raise RuntimeError(
                    f"[HeadFree] env.yaml 里 {sim_names[i]} 的 action_scale="
                    f"{self._action_scale[i]:.4f} != 0 —— 说明 head 仍在动作空间里, "
                    f"这是 **非 HeadFree** 的 run。请改用 piplus_walk_kick_ballvel_perceptive.py。")
            self._action_scale[i] = self._head_action_scale

        # ---- (3) head 限位 / 限速: 以 env.yaml 为准 ----
        rng = act_cfg.get("head_pos_range", None)
        spd = act_cfg.get("head_max_speed", None)
        self._head_range = {}
        for n, (lo, hi) in HEAD_URDF_LIMITS.items():
            if isinstance(rng, dict) and n in rng:
                r = rng[n]
                self._head_range[n] = (float(r[0]), float(r[1]))
            else:
                self._head_range[n] = (lo * HEAD_RANGE_RATIO_FALLBACK,
                                       hi * HEAD_RANGE_RATIO_FALLBACK)
            # 无论如何不许超 URDF
            self._head_range[n] = (max(self._head_range[n][0], lo),
                                   min(self._head_range[n][1], hi))
        self._head_train_max_speed = float(spd) if spd is not None else HEAD_MAX_SPEED_FALLBACK
        self._head_range_from_yaml = isinstance(rng, dict)

        yaw_lo, yaw_hi = self._head_range["head_yaw_joint"]
        pit_lo, pit_hi = self._head_range["head_pitch_joint"]
        self._head_yaw_lo, self._head_yaw_hi = yaw_lo, yaw_hi
        self._head_pitch_lo, self._head_pitch_hi = pit_lo, pit_hi

        # ---- 打印对照表, 上机前人工扫一眼 ----
        print("[HeadFree] ===== policy action -> sim joint 映射 =====")
        for k, j in enumerate(self._act_sim_ids):
            print(f"[HeadFree]   act[{k:2d}] -> sim[{j:2d}] {sim_names[j]:<24s} "
                  f"scale={self._action_scale[j]:.4f} offset={self._action_offset[j]:+.3f} "
                  f"kp={self._p_gains[j]:.2f} kd={self._d_gains[j]:.3f}")
        for j in self._head_sim_ids:
            print(f"[HeadFree]   ( head ) sim[{j:2d}] {sim_names[j]:<24s} "
                  f"scale={self._action_scale[j]:.4f} (部署侧自定) "
                  f"kp={self._p_gains[j]:.2f} kd={self._d_gains[j]:.3f} "
                  f"range=[{self._head_range[sim_names[j]][0]:+.3f}, "
                  f"{self._head_range[sim_names[j]][1]:+.3f}] rad")
        print(f"[HeadFree]   action_dim={self.NUM_POLICY_ACTIONS}  joints={n_joints}  "
              f"head_pos_range 来源={'env.yaml' if self._head_range_from_yaml else '0.9xURDF 兜底'}  "
              f"训练 head_max_speed={self._head_train_max_speed:.2f} rad/s")
        print("[HeadFree] ============================================")

    def _finalize_head_cfg(self) -> None:
        """把 head 追踪的限位/限速夹进训练范围。必须在 _parse_action_config 之后调用。"""
        # 搜索姿态钳到训练 pitch 范围
        self._ht_search_pitch = float(
            np.clip(self._ht_search_pitch_req, self._head_pitch_lo, self._head_pitch_hi))
        if self._ht_search_pitch_req > self._head_pitch_hi + 1e-4:
            self.ros_node.get_logger().warn(
                f"[head] search_pitch {math.degrees(self._ht_search_pitch_req):.1f} deg 超出训练 "
                f"head_pitch 上限 ({math.degrees(self._head_pitch_hi):.1f} deg), 已钳到上限。"
                f" 球在脚边时头是够不到的, 球会往画面下沿跑。")
        # 追踪限速不许超训练时头部的最大速度 —— 超了就是身体策略没见过的头部惯量扰动
        if self._ht_max_rate > self._head_train_max_speed:
            self.ros_node.get_logger().warn(
                f"[head] head_track max_rate {self._ht_max_rate:.2f} rad/s "
                f"({math.degrees(self._ht_max_rate):.0f} dps) 超过训练 head_max_speed "
                f"{self._head_train_max_speed:.2f} rad/s, 已钳到训练值。"
                f" 头转得比训练里快 = 身体策略没见过的惯量扰动。")
            self._ht_max_rate = self._head_train_max_speed
        # 初始搜索姿态
        self._head_cmd_pitch = self._ht_search_pitch
        # zero_gain 模式下把 head 的 kp/kd 归零 (拷贝一份, 不动基类数组)
        if self._head_zero_gain:
            self._p_gains = np.array(self._p_gains, dtype=np.float32, copy=True)
            self._d_gains = np.array(self._d_gains, dtype=np.float32, copy=True)
            for i in self._head_sim_ids:
                self._p_gains[i] = 0.0
                self._d_gains[i] = 0.0
            self.ros_node.get_logger().warn(
                "[head] head_mode=zero_gain: head 的 kp/kd 已置 0, 目标角=当前实测角。"
                " ⚠ 只有当另一个节点自己下发 head 的 MotorControlCommand 时才有意义;"
                " 两个节点交替下发同一路电机会让 kp 在 0 和它的值之间跳变。")

    def _check_onnx_io(self) -> None:
        """核对 ONNX 的输入/输出维度。装错模型在这里炸, 不要等到第一帧下发。"""
        inp = self.ort_sessions["actor"].get_inputs()[0]
        out = self.ort_sessions["actor"].get_outputs()[0]

        def _last_dim(shape):
            if not shape:
                return None
            d = shape[-1]
            return int(d) if isinstance(d, int) else None

        self._onnx_in_dim = _last_dim(inp.shape)
        self._onnx_out_dim = _last_dim(out.shape)
        n_act = self.NUM_POLICY_ACTIONS
        if self._onnx_out_dim is not None and self._onnx_out_dim != n_act:
            hint = ""
            if self._onnx_out_dim == len(self.ros_node.sim_joint_names):
                hint = (" 这是 22 维的非 HeadFree 模型 —— 请改用 "
                        "piplus_walk_kick_ballvel_perceptive.py。")
            raise RuntimeError(
                f"[HeadFree] ONNX 输出 {self._onnx_out_dim} 维, 但 env.yaml 的动作空间是 "
                f"{n_act} 维。{hint}")
        self._obs_dim_checked = False
        print(f"[HeadFree] ONNX io: in={inp.shape} out={out.shape} "
              f"(动作 {n_act} 维 ✅)")

    def _check_obs_dim(self, obs: np.ndarray) -> None:
        """第一帧的 obs 维度自检 (ONNX 输入维度是静态时才有得比)。"""
        if self._onnx_in_dim is None or self._obs_dim_checked:
            self._obs_dim_checked = True
            return
        self._obs_dim_checked = True
        if obs.shape[0] != self._onnx_in_dim:
            hint = ""
            if obs.shape[0] - self._onnx_in_dim in (16, -16):
                hint = (" 差 16 = head 2 列 x 8 帧 last_action —— "
                        "HeadFree 模型 (650/630) 和普通 ballvel 模型 (666/646) 装反了。")
            elif obs.shape[0] - self._onnx_in_dim in (20, -20):
                hint = " 差 20 = ball_vel 一档 —— --posonly 和模型对不上。"
            raise RuntimeError(
                f"[HeadFree] obs 维度 {obs.shape[0]} != ONNX 输入 {self._onnx_in_dim}。{hint}")

    # ------------- model loading ------------- #
    def _load_models(self) -> None:
        ort_execution_providers = ort.get_available_providers()
        actor_path = self._actor_path_override or os.path.join(self.logdir, "exported", "actor.onnx")
        self.ort_sessions["actor"] = ort.InferenceSession(actor_path, providers=ort_execution_providers)
        print(f"[WalkKickBallVelHeadFreeEncDecAgent] Loaded ONNX actor (encoder+actor) from {actor_path}")

        normalizer_path = os.path.join(self.logdir, "exported", "policy_normalizer.npz")
        if os.path.exists(normalizer_path):
            self.normalizer = Normalizer(load_path=normalizer_path)
            print(f"[WalkKickBallVelHeadFreeEncDecAgent] Loaded normalizer from {normalizer_path}")
        else:
            self.normalizer = None
            print("[WalkKickBallVelHeadFreeEncDecAgent] No policy_normalizer.npz, running un-normalized")

    # ------------- lifecycle ------------- #
    def reset(self) -> None:
        self._kick_was_active = False
        self._soccer_hist = None
        self._clean_hist = None
        self._soccer_vel_hist = None
        self._soccer_valid_hist = None
        self._pub_counter = 0
        self._step_id = 0
        self._raw_policy_action = None
        self._ball_hist_step_id = -1
        self._last_cmd = None
        self._last_valid_ball_xy = None
        self._last_valid_ball_vel = None
        self._last_valid_ball_t = -1e9
        self._ball_gate_is_open = False
        self._ball_gate_release_t = None
        self._kick_dir_yaw_w = 0.0
        self._ball_active = False
        # head-track: start in search pose (look down / centered) until a ball is seen
        self._head_cmd_yaw = 0.0
        self._head_cmd_pitch = self._ht_search_pitch
        self._head_track_stale_t = self._ht_hold_grace
        self._des_yaw_f = None
        self._des_pitch_f = None
        self._yaw_track_active = False
        self._pitch_track_active = False
        self._head_hist.clear()
        super().reset()
        self.ros_node.get_logger().info(
            f"[WalkKickBallVelHeadFreeEncDecAgent] reset: "
            f"kick_speed={self._kick_speed_const:.2f} m/s, dir_mode={self._kick_dir_mode}; "
            f"posonly={self._posonly} "
            f"({'encoder 只吃球位置' if self._posonly else 'encoder 加球速'}); "
            f"ball_frame={self._ball_frame}, vel_source={self._ball_vel_source}, "
            f"vel_scale={self._ball_vel_scale:.2f}; "
            f"ball_hist={self._history_samples}x2 @ pub_period={self._pub_period} (10Hz async), "
            f"timeout={self._ball_topic_timeout:.2f}s; "
            f"ball buffer: 全程填真球 (far={self.BALL_FAR_MASK_DIST:.2f}m, "
            f"hold={self.BALL_HOLD_MAX_S:.2f}s); "
            f"latent 门控: kick_speed!=0 才接入策略 (release hold {self.BALL_GATE_RELEASE_HOLD_S:.2f}s); "
            f"action={self.NUM_POLICY_ACTIONS}维(无head), head_mode={self._head_mode} "
            f"(range yaw[{self._head_yaw_lo:+.2f},{self._head_yaw_hi:+.2f}] "
            f"pitch[{self._head_pitch_lo:+.2f},{self._head_pitch_hi:+.2f}] rad, "
            f"max_rate={self._ht_max_rate:.2f} rad/s <= 训练 {self._head_train_max_speed:.2f})"
        )

    def _precompute_head_lock_actions(self) -> None:
        scale = np.asarray(self.action_scale, dtype=np.float64)
        offset = np.asarray(self.action_offset, dtype=np.float64)
        eps = 1e-6

        # 锁定姿态也必须落在训练的 head_pos_range 内
        yaw_t = float(np.clip(self._head_yaw_target_rad, self._head_yaw_lo, self._head_yaw_hi))
        pit_t = float(np.clip(self._head_pitch_target_rad, self._head_pitch_lo, self._head_pitch_hi))
        if abs(yaw_t - self._head_yaw_target_rad) > 1e-4 or abs(pit_t - self._head_pitch_target_rad) > 1e-4:
            self.ros_node.get_logger().warn(
                f"[head_lock] 目标角超出训练 head_pos_range, 已钳位: "
                f"yaw {math.degrees(self._head_yaw_target_rad):+.1f}->{math.degrees(yaw_t):+.1f} deg, "
                f"pitch {math.degrees(self._head_pitch_target_rad):+.1f}->{math.degrees(pit_t):+.1f} deg")
        self._head_yaw_target_rad, self._head_pitch_target_rad = yaw_t, pit_t

        def invert(idx: int, theta: float, name: str) -> Optional[float]:
            s = float(scale[idx])
            if abs(s) < eps:
                self.ros_node.get_logger().warn(
                    f"[head_lock] action_scale[{name}]={s:.3g} ~ 0; head lock on {name} SKIPPED."
                )
                return None
            return (theta - float(offset[idx])) / s

        self._head_lock_action_yaw = invert(self._head_yaw_joint_idx, self._head_yaw_target_rad, "head_yaw_joint")
        self._head_lock_action_pitch = invert(self._head_pitch_joint_idx, self._head_pitch_target_rad, "head_pitch_joint")
        self.ros_node.get_logger().info(
            f"[head_lock] ENABLED: head_yaw={math.degrees(self._head_yaw_target_rad):+.1f} deg, "
            f"head_pitch={math.degrees(self._head_pitch_target_rad):+.1f} deg "
            f"action_yaw={self._head_lock_action_yaw}, action_pitch={self._head_lock_action_pitch}"
        )

    def _apply_head_lock(self, action: np.ndarray) -> np.ndarray:
        if not self._head_lock:
            return action
        if self._head_lock_action_yaw is not None:
            action[self._head_yaw_joint_idx] = self._head_lock_action_yaw
        if self._head_lock_action_pitch is not None:
            action[self._head_pitch_joint_idx] = self._head_lock_action_pitch
        return action

    # ------------- head ball-tracking (HeadFree: head 本来就归本脚本管) ------------- #
    #
    # Points the head at the ball -> absolute head yaw/pitch (torso frame) -> smoothed
    # (deadband + proportional ease-in + slew-rate cap + joint clamp). On ball loss: hold,
    # then slowly ease back to a downward search pose.
    #
    # Ball source depends on --ball_frame:
    #   camera    : optical ball -> torso(base) via head FK (same as camera obs path).
    #   base_link : vision already in torso frame -> use RAW xyz (NOT de-tilted; head joints
    #               move in the torso frame, so we want torso-frame ball, not yaw-base).
    #
    # ✅ 老版本那条 "head 覆盖值污染 last_action -> encoder OOD" 的 CAVEAT 在 HeadFree 下
    #    不成立: head 不在动作空间, last_action 只有 20 列, 头怎么动都进不了 encoder。
    #    训练时 head 本来就在 head_pos_range 内以 ≤head_max_speed 随机乱扫, 策略见过。
    #    ** 唯一的约束是别超出那个范围/速度 ** —— _finalize_head_cfg() 已经强制钳住。
    # CAVEAT (pitch vs z): base_link pitch target uses the ball z. If vision z is off, pitch
    # aim is off; yaw tracking (dx,dy only) stays robust.
    # CAVEAT (joint_pos obs): head 仍在 joint_pos/joint_vel obs 里 (22 列), 所以头的实际
    #    角度/速度是会进策略的 —— 这也正是训练里的情况, 保持一致即可。
    @staticmethod
    def _slew(cur: float, target: float, max_step: float) -> float:
        delta = target - cur
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        return cur + delta

    def _compute_head_track_targets(self) -> Optional[Tuple[float, float]]:
        """Absolute (yaw, pitch) rad to point the head at the ball, or None if no fresh ball.
        Uses the RAW torso(base_link)-frame ball position."""
        if self._ball_frame == "camera":
            cam = self._latest_ball_xyz_camera()
            if cam is None:
                return None
            ball_xyz_camera, cam_stamp = cam
            p_ball_base = self._ball_xyz_base_from_camera(ball_xyz_camera, stamp=cam_stamp)
        else:  # base_link: vision already torso-frame; raw xyz (NOT de-tilted)
            arr = self._latest_ball_base()
            if arr is None or arr.shape[0] < 3 or not np.isfinite(arr[2]):
                return None
            p_ball_base = np.array([arr[0], arr[1], arr[2]], dtype=np.float64)
        d = np.asarray(p_ball_base, dtype=np.float64).reshape(3) - HEAD_YAW_POS_BASE
        dx, dy, dz = float(d[0]), float(d[1]), float(d[2])
        horiz = math.hypot(dx, dy)
        if horiz < 1e-4:
            return None
        desired_yaw = math.atan2(dy, dx) + self._ht_yaw_trim          # +left  (about base +z)
        desired_pitch = math.atan2(-dz, horiz) + self._ht_pitch_trim  # +down  (ball below -> +)
        return desired_yaw, desired_pitch

    def _apply_head_track(self, action: np.ndarray) -> np.ndarray:
        dt = MAIN_LOOP_DT
        tgt = self._compute_head_track_targets()

        if tgt is not None:
            self._head_track_stale_t = 0.0
            des_yaw, des_pitch = tgt

            # EMA-smooth the target to reject per-frame vision noise
            a = self._ht_smooth
            self._des_yaw_f = des_yaw if self._des_yaw_f is None else (1 - a) * self._des_yaw_f + a * des_yaw
            self._des_pitch_f = des_pitch if self._des_pitch_f is None else (1 - a) * self._des_pitch_f + a * des_pitch
            des_yaw, des_pitch = self._des_yaw_f, self._des_pitch_f

            # Bearing error vs the ACTUAL head angle (how far the ball is off image center).
            actual_yaw = float(self.ros_node.joint_pos_[self._head_yaw_joint_idx])
            actual_pitch = float(self.ros_node.joint_pos_[self._head_pitch_joint_idx])

            # (1) Clamp the pitch TARGET to the joint limit before measuring the error.
            #     A ball near the feet needs ~52 deg of look-down but the joint stops at
            #     训练 head_pitch 上限 (~45 deg), so the raw error can never reach zero —
            #     the latch would read "not aimed yet" forever even though the head is
            #     already doing the best it physically can. Clamping does NOT change where
            #     the head points (_head_cmd_pitch is clipped below anyway), only the error
            #     used by the latch.
            des_pitch_c = float(np.clip(des_pitch, self._head_pitch_lo, self._head_pitch_hi))

            # (2) Independent hysteresis latch per axis. A single max(yaw, pitch) latch lets
            #     a saturated pitch error unfreeze the YAW loop too; yaw then chases delayed
            #     (~100-150 ms) noisy vision at kp per step -> visible left/right sway with a
            #     static robot and a static ball.
            be_yaw = abs(des_yaw - actual_yaw)
            be_pitch = abs(des_pitch_c - actual_pitch)
            if be_yaw < self._ht_deadband:
                self._yaw_track_active = False
            elif be_yaw > self._ht_reacquire:
                self._yaw_track_active = True
            if be_pitch < self._ht_deadband:
                self._pitch_track_active = False
            elif be_pitch > self._ht_reacquire:
                self._pitch_track_active = True

            max_step = self._ht_max_rate * dt
            if self._yaw_track_active:
                self._head_cmd_yaw = self._slew(
                    self._head_cmd_yaw, self._head_cmd_yaw + self._ht_kp * (des_yaw - self._head_cmd_yaw), max_step)
            if self._pitch_track_active:
                self._head_cmd_pitch = self._slew(
                    self._head_cmd_pitch, self._head_cmd_pitch + self._ht_kp * (des_pitch_c - self._head_cmd_pitch), max_step)
            # else (per axis): ball centered on that axis -> HOLD frozen -> no jitter
        else:
            # ball not fresh: hold last aim for a grace period, then ease to search pose
            self._des_yaw_f = None
            self._des_pitch_f = None
            self._yaw_track_active = False
            self._pitch_track_active = False
            self._head_track_stale_t += dt
            if self._head_track_stale_t >= self._ht_hold_grace:
                s = self._ht_search_rate * dt
                self._head_cmd_yaw = self._slew(self._head_cmd_yaw, 0.0, s)
                self._head_cmd_pitch = self._slew(self._head_cmd_pitch, self._ht_search_pitch, s)

        # clamp to URDF joint limits (with margin)
        self._head_cmd_yaw = float(np.clip(self._head_cmd_yaw, self._head_yaw_lo, self._head_yaw_hi))
        self._head_cmd_pitch = float(
            np.clip(self._head_cmd_pitch, self._head_pitch_lo, self._head_pitch_hi))

        # write ONLY the two head joints (target angle -> action via inverse scale/offset)
        scale = np.asarray(self.action_scale, dtype=np.float64)
        offset = np.asarray(self.action_offset, dtype=np.float64)
        sy = float(scale[self._head_yaw_joint_idx])
        sp = float(scale[self._head_pitch_joint_idx])
        if abs(sy) > 1e-6:
            action[self._head_yaw_joint_idx] = (
                self._head_cmd_yaw - float(offset[self._head_yaw_joint_idx])) / sy
        if abs(sp) > 1e-6:
            action[self._head_pitch_joint_idx] = (
                self._head_cmd_pitch - float(offset[self._head_pitch_joint_idx])) / sp
        return action

    # ------------- one inference step ------------- #
    def step(self) -> Tuple[np.ndarray, bool]:
        self._step_id += 1
        self._sample_head_hist()      # 必须在读球 obs 之前, 保证本步的头角度已入缓冲
        obs = self._get_observation()
        self._check_obs_dim(obs)      # 第一帧自检: 650/630 vs 666/646 装反了当场炸
        if self.normalizer is not None:
            normalized_obs = self.normalizer.normalize(obs).astype(np.float32)[None, :]
        else:
            normalized_obs = obs.astype(np.float32)[None, :]

        actor_in = self.ort_sessions["actor"].get_inputs()[0].name
        policy_action = self.ort_sessions["actor"].run(None, {actor_in: normalized_obs})[0]
        policy_action = np.asarray(policy_action, dtype=np.float32).reshape(-1)

        if policy_action.shape[0] != self.NUM_POLICY_ACTIONS:
            raise RuntimeError(
                f"[HeadFree] ONNX 这一帧输出 {policy_action.shape[0]} 维, "
                f"期望 {self.NUM_POLICY_ACTIONS} 维。")

        # 给 node 当 last_action obs 的就是这 20 维 (训练里 mdp.last_action 也是 20 维)
        self._raw_policy_action = policy_action.copy()

        # ---- 20 维 -> 22 维 sim 关节动作向量 ----
        # 未被策略控制的两列 (head) 先填"保持默认姿态"的 0, 再由 head 控制器覆盖。
        # 注意不能留 0 不管: 0 -> target = offset = 默认角度, 头会直勾勾看正前方,
        # 视觉就永远找不到脚边的球。
        action = np.zeros(len(self.ros_node.sim_joint_names), dtype=np.float32)
        action[self._act_sim_ids_np] = policy_action

        if self._head_track:
            action = self._apply_head_track(action)
        elif self._head_lock:
            action = self._apply_head_lock(action)
        else:  # zero_gain: 目标角 = 当前实测角 (kp/kd 已经是 0, 这只是让指令无害)
            action = self._apply_head_hold(action)
        return action, False

    def _apply_head_hold(self, action: np.ndarray) -> np.ndarray:
        """zero_gain 模式: head 目标 = 当前实测角度。配合 kp=kd=0 -> 本节点对头不施力。"""
        jp = self.ros_node.joint_pos_
        scale = np.asarray(self.action_scale, dtype=np.float64)
        offset = np.asarray(self.action_offset, dtype=np.float64)
        for idx in self._head_sim_ids:
            s = float(scale[idx])
            if abs(s) > 1e-6:
                action[idx] = (float(jp[idx]) - float(offset[idx])) / s
        return action

    # ------------- kick anchoring ------------- #
    def _current_robot_yaw_rel(self) -> float:
        q = self.ros_node._get_quat_w_obs()
        return _wrap_to_pi(_yaw_from_quat_wxyz(q) - self._robot_yaw_w_at_anchor)

    def _anchor_kick(self) -> None:
        q = self.ros_node._get_quat_w_obs()
        self._robot_yaw_w_at_anchor = _yaw_from_quat_wxyz(q)
        self._kick_dir_yaw_w = 0.0
        self.ros_node.get_logger().info(
            f"[WalkKickBallVelHeadFreeEncDecAgent] kick anchored: yaw_w={self._robot_yaw_w_at_anchor:+.3f} rad "
            f"(mode={self._kick_dir_mode})"
        )

    # ------------- 球坐标: camera 链路 (optical -> base FK -> yaw detilt) ------------- #
    def _latest_ball_xyz_camera(self) -> Optional[Tuple[np.ndarray, Optional[float]]]:
        """返回 (xyz_camera, stamp)。stamp 可能为 None (老 node 没有带 stamp 的 getter)。"""
        g2 = getattr(self.ros_node, "get_latest_ball_position_camera_with_stamp", None)
        if g2 is not None:
            r = g2(max_age_s=self._ball_topic_timeout)
            return None if r is None else (r[0], r[1])
        getter = getattr(self.ros_node, "get_latest_ball_position_camera", None)
        if getter is None:
            return None
        xyz = getter(max_age_s=self._ball_topic_timeout)
        return None if xyz is None else (xyz, None)

    # ------------- 头部角度历史 (按图像时间戳回溯用) ------------- #
    def _sample_head_hist(self) -> None:
        """每个 control step 存一份 (t, yaw, pitch)。t 必须和球话题 stamp 用同一个时钟。"""
        try:
            t = self.ros_node.get_clock().now().nanoseconds * 1e-9
            jp = self.ros_node.joint_pos_
            self._head_hist.append((t,
                                    float(jp[self._head_yaw_joint_idx]),
                                    float(jp[self._head_pitch_joint_idx])))
        except Exception:
            pass

    def _head_angles_at(self, t_query: float) -> Optional[Tuple[float, float]]:
        """线性插值出 t_query 时刻的 (head_yaw, head_pitch)。缓冲为空返回 None,
        超出缓冲范围钳到最近端点 (不外推)。"""
        h = self._head_hist
        if not h:
            return None
        if t_query <= h[0][0]:
            return h[0][1], h[0][2]
        if t_query >= h[-1][0]:
            return h[-1][1], h[-1][2]
        for i in range(len(h) - 1, 0, -1):
            t1, y1, p1 = h[i]
            t0, y0, p0 = h[i - 1]
            if t0 <= t_query <= t1:
                w = (t_query - t0) / max(t1 - t0, 1e-9)
                return y0 + w * (y1 - y0), p0 + w * (p1 - p0)
        return h[-1][1], h[-1][2]

    def _ball_xyz_base_from_camera(self, ball_xyz_camera: np.ndarray,
                                   stamp: Optional[float] = None) -> np.ndarray:
        """相机 FLU -> base(torso) 系。

        ** stamp 必须传 **: 相机测到的是球相对光轴的偏角 β = φ - θ(t-τ) (φ=球真实方位角,
        θ=头部角度, τ=管线延迟)。如果用 **当前** 头角度 θ(t) 去还原, 得到的是
            φ + [θ(t) - θ(t-τ)] ≈ φ + τ·θ̇
        —— 头一动, 重建出来的球就朝同方向跑掉 τ·θ̇。后果有两个:
          1) 喂 encoder 的球位置在头动时是假的 (实测静止球被重建成摆 0.65m、1.9m/s)
          2) head_track 去追这个假目标 -> 环路里出现纯延迟
             ċmd = (φ - cmd(t-τ))/T,  T = dt/kp = 57ms @kp=0.35
             失稳判据 τ/T > π/2 -> τ > 90ms 即不稳 (实测头 12.8% 时间打满限速)
        按时间戳回溯头部角度后 des ≡ φ 恒等于常数, 反馈路径里不再有延迟,
        稳定性与 τ 无关。这不是补偿延迟, 是把重建做对。

        NOTE: _detilt_base_to_yaw 用的仍是当前 IMU 四元数, 那一路没有做时间对齐
              (和改动前一致)。躯干角速度大时那一段 τ·ω 误差仍在。
        """
        head_yaw = head_pitch = None
        if stamp is not None:
            hp = self._head_angles_at(float(stamp) - PIPELINE_LATENCY_S)
            if hp is not None:
                head_yaw, head_pitch = hp
        if head_yaw is None:                      # 兜底: 缓冲还没建起来 / 没给 stamp
            joint_pos = self.ros_node.joint_pos_
            head_yaw = float(joint_pos[self._head_yaw_joint_idx])
            head_pitch = float(joint_pos[self._head_pitch_joint_idx])

        r_base_head_yaw = _rot_z(head_yaw)
        r_base_head_pitch = r_base_head_yaw @ _rot_y(head_pitch)
        p_base_head_pitch = HEAD_YAW_POS_BASE + r_base_head_yaw @ HEAD_PITCH_POS_HEAD_YAW
        p_base_camera_link = p_base_head_pitch + r_base_head_pitch @ CAMERA_LINK_POS_HEAD_PITCH
        p_base_left_eye = p_base_camera_link + r_base_head_pitch @ LEFT_EYE_POS_CAMERA_LINK
        r_base_left_eye_input = r_base_head_pitch @ R_CAMERA_LINK_BALL_INPUT

        ball_xyz_camera = np.asarray(ball_xyz_camera, dtype=np.float64).reshape(3)
        return p_base_left_eye + r_base_left_eye_input @ ball_xyz_camera

    def _detilt_base_to_yaw(self, ball_xyz_base: np.ndarray) -> np.ndarray:
        q = np.asarray(self.ros_node._get_quat_w_obs(), dtype=np.float64).reshape(4)  # wxyz
        yaw = _yaw_from_quat_wxyz(q)
        q_yaw = np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)], dtype=np.float64)
        t = _quat_mul(_quat_conj(q_yaw), q)
        return _quat_apply(t, ball_xyz_base)

    # ------------- 球坐标: base_link 链路 (视觉已 base 系) ------------- #
    def _latest_ball_base(self) -> Optional[np.ndarray]:
        """返回 [x, y, z, vx, vy, vz] (base 系, m / m·s⁻¹); 仅位置时 vx/vy/vz = nan; 无数据 None。"""
        getter = getattr(self.ros_node, "get_latest_ball_base", None)
        if getter is None:
            return None
        return getter(max_age_s=self._ball_topic_timeout)

    # ------------- 统一读球位置 (yaw-base, hold-last) ------------- #
    def _read_ball_xy(self) -> Tuple[float, float]:
        if self._ball_frame == "camera":
            cam = self._latest_ball_xyz_camera()
            if cam is not None:
                xyz_cam, cam_stamp = cam
                xyz_yaw = self._detilt_base_to_yaw(
                    self._ball_xyz_base_from_camera(xyz_cam, stamp=cam_stamp))
                bx, by = float(xyz_yaw[0]), float(xyz_yaw[1])
                self._last_valid_ball_xy = (bx, by)
                self._last_valid_ball_t = time.time()
                return bx, by
        else:  # base_link
            arr = self._latest_ball_base()
            if arr is not None and arr.shape[0] >= 2:
                bx, by = float(arr[0]), float(arr[1])
                bz = float(arr[2]) if (arr.shape[0] >= 3 and np.isfinite(arr[2])) else 0.0
                if self._ball_base_detilt:
                    v = self._detilt_base_to_yaw(np.array([bx, by, bz], dtype=np.float64))
                    bx, by = float(v[0]), float(v[1])
                self._last_valid_ball_xy = (bx, by)
                self._last_valid_ball_t = time.time()
                return bx, by
        if self._last_valid_ball_xy is not None:
            return self._last_valid_ball_xy
        return 0.0, 0.0

    # ------------- 距离 mask 后的球位置 (喂 obs 用的唯一入口) ------------- #
    def _read_ball_xy_masked(self) -> Tuple[float, float, bool]:
        """返回 (x, y, valid)。valid=False 时 x=y=0。

        归 0 规则 (和训练 ball_pos_b_noisy_hist / ball_vel_b_noisy_hist_async 一致):
          1) 从来没见过球                        -> 0
          2) 距离 > BALL_FAR_MASK_DIST           -> 0   (训练 ball_far_mask_dist=1.2)
          3) hold-last 超过 BALL_HOLD_MAX_S      -> 0   (训练 ball_mask_run_window:
             连续遮挡超过 encoder 历史窗后, buffer 已全 0, 目标也归 0)
        ** 与 A 键无关 ** —— 训练里球的 mask 从不看 kick 指令。
        """
        bx, by = self._read_ball_xy()
        if self._last_valid_ball_xy is None:
            return 0.0, 0.0, False
        if self.BALL_HOLD_MAX_S > 0.0 and \
                (time.time() - self._last_valid_ball_t) > self.BALL_HOLD_MAX_S:
            return 0.0, 0.0, False
        if self.BALL_FAR_MASK_DIST > 0.0 and \
                math.hypot(bx, by) > self.BALL_FAR_MASK_DIST:
            return 0.0, 0.0, False
        return bx, by, True

    # ------------- 统一读测量球速 (yaw-base, hold-last); 仅 base_link+measured ------------- #
    def _read_ball_vel(self) -> Tuple[float, float]:
        arr = self._latest_ball_base()
        if arr is not None and arr.shape[0] >= 6 and np.isfinite(arr[3]) and np.isfinite(arr[4]):
            vx, vy = float(arr[3]), float(arr[4])
            vz = float(arr[5]) if np.isfinite(arr[5]) else 0.0
            if self._ball_base_detilt:
                v = self._detilt_base_to_yaw(np.array([vx, vy, vz], dtype=np.float64))
                vx, vy = float(v[0]), float(v[1])
            self._last_valid_ball_vel = (vx, vy)
            return vx, vy
        if self._last_valid_ball_vel is not None:
            return self._last_valid_ball_vel
        return 0.0, 0.0

    # ------------- 7-D soccer command (walk / kick) ------------- #
    def _build_soccer_cmd(self) -> np.ndarray:
        joy = self.ros_node.joy_stick_data
        lx = float(joy.lx or 0.0)
        ly = float(joy.ly or 0.0)
        rx = float(joy.rx or 0.0)
        a_pressed = bool(getattr(joy, "A", False))

        # ---- 第 1 层: buffer 里写什么 ----
        # 一直写距离 mask 后的真球, 走路/踢球都写, 和 A 键无关 (训练就是这样)。
        ball_x_b, ball_y_b, ball_ok = self._read_ball_xy_masked()
        self._ball_active = ball_ok

        if a_pressed:
            # ---------------- KICK ----------------
            if not self._kick_was_active:
                self._anchor_kick()
            self._kick_was_active = True

            vx, vy, wz = 0.0, 0.0, 0.0
            yaw_rel = self._current_robot_yaw_rel()

            if lx * lx + ly * ly >= self._LSTICK_DEADZONE_SQ:
                vx_intent = ly * self.x_vel_scale
                vy_intent = -lx * self.y_vel_scale
                stick_dir = math.atan2(vy_intent, vx_intent)
                if self._kick_dir_mode == "anchor":
                    self._kick_dir_yaw_w = stick_dir
                else:  # follow
                    self._kick_dir_yaw_w = _wrap_to_pi(yaw_rel + stick_dir)
                kick_speed = self._kick_speed_const
            else:
                kick_speed = 0.0
            kick_dir_b = _wrap_to_pi(self._kick_dir_yaw_w - yaw_rel)
        else:
            # ---------------- WALK / IDLE ----------------
            self._kick_was_active = False
            vx = ly * self.x_vel_scale
            vy = -lx * self.y_vel_scale
            wz = -rx * self.yaw_vel_scale
            kick_dir_b = 0.0
            kick_speed = 0.0

        # ---- 第 2 层: 这一帧要不要把 buffer 交给策略 ----
        # 开门条件 = kick 指令真正生效 (按住 A 且左摇杆出死区 -> kick_speed != 0)。
        # 不测 kick_dir: kick_dir_b == 0 是"朝正前方踢", 是合法指令, 不能拿来当关门条件。
        # 松手后再保持 BALL_GATE_RELEASE_HOLD_S, 免得挥腿到一半 latent 被抽走。
        if kick_speed != 0.0:
            self._ball_gate_release_t = None
            gate = True
        else:
            if self._ball_gate_is_open and self._ball_gate_release_t is None:
                self._ball_gate_release_t = time.time()
            gate = (self._ball_gate_release_t is not None and
                    (time.time() - self._ball_gate_release_t) < self.BALL_GATE_RELEASE_HOLD_S)
            if not gate:
                self._ball_gate_release_t = None
        self._ball_gate_is_open = bool(gate)

        return np.array(
            [vx, vy, wz, ball_x_b, ball_y_b, kick_dir_b, kick_speed],
            dtype=np.float32,
        )

    # ------------- 历史刷新: 命令 (10Hz async + 50Hz clean) + 测量球速 async ------------- #
    def _ensure_ball_history_fresh(self) -> None:
        if self._ball_hist_step_id == self._step_id and self._soccer_hist is not None:
            return

        cmd = self._build_soccer_cmd()
        self._last_cmd = cmd

        # 这一帧的球位置是否有效 (由 _build_soccer_cmd 里的距离 mask 决定)
        valid_now = bool(self._ball_active)

        use_measured = (not self._posonly) and (self._ball_vel_source == "measured")
        if use_measured:
            # 球位置有效才喂测量球速; 被 mask 的帧 vel 写 0 ——
            # 和训练 ball_vel_b_noisy_hist_async 的一致性规则完全相同
            # ("command 球位置被 mask 成 0 时, 该帧球速也置 0 且不加噪")。
            vel_now = np.array(self._read_ball_vel(), dtype=np.float32) if valid_now \
                else np.zeros(2, dtype=np.float32)
        else:
            vel_now = None

        if self._soccer_hist is None:
            self._soccer_hist = np.tile(cmd, (self._history_samples, 1)).astype(np.float32)
            self._clean_hist = np.tile(cmd, (self._history_samples, 1)).astype(np.float32)
            self._soccer_vel_hist = np.zeros((self._history_samples, 2), dtype=np.float32)
            self._soccer_valid_hist = np.full(self._history_samples, valid_now, dtype=bool)
            if vel_now is not None:
                self._soccer_vel_hist[:] = vel_now
            self._pub_counter = 0

        # 50Hz clean (soccer_command 用)
        self._clean_hist[:-1] = self._clean_hist[1:]
        self._clean_hist[-1] = cmd

        # 10Hz async (ball_pos / ball_vel encoder 输入用)
        if self._pub_counter % self._pub_period == 0:
            self._soccer_hist[:-1] = self._soccer_hist[1:]
            self._soccer_hist[-1] = cmd
            self._soccer_valid_hist[:-1] = self._soccer_valid_hist[1:]
            self._soccer_valid_hist[-1] = valid_now
            if vel_now is not None:
                self._soccer_vel_hist[:-1] = self._soccer_vel_hist[1:]
                self._soccer_vel_hist[-1] = vel_now
        self._pub_counter += 1

        self._ball_hist_step_id = self._step_id

    # ------------- obs getters (方法名匹配 env.yaml 的 obs func) ------------- #
    def _get_soccer_command_async_obs(self) -> np.ndarray:
        self._ensure_ball_history_fresh()
        out = self._clean_hist.copy()
        out = np.delete(out, [3, 4], axis=1)   # 删球位置列 -> (H, 5): [vx,vy,wz,kick_dir,kick_spd]
        return out.reshape(-1)

    def _get_ball_pos_b_noisy_hist_obs(self) -> np.ndarray:
        self._ensure_ball_history_fresh()
        if not self._ball_gate_is_open:
            # 门关: 交给策略全 0 -> encoder 输出训练里"球在 far_mask 外"的 latent。
            # 注意 buffer 本身没被清空, 一直在滚真球, 所以门一开就是满窗。
            return np.zeros(self._history_samples * 2, dtype=np.float32)
        return self._soccer_hist[:, 3:5].reshape(-1).copy()

    def _get_ball_vel_b_noisy_hist_async_obs(self) -> np.ndarray:
        """encoder 球速输入 (仅 non-posonly 模型的 env.yaml 会调)。
        measured    : 测量球速 async 历史 (yaw-base) * vel_scale。
        finite_diff : 球位置 async 历史有限差分 (yaw-base) * vel_scale。
        球被 mask (太远 / hold 过期 / 没见过) 的帧: vel 一律 0, 且 finite_diff **不跨越**
        mask 边界做差分 —— 否则 0 -> 0.36m 的跳变会算出 (0.36-0)/0.1 = 3.6 m/s 的假球速,
        并在 encoder 窗口里滞留整整一个窗口长度。训练侧喂的是真实球速 + 被 mask 帧写 0,
        从来不会出现这种边界差分尖峰。"""
        self._ensure_ball_history_fresh()
        if not self._ball_gate_is_open:
            return np.zeros(self._history_samples * 2, dtype=np.float32)
        valid = self._soccer_valid_hist                          # (H,) bool
        if self._ball_vel_source == "measured":
            vel = self._soccer_vel_hist.copy()
        else:
            pos = self._soccer_hist[:, 3:5]                     # (H, 2) yaw-base async
            dt = max(int(self._pub_period), 1) * MAIN_LOOP_DT
            vel = np.zeros_like(pos)
            # 只有相邻两帧都有效才做差分; 任一端被 mask -> 该帧 vel = 0
            pair_ok = np.logical_and(valid[1:], valid[:-1])[:, None]
            vel[1:] = np.where(pair_ok, (pos[1:] - pos[:-1]) / dt, 0.0)
        # 兜底: 位置被 mask 的帧 vel 必须是 0 (measured 分支同样适用)
        vel = np.where(valid[:, None], vel, 0.0)
        vel = vel * self._ball_vel_scale
        return vel.reshape(-1).astype(np.float32)

    def _get_imu_ang_vel_obs(self) -> np.ndarray:
        return self.ros_node._get_base_ang_vel_obs()

    def _get_imu_projected_gravity_obs(self) -> np.ndarray:
        return self.ros_node._get_projected_gravity_obs()


# --------------------------------------------------------------------------- #
# Agent factory (obs-config patch: 去掉 policy.concatenate_dim)
# --------------------------------------------------------------------------- #
def _create_walk_kick_ballvel_agent(logdir, actor_path, ros_node, **kwargs):
    from instinct_onboard.agents import base as _base

    _orig_parse = _base.OnboardAgent._parse_obs_config

    def _patched_parse(self):
        self.cfg["observations"]["policy"].pop("concatenate_dim", None)
        return _orig_parse(self)

    _base.OnboardAgent._parse_obs_config = _patched_parse
    try:
        agent = WalkKickBallVelHeadFreeEncDecAgent(
            logdir=logdir, ros_node=ros_node, actor_path=actor_path, **kwargs
        )
    finally:
        _base.OnboardAgent._parse_obs_config = _orig_parse
    return agent


# --------------------------------------------------------------------------- #
# ROS node: cold_start <-> soccer, camera + base_link 两套球订阅
# --------------------------------------------------------------------------- #
class PiPlusWalkKickBallVelHeadFreeEncDecNode(HightorqueNode):
    """State machine:
        cold_start --[R1, when done]--> soccer
        soccer     --[R1]--> re-reset soccer (clears obs history)

    HeadFree 差异: 喂策略的 last_action obs 是 **20 维** (无 head), 而发给电机的
    action 仍是 22 维。两者的映射由 soccer agent 的 _act_sim_ids 决定, 在
    register_agent() 时接过来。
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.available_agents: dict = dict()
        self.current_agent_name: Optional[str] = None

        # last_action obs 用的独立缓冲。RealNode.send_action 里的 self.action 保持不变
        # (仍是发给电机 / 发到 /raw_actions 的那份 22 维), 这里只影响喂给策略的观测。
        # 长度和索引在 register_agent("soccer", ...) 里按 agent 的动作空间重建;
        # 这里先按"名字里没有 head"给个占位, 免得 soccer 还没注册就被 obs 调到。
        try:
            self._act_sim_ids_np: np.ndarray = np.asarray(
                [i for i, n in enumerate(self.sim_joint_names) if "head" not in n], dtype=np.int64)
        except Exception:
            self._act_sim_ids_np = np.arange(self.NUM_ACTIONS, dtype=np.int64)
        self._last_action_for_obs = np.zeros(self._act_sim_ids_np.shape[0], dtype=np.float32)
        # 老开关, HeadFree 下无意义 (head 不在动作空间, 没有"覆盖值"可言), 留着不报错
        self._head_obs_passthrough: bool = True
        self.raw_policy_action_publisher = self.create_publisher(
            Float32MultiArray, "/policy_raw_actions", 10)

        self._ball_frame: str = "camera"

        # camera 链路
        self._ball_position_camera_topic: str = DEFAULT_BALL_POSITION_CAMERA_TOPIC
        self._ball_position_camera_msg_type: str = "PointStamped"
        self._ball_position_camera: Optional[np.ndarray] = None
        self._ball_position_camera_stamp: Optional[float] = None

        # base_link 链路
        self._ball_base_topic: str = DEFAULT_BALL_BASE_TOPIC
        self._ball_base_msg_type: str = "Float32MultiArray"
        self._ball_base: Optional[np.ndarray] = None          # [x, y, vx, vy]
        self._ball_base_stamp: Optional[float] = None

    def _get_imu_ang_vel_obs(self):
        return self._get_base_ang_vel_obs()

    def _get_imu_orientation_obs(self):
        return self._get_quat_w_obs()

    def _get_imu_projected_gravity_obs(self):
        return self._get_projected_gravity_obs()

    # ---- 覆盖 RealNode._get_last_action_obs ----
    # HeadFree: 返回 **20 维** (无 head), 和训练里 mdp.last_action 的宽度一致。
    # 老版本的 --head_obs_overridden 在这里没有意义了 —— head 压根不在动作空间。
    def _get_last_action_obs(self):
        return self._last_action_for_obs

    def register_agent(self, name: str, agent) -> None:
        self.available_agents[name] = agent
        # soccer agent 解析出的动作列映射是权威的, 接过来给 last_action obs 用
        ids = getattr(agent, "_act_sim_ids_np", None)
        if ids is not None:
            self._act_sim_ids_np = np.asarray(ids, dtype=np.int64)
            if self._last_action_for_obs.shape[0] != self._act_sim_ids_np.shape[0]:
                self._last_action_for_obs = np.zeros(
                    self._act_sim_ids_np.shape[0], dtype=np.float32)
            self.get_logger().info(
                f"[HeadFree] last_action obs 宽度 = {self._act_sim_ids_np.shape[0]} "
                f"(电机动作仍是 {self.NUM_ACTIONS} 维)")

    def set_ball_config(self, frame: str, camera_topic: str, base_topic: str, base_msg_type: str,
                        camera_msg_type: str = "PointStamped") -> None:
        self._ball_frame = str(frame).lower()
        self._ball_position_camera_topic = str(camera_topic)
        self._ball_position_camera_msg_type = str(camera_msg_type)
        self._ball_base_topic = str(base_topic)
        self._ball_base_msg_type = str(base_msg_type)

    # ---- camera 链路: PointStamped (相机 FLU) ----
    def _ball_position_camera_callback(self, msg: PointStamped) -> None:
        self._ball_position_camera = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self._ball_position_camera_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        else:
            self._ball_position_camera_stamp = self.get_clock().now().nanoseconds * 1e-9

    # ---- camera 链路: Odometry (ball_vision_ros2 的左目话题; 只取 pose.position, twist 丢弃) ----
    def _ball_position_camera_callback_odometry(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._ball_position_camera = np.array([p.x, p.y, p.z], dtype=np.float32)  # 相机 FLU 位置; 速度靠 finite_diff
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self._ball_position_camera_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        else:
            self._ball_position_camera_stamp = self.get_clock().now().nanoseconds * 1e-9

    def get_latest_ball_position_camera_with_stamp(self, max_age_s: float = 0.2):
        """返回 (xyz, stamp) 或 None。stamp 用于按图像时间戳回溯头部角度。"""
        if self._ball_position_camera is None or self._ball_position_camera_stamp is None:
            return None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._ball_position_camera_stamp > max_age_s:
            return None
        return self._ball_position_camera.copy(), float(self._ball_position_camera_stamp)

    def get_latest_ball_position_camera(self, max_age_s: float = 0.2) -> Optional[np.ndarray]:
        if self._ball_position_camera is None or self._ball_position_camera_stamp is None:
            return None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._ball_position_camera_stamp > max_age_s:
            return None
        return self._ball_position_camera.copy()

    # ---- base_link 链路: Float32MultiArray[x,y,vx,vy] 或 PointStamped ----
    def _ball_base_callback_multiarray(self, msg: Float32MultiArray) -> None:
        d = np.asarray(msg.data, dtype=np.float32).reshape(-1)
        # 统一 6 维 schema [x, y, z, vx, vy, vz]。multiarray 约定 data=[x,y,(vx,vy)]:
        # 无 z 分量 -> z=0; 无速度 (len<4) -> vx/vy/vz = nan (视为仅位置)。
        out = np.full(6, np.nan, dtype=np.float32)
        if d.shape[0] >= 1:
            out[0] = d[0]                                 # x
        if d.shape[0] >= 2:
            out[1] = d[1]                                 # y
        out[2] = 0.0                                      # z (multiarray 无 z)
        if d.shape[0] >= 4:
            out[3] = d[2]                                 # vx
            out[4] = d[3]                                 # vy
            out[5] = 0.0                                  # vz (multiarray 无 z 速度)
        self._ball_base = out
        self._ball_base_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _ball_base_callback_pointstamped(self, msg: PointStamped) -> None:
        # 6 维 schema: base_link PointStamped 带 z, 位置 xyz 全收; 无速度 -> nan。
        out = np.array([msg.point.x, msg.point.y, msg.point.z,
                        np.nan, np.nan, np.nan], dtype=np.float32)
        self._ball_base = out
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self._ball_base_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        else:
            self._ball_base_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _ball_base_callback_odometry(self, msg: Odometry) -> None:
        # nav_msgs/Odometry: pose.position.xyz = ball pos, twist.linear.xyz = ball vel (base_link)。
        # 6 维 schema [x,y,z,vx,vy,vz]: 带上真实 position.z, 让 base_link -> yaw-base 的 de-tilt
        # (--ball_base_detilt) 用真 z, 不再拍平为 0。z 若不准会经 pitch 串进 xy, 需核视觉 z。
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._ball_base = np.array([p.x, p.y, p.z, v.x, v.y, v.z], dtype=np.float32)
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            self._ball_base_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        else:
            self._ball_base_stamp = self.get_clock().now().nanoseconds * 1e-9

    def get_latest_ball_base(self, max_age_s: float = 0.2) -> Optional[np.ndarray]:
        if self._ball_base is None or self._ball_base_stamp is None:
            return None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._ball_base_stamp > max_age_s:
            return None
        return self._ball_base.copy()

    def start_ros_handlers(self) -> None:
        super().start_ros_handlers()
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self._ball_frame == "camera":
            if self._ball_position_camera_msg_type == "Odometry":
                self.ball_subscriber = self.create_subscription(
                    Odometry, self._ball_position_camera_topic,
                    self._ball_position_camera_callback_odometry, 10)
                cmt = "Odometry (pose.position only; 速度自己 finite_diff)"
            else:
                self.ball_subscriber = self.create_subscription(
                    PointStamped, self._ball_position_camera_topic,
                    self._ball_position_camera_callback, 10)
                cmt = "PointStamped"
            self.get_logger().info(
                f"Ball subscriber [camera]: {self._ball_position_camera_topic} "
                f"({cmt}, 相机 FLU; node -> base_link (head FK) -> yaw (IMU de-tilt))")
        else:  # base_link
            if self._ball_base_msg_type == "PointStamped":
                self.ball_subscriber = self.create_subscription(
                    PointStamped, self._ball_base_topic, self._ball_base_callback_pointstamped, 10)
                mt = "PointStamped (pos only; measured 速度不可用)"
            elif self._ball_base_msg_type == "Odometry":
                self.ball_subscriber = self.create_subscription(
                    Odometry, self._ball_base_topic, self._ball_base_callback_odometry, 10)
                mt = "Odometry (pose.position.xyz + twist.linear.xyz; z 已带入 de-tilt)"
            else:
                self.ball_subscriber = self.create_subscription(
                    Float32MultiArray, self._ball_base_topic, self._ball_base_callback_multiarray, 10)
                mt = "Float32MultiArray[x,y,vx,vy]"
            self.get_logger().info(
                f"Ball subscriber [base_link]: {self._ball_base_topic} ({mt}); "
                "视觉已在 base 系, 直接当 yaw-base 用。")

        self.get_logger().info(f"Starting main loop with duration: {MAIN_LOOP_DT} seconds.")
        self.main_loop_timer = self.create_timer(MAIN_LOOP_DT, self.main_loop_callback)
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_timer_counter = 0
            self.main_loop_timer_counter_time = time.time()
            self.main_loop_callback_time_consumptions = queue.Queue(maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)

    def main_loop_callback(self) -> None:
        t0 = time.time()

        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        agent = self.available_agents[self.current_agent_name]

        if self.current_agent_name == "cold_start":
            action, done = agent.step()
            if done:
                self.get_logger().info(
                    "ColdStartAgent done, press 'R1' to switch to soccer agent.",
                    throttle_duration_sec=10.0)
            self._send(agent, action)
            if done and self.joy_stick_data.R1:
                self.get_logger().info("R1 pressed, switching to soccer agent.")
                self._switch_to("soccer")

        elif self.current_agent_name == "soccer":
            action, done = agent.step()
            self._send(agent, action)
            if self.joy_stick_data.R1:
                self.get_logger().info("R1 pressed, re-resetting soccer agent.")
                agent.reset()

        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_callback_time_consumptions.put(time.time() - t0)
            self.main_loop_timer_counter += 1
            if self.main_loop_timer_counter % MAIN_LOOP_FREQUENCY_CHECK_INTERVAL == 0:
                times = [self.main_loop_callback_time_consumptions.get()
                         for _ in range(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)]
                self.get_logger().info(
                    f"Actual main loop frequency: "
                    f"{(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL / (time.time() - self.main_loop_timer_counter_time)):.2f} Hz. "
                    f"Mean time consumption: {np.mean(times):.4f} s.")
                self.main_loop_timer_counter = 0
                self.main_loop_timer_counter_time = time.time()

    def _switch_to(self, name: str) -> None:
        self.current_agent_name = name
        self.available_agents[name].reset()

    def _send(self, agent, action: np.ndarray) -> None:
        raw = getattr(agent, "_raw_policy_action", None)
        if raw is not None:
            # soccer agent: 策略原始 20 维输出, 直接就是训练里 last_action 的宽度
            self._last_action_for_obs[:] = np.asarray(raw, dtype=np.float32).reshape(-1)
            self.raw_policy_action_publisher.publish(
                Float32MultiArray(data=np.asarray(raw, dtype=np.float32).tolist()))
        else:
            # cold_start agent 没有 _raw_policy_action -> 从它下发的 22 维 action 里
            # 取策略控制的那 20 列, 保住 ros_nodes/base.py 说的跨 agent last_action 共享。
            # ** 不能整段赋值 **: 长度对不上, 而且 head 两列不属于策略动作空间。
            a = np.asarray(action, dtype=np.float32).reshape(-1)
            if a.shape[0] == self.NUM_ACTIONS:
                self._last_action_for_obs[:] = a[self._act_sim_ids_np]
            elif a.shape[0] == self._last_action_for_obs.shape[0]:
                self._last_action_for_obs[:] = a
            else:
                self.get_logger().error(
                    f"[HeadFree] action 维度 {a.shape[0]} 既不是 {self.NUM_ACTIONS} 也不是 "
                    f"{self._last_action_for_obs.shape[0]}, last_action obs 这一帧不更新。")
        self.send_action(action, agent.action_offset, agent.action_scale, agent.p_gains, agent.d_gains)


# --------------------------------------------------------------------------- #
# Entrypoint
# --------------------------------------------------------------------------- #
def main(args) -> None:
    rclpy.init()

    logdir, soccer_actor_path = resolve_logdir_and_actor(args.logdir)

    node = PiPlusWalkKickBallVelHeadFreeEncDecNode(
        robot_class_name="PiPlus_S_12L8A0G2H0W",
        joint_pos_protect_ratio=2.0,
        dryrun=not args.nodryrun,
        computer_clip_torque=False,
    )
    node.set_ball_config(
        frame=args.ball_frame,
        camera_topic=args.ball_position_camera_topic,
        base_topic=args.ball_base_topic,
        base_msg_type=args.ball_base_msg_type,
        camera_msg_type=args.ball_position_camera_msg_type,
    )
    if not args.head_obs_passthrough:
        node.get_logger().warn(
            "[head] --head_obs_overridden 在 HeadFree 下是 no-op: head 不在动作空间, "
            "last_action obs 只有 20 列, 没有'覆盖值'可喂。")

    soccer_agent = _create_walk_kick_ballvel_agent(
        logdir=logdir,
        actor_path=soccer_actor_path,
        ros_node=node,
        kick_speed=args.kick_speed,
        x_vel_scale=args.x_vel_scale,
        y_vel_scale=args.y_vel_scale,
        yaw_vel_scale=args.yaw_vel_scale,
        kick_dir_mode=args.kick_dir_mode,
        soccer_pub_period=args.soccer_pub_period,
        soccer_history_samples=args.soccer_history_samples,
        ball_topic_timeout=args.ball_topic_timeout,
        head_mode=args.head_mode,
        head_action_scale=args.head_action_scale,
        head_pitch_rad=math.radians(args.head_pitch_deg),
        head_yaw_rad=math.radians(args.head_yaw_deg),
        head_track_kp=args.head_track_kp,
        head_track_deadband_rad=math.radians(args.head_track_deadband_deg),
        head_track_reacquire_rad=math.radians(args.head_track_reacquire_deg),
        head_track_smooth=args.head_track_smooth,
        head_track_max_rate_rad_s=math.radians(args.head_track_max_rate_dps),
        head_track_search_pitch_rad=math.radians(args.head_track_search_pitch_deg),
        head_track_search_rate_rad_s=math.radians(args.head_track_search_rate_dps),
        head_track_hold_grace_s=args.head_track_hold_grace_s,
        head_track_yaw_trim_rad=math.radians(args.head_track_yaw_trim_deg),
        head_track_pitch_trim_rad=math.radians(args.head_track_pitch_trim_deg),
        posonly=args.posonly,
        ball_vel_scale=args.ball_vel_scale,
        ball_frame=args.ball_frame,
        ball_vel_source=args.ball_vel_source,
        ball_base_detilt=args.ball_base_detilt,
    )
    node.register_agent("soccer", soccer_agent)

    cold_start_agent = ColdStartAgent(
        startup_step_size=args.startup_step_size,
        ros_node=node,
        joint_target_pos=soccer_agent.default_joint_pos,
        action_scale=soccer_agent.action_scale,
        action_offset=soccer_agent.action_offset,
        p_gains=soccer_agent.p_gains * args.kpkd_factor,
        d_gains=soccer_agent.d_gains * args.kpkd_factor,
    )
    node.register_agent("cold_start", cold_start_agent)

    H = args.soccer_history_samples
    cmd_cols = 5   # 7 维命令删球位置[3,4] -> 5 列
    n_joint = node.NUM_JOINTS                       # 22, joint_pos/joint_vel 仍含 head
    n_act = soccer_agent.NUM_POLICY_ACTIONS         # 20, last_action 无 head
    # 24(ang_vel) + 24(gravity) + H*5(cmd) + 8*22(jp) + 8*22(jv) + 8*20(act) + H*2(ball_pos)
    base = 24 + 24 + H * cmd_cols + 8 * n_joint + 8 * n_joint + 8 * n_act + H * 2
    total = base + (0 if args.posonly else H * 2)
    print(f"logdir: {logdir}  |  actor: {soccer_actor_path or '(default exported/actor.onnx)'}")
    print(f"posonly         : {args.posonly}  "
          f"({'encoder 只吃球位置' if args.posonly else 'encoder 加球速'})")
    print(f"ball_frame      : {args.ball_frame}  "
          f"({'相机链路 optical->base FK->yaw detilt' if args.ball_frame=='camera' else 'base_link 链路 (视觉已 base 系)'})")
    print(f"ball_vel_source : {args.ball_vel_source}  "
          f"({'视觉测量球速' if args.ball_vel_source=='measured' else '位置历史有限差分'}), vel_scale={args.ball_vel_scale:.2f}")
    if args.ball_frame == "camera":
        print(f"ball topic      : {args.ball_position_camera_topic}  (PointStamped OPTICAL; timeout={args.ball_topic_timeout:.2f}s)")
    else:
        print(f"ball topic      : {args.ball_base_topic}  ({args.ball_base_msg_type}; timeout={args.ball_topic_timeout:.2f}s)")
        print(f"                  ** 上真机前请 `ros2 topic echo {args.ball_base_topic} --once` 核对字段顺序/单位! **")
    print(f"kick_speed      : {args.kick_speed:.2f} m/s   dir_mode: {args.kick_dir_mode}")
    _eff_rate = math.degrees(soccer_agent._ht_max_rate)
    _train_rate = math.degrees(soccer_agent._head_train_max_speed)
    if args.head_mode == "track":
        _eff_sp = math.degrees(soccer_agent._ht_search_pitch)
        print(f"head mode       : TRACK ball ({args.ball_frame} frame) | "
              f"freeze=±{args.head_track_deadband_deg:.0f}deg reacquire=±{args.head_track_reacquire_deg:.0f}deg "
              f"smooth={args.head_track_smooth:.2f} kp={args.head_track_kp:.2f} "
              f"max_rate={_eff_rate:.0f}dps search_pitch={_eff_sp:.0f}deg"
              + (f" (req {args.head_track_search_pitch_deg:.0f} CLAMPED)"
                 if args.head_track_search_pitch_deg > _eff_sp + 0.1 else ""))
        print(f"  NOTE: 视觉节点要关掉它自己的头部跟踪 (ball_vision_ros2.py --no-head-track), "
              f"否则两边抢头。")
    elif args.head_mode == "lock":
        print(f"head mode       : LOCK fixed (pitch={math.degrees(soccer_agent._head_pitch_target_rad):.0f}deg "
              f"yaw={math.degrees(soccer_agent._head_yaw_target_rad):.0f}deg)")
    else:
        print(f"head mode       : ZERO_GAIN (head kp=kd=0, 目标=实测角; 头交给别的控制器)")
        print(f"  ⚠ 只有当那个控制器自己下发 head 的 MotorControlCommand 时才有意义。")
    print(f"head range      : yaw [{soccer_agent._head_yaw_lo:+.3f}, {soccer_agent._head_yaw_hi:+.3f}] "
          f"pitch [{soccer_agent._head_pitch_lo:+.3f}, {soccer_agent._head_pitch_hi:+.3f}] rad "
          f"({'env.yaml' if soccer_agent._head_range_from_yaml else '0.9xURDF 兜底'}); "
          f"限速 {_eff_rate:.0f}dps <= 训练 head_max_speed {_train_rate:.0f}dps")
    print(f"head action_scale: {soccer_agent._head_action_scale:.5f} (部署侧自定; "
          f"env.yaml 里 head 没有 scale, 留 0 会让 ColdStart 除零下发 NaN)")
    _A = WalkKickBallVelHeadFreeEncDecAgent
    print(f"ball buffer     : 全程填真球 (encoder 一直看得到球); "
          f"距离 mask far={_A.BALL_FAR_MASK_DIST:.2f}m, hold-last {_A.BALL_HOLD_MAX_S:.2f}s "
          f"(和训练 observations.py 同一套规则)")
    print(f"ball latent 门控: kick_speed != 0 (按住 A + 左摇杆出死区) 才把球接入策略; "
          f"松手后再保持 {_A.BALL_GATE_RELEASE_HOLD_S:.2f}s")
    print(f"                  门关 -> 喂全 0 -> encoder 输出'球在 far_mask 外'的 latent (不是 z=0); "
          f"buffer 不清空, 门一开就是满窗 1.0s 真球, 无填窗延迟")
    print(f"buttons         : (none)=walk  A=kick")
    print(f"history         : ball {H}x2 @ pub_period={args.soccer_pub_period} (10Hz async); cmd {H} @ 50Hz")
    print(f"  -> soccer_command : {H * cmd_cols}-dim (7维命令删球位置[3,4] -> 5列)")
    print(f"  -> ball_pos_b_noisy: {H * 2}-dim")
    if not args.posonly:
        print(f"  -> ball_vel_b_noisy: {H * 2}-dim ({'测量球速' if args.ball_vel_source=='measured' else '位置差分估计'})")
    print(f"  -> joint_pos / joint_vel : {8 * n_joint}-dim each (22 关节, ** head 保留 **)")
    print(f"  -> last_action           : {8 * n_act}-dim ({n_act} 维动作, ** 无 head **)")
    print(f"  -> expected onnx input total: {total}  (posonly=True 时 {base})")
    print(f"     对照非 HeadFree 的 ballvel 版: 666 / 646。差 16 = head 2 列 x 8 帧。")
    print("  NOTE: --posonly 必须和加载模型的 env.yaml 一致; 启动自检会当场报错, 不会带病上机。")
    print(f"action mapping  : policy {n_act} 维 -> sim 关节 "
          f"{list(soccer_agent._act_sim_ids)}; head 在 {soccer_agent._head_sim_ids} 由本脚本控制")

    node.start_ros_handlers()
    node.get_logger().info("PiPlusWalkKickBallVelHeadFreeEncDecNode is ready.")

    if args.skip_cold_start:
        node.current_agent_name = "soccer"
        node.available_agents["soccer"].reset()
        node.get_logger().info("Skipping cold start, starting with soccer agent directly.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="PiPlus Walk+Kick BallVel HeadFree Perceptive Node "
                    "(22-DOF robot / 20-DOF policy, head 归本脚本管, 7-D cmd)"
    )
    parser.add_argument("--logdir", type=str, required=True,
        help="policy path (dir auto-loads exported/actor.onnx, or a direct .onnx).")
    parser.add_argument("--kick_speed", type=float, default=3.5,
        help="Kick speed magnitude in m/s (held constant while A + stick).")
    parser.add_argument("--kick_dir_mode", type=str, default="anchor", choices=["follow", "anchor"],
        help="Kick direction frame: 'follow' (current robot frame) or 'anchor' (fixed at A-press).")

    # ---- encoder 球速 ----
    parser.add_argument("--posonly", action="store_true", default=False,
        help="Encoder obs = ball POSITION only (646). Without this flag (default) adds "
             "ball_vel_b_noisy (666). MUST match the loaded model's env.yaml.")
    parser.add_argument("--ball_vel_scale", type=float, default=0.25,
        help="Scale applied to the ball-velocity obs (match training vel_scale). Default 0.25")
    parser.add_argument("--ball_vel_source", type=str, default="finite_diff",
        choices=["finite_diff", "measured"],
        help="Ball velocity source (non-posonly only). 'finite_diff': from ball position history. "
             "'measured': use vision-reported velocity (base_link frame only).")

    # ---- 坐标系 / 话题 ----
    parser.add_argument("--ball_frame", type=str, default="camera", choices=["camera", "base_link"],
        help="Frame of the vision ball topic.")
    parser.add_argument("--ball_position_camera_topic", type=str, default=DEFAULT_BALL_POSITION_CAMERA_TOPIC,
        help="Camera-frame ball topic (相机 FLU). Used when --ball_frame camera.")
    parser.add_argument("--ball_position_camera_msg_type", type=str, default="PointStamped",
        choices=["PointStamped", "Odometry"],
        help="Message type of --ball_position_camera_topic. PointStamped: point.xyz (old backend "
             "/soccer/ball_position_camera). Odometry: pose.position.xyz (ball_vision_ros2 左目话题 "
             "/soccer/ball_left_eye_xyz_velocity; twist 丢弃, 速度自己 finite_diff). VERIFY with `ros2 topic echo`!")
    parser.add_argument("--ball_base_topic", type=str, default=DEFAULT_BALL_BASE_TOPIC,
        help="Base_link-frame ball topic. Used when --ball_frame base_link.")
    parser.add_argument("--ball_base_msg_type", type=str, default="Float32MultiArray",
        choices=["Float32MultiArray", "PointStamped", "Odometry"],
        help="Message type of --ball_base_topic. Float32MultiArray: data=[x,y,vx,vy] (len 2 = pos only). "
             "PointStamped: point.x/y/z (pos only). Odometry: pose.position.xyz + twist.linear.xyz (base_link, "
             "z 已带入 de-tilt). VERIFY with `ros2 topic echo`!")
    parser.add_argument("--ball_base_detilt", action="store_true", default=False,
        help="Apply IMU de-tilt to base_link ball xy/vel (only if vision gives RAW tilted base_link).")
    # ---- 球 obs mask (距离制, 与 A 键解耦) ----
    # 兼容老命令行: 球 obs 的行为现在固定 (见 WalkKickBallVelHeadFreeEncDecAgent.BALL_* 常量),
    # 保留这两个旧开关为 no-op, 免得已有启动脚本报错。
    parser.add_argument("--no_mask_ball_idle", dest="_dead_mask_ball_idle",
        action="store_false", default=False, help=argparse.SUPPRESS)
    parser.add_argument("--mask_ball_idle", dest="_dead_mask_ball_idle",
        action="store_true", help=argparse.SUPPRESS)

    # ---- 通用 ----
    parser.add_argument("--ball_topic_timeout", type=float, default=0.2,
        help="Max age (s) for the ball before masking/hold-last. Default: 0.2")
    parser.add_argument("--soccer_pub_period", type=int, default=5,
        help="Push a fresh BALL snapshot into the async history every N control steps (5 -> 10Hz).")
    parser.add_argument("--soccer_history_samples", type=int, default=10,
        help="Stacked history snapshots. MUST match history_samples in the env cfg. Default: 10")
    parser.add_argument("--startup_step_size", type=float, default=0.2,
        help="Cold start step size (default: 0.2)")
    # HeadFree 下无意义 (head 不在动作空间, last_action 只有 20 列), 保留为 no-op 不报错
    parser.add_argument("--head_obs_overridden", dest="head_obs_passthrough",
                        action="store_false", default=True, help=argparse.SUPPRESS)
    # ---- head: HeadFree 下头部完全归本脚本 ----
    parser.add_argument("--head_mode", type=str, default="track",
        choices=["track", "lock", "zero_gain"],
        help="track(默认): 用视觉把头指向球; lock: 固定看向 --head_pitch_deg/--head_yaw_deg; "
             "zero_gain: head kp=kd=0 且目标=实测角, 把头让给别的控制器 (⚠ 见文件头说明)。")
    parser.add_argument("--head_action_scale", type=float, default=DEFAULT_HEAD_ACTION_SCALE,
        help="head 目标角 <-> action 列的换算系数。headfree 的 env.yaml 里 head 没有 scale "
             "(解析出来是 0), 留 0 会让 ColdStartAgent 的 (target-offset)/scale 除零 -> 下发 NaN。"
             "取值不影响物理 (乘回去抵消), 默认沿用非 headfree 任务的 0.09614。")
    # 老开关: --head_lock / --head_track 映射到 --head_mode, 保住肌肉记忆和老启动脚本
    parser.add_argument("--head_lock", dest="head_mode", action="store_const", const="lock",
        help="等价于 --head_mode lock。")
    parser.add_argument("--head_track", dest="head_mode", action="store_const", const="track",
        help="等价于 --head_mode track (本来就是默认)。")
    parser.add_argument("--head_pitch_deg", type=float, default=45.0,
        help="Head pitch target (deg) when --head_mode lock. 会被钳进训练 head_pos_range。")
    parser.add_argument("--head_yaw_deg", type=float, default=0.0,
        help="Head yaw target (deg) when --head_mode lock. 会被钳进训练 head_pos_range。")

    parser.add_argument("--head_track_kp", type=float, default=0.35,
        help="Fraction of the angular error corrected per control step (ease-in). Lower = smoother/slower.")
    parser.add_argument("--head_track_deadband_deg", type=float, default=7.0,
        help="Center FREEZE zone (half-width, deg). Ball within this angle of image center -> head holds still.")
    parser.add_argument("--head_track_reacquire_deg", type=float, default=12.0,
        help="Hysteresis outer threshold (deg). Head starts re-centering only once the ball drifts past this. "
             "Must be > deadband.")
    parser.add_argument("--head_track_smooth", type=float, default=0.4,
        help="EMA weight of each new ball measurement (0.05..1.0). Lower = more smoothing/lag. 1.0 = none.")
    parser.add_argument("--head_track_max_rate_dps", type=float, default=90.0,
        help="Max head slew speed (deg/s) while re-centering. Lower = calmer head. "
             "** 会被钳到训练的 head_max_speed (2.0 rad/s = 114 dps) 以内 ** —— 头转得比训练里"
             "快, 对身体策略就是没见过的惯量扰动。")
    parser.add_argument("--head_track_search_pitch_deg", type=float, default=45.0,
        help="Downward pitch the head eases to when the ball is lost (search pose). +down. "
             "NOTE: 训练 head_pitch 上限 = 0.9*0.88 = 0.792 rad (~45deg), 超了会被钳位。")
    parser.add_argument("--head_track_search_rate_dps", type=float, default=30.0,
        help="Slew speed (deg/s) used to ease back to the search pose after losing the ball.")
    parser.add_argument("--head_track_hold_grace_s", type=float, default=0.5,
        help="Hold the last aim this long after losing the ball before easing to the search pose.")
    parser.add_argument("--head_track_yaw_trim_deg", type=float, default=0.0,
        help="Static yaw offset to null residual centering bias (FK/extrinsic calibration).")
    parser.add_argument("--head_track_pitch_trim_deg", type=float, default=0.0,
        help="Static pitch offset to null residual centering bias (+down).")
    parser.add_argument("--kpkd_factor", type=float, default=1.0,
        help="KP/KD gain multiplier for cold start")
    parser.add_argument("--x_vel_scale", type=float, default=1.0,
        help="Forward/backward velocity scale (default: 1.0)")
    parser.add_argument("--y_vel_scale", type=float, default=-0.5,
        help="Lateral velocity scale (default: -0.5)")
    parser.add_argument("--yaw_vel_scale", type=float, default=-1.2,
        help="Yaw velocity scale (default: -1.2)")
    parser.add_argument("--nodryrun", action="store_true", default=False,
        help="Actually send commands to robot (default: dryrun)")
    parser.add_argument("--skip_cold_start", action="store_true", default=False,
        help="Skip cold start agent")
    parser.add_argument("--debug", action="store_true", default=False,
        help="Enable debugpy on port 6789")

    args = parser.parse_args()

    if args.debug:
        import debugpy
        ip_address = ("0.0.0.0", 6789)
        print("Waiting for debugger at %s:%d" % ip_address, flush=True)
        debugpy.listen(ip_address)
        debugpy.wait_for_client()
        debugpy.breakpoint()

    main(args)