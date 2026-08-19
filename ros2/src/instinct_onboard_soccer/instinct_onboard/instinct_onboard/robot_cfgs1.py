import math

import numpy as np

"""A general configuration file for the robots, shared between different scripts. """


class UnitreeWirelessButtons:
    R1 = 0b00000001  # 1
    L1 = 0b00000010  # 2
    start = 0b00000100  # 4
    select = 0b00001000  # 8
    R2 = 0b00010000  # 16
    L2 = 0b00100000  # 32
    F1 = 0b01000000  # 64
    F2 = 0b10000000  # 128
    A = 0b100000000  # 256
    B = 0b1000000000  # 512
    X = 0b10000000000  # 1024
    Y = 0b100000000000  # 2048
    up = 0b1000000000000  # 4096
    right = 0b10000000000000  # 8192
    down = 0b100000000000000  # 16384
    left = 0b1000000000000000  # 32768


class G1_29Dof_TorsoBase:
    NUM_JOINTS = 29
    NUM_ACTIONS = 29
    joint_map = [
        15,
        22,  # shoulder pitch
        14,  # waist pitch
        16,
        23,  # shoulder roll
        13,  # waist roll
        17,
        24,  # shoulder yaw
        12,  # waist yaw
        18,
        25,  # elbow
        0,
        6,  # hip pitch
        19,
        26,  # wrist roll
        1,
        7,  # hip roll
        20,
        27,  # wrist pitch
        2,
        8,  # hip yaw
        21,
        28,  # wrist yaw
        3,
        9,  # knee
        4,
        10,  # ankle pitch
        5,
        11,  # ankle roll
    ]
    sim_joint_names = [  # NOTE: order matters. This list is the order in simulation.
        "left_shoulder_pitch_joint",  #
        "right_shoulder_pitch_joint",
        "waist_pitch_joint",
        "left_shoulder_roll_joint",  #
        "right_shoulder_roll_joint",
        "waist_roll_joint",
        "left_shoulder_yaw_joint",  #
        "right_shoulder_yaw_joint",
        "waist_yaw_joint",
        "left_elbow_joint",  #
        "right_elbow_joint",
        "left_hip_pitch_joint",
        "right_hip_pitch_joint",
        "left_wrist_roll_joint",
        "right_wrist_roll_joint",
        "left_hip_roll_joint",  #
        "right_hip_roll_joint",
        "left_wrist_pitch_joint",
        "right_wrist_pitch_joint",
        "left_hip_yaw_joint",
        "right_hip_yaw_joint",
        "left_wrist_yaw_joint",  #
        "right_wrist_yaw_joint",
        "left_knee_joint",
        "right_knee_joint",
        "left_ankle_pitch_joint",  #
        "right_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
    ]
    real_joint_names = [  # NOTE: order matters. This list is the order in real robot.
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]
    joint_signs = np.array(
        [
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
        ],
        dtype=np.float32,
    )
    joint_limits_high = np.array(
        [
            2.6704,
            2.6704,
            0.5200,
            2.2515,
            1.5882,
            0.5200,
            2.6180,
            2.6180,
            2.6180,
            2.0944,
            2.0944,
            2.8798,
            2.8798,
            1.9722,
            1.9722,
            2.9671,
            0.5236,
            1.6144,
            1.6144,
            2.7576,
            2.7576,
            1.6144,
            1.6144,
            2.8798,
            2.8798,
            0.5236,
            0.5236,
            0.2618,
            0.2618,
        ],
        dtype=np.float32,
    )
    joint_limits_low = np.array(
        [
            -3.0892,
            -3.0892,
            -0.5200,
            -1.5882,
            -2.2515,
            -0.5200,
            -2.6180,
            -2.6180,
            -2.6180,
            -1.0472,
            -1.0472,
            -2.5307,
            -2.5307,
            -1.9722,
            -1.9722,
            -0.5236,
            -2.9671,
            -1.6144,
            -1.6144,
            -2.7576,
            -2.7576,
            -1.6144,
            -1.6144,
            -0.0873,
            -0.0873,
            -0.8727,
            -0.8727,
            -0.2618,
            -0.2618,
        ],
        dtype=np.float32,
    )
    torque_limits = np.array(
        [  # from urdf and in simulation order
            25,
            25,
            50,
            25,
            25,
            50,
            25,
            25,
            88,
            25,
            25,
            88,
            88,
            25,
            25,
            88,
            88,
            5,
            5,
            88,
            88,
            5,
            5,
            139,
            139,
            50,
            50,
            50,
            50,
        ],
        dtype=np.float32,
    )
    turn_on_motor_mode = [0x01] * 29
    mode_pr = 0
    mode_machine = 5
    """ please check this value from
        https://support.unitree.com/home/zh/G1_developer/basic_services_interface
        https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
    """
    realsense_depth_link_transform = {
        "translation": (
            0.04764571478 + 0.0039635 - 0.0042 * math.cos(math.radians(48)),
            0.015,
            0.46268178553 - 0.044 + 0.0042 * math.sin(math.radians(48)) + 0.016,
        ),
        "rotation": (
            math.cos(math.radians(0.5) / 2) * math.cos(math.radians(48) / 2),  # w
            math.sin(math.radians(0.5) / 2),  # x
            math.sin(math.radians(48) / 2),  # y
            0.0,  # z
        ),
        "parent_frame": "torso_link",
        "child_frame": "realsense_depth_link",
    }


class PiPlus_S_12L8A0G0H1W:
    """Configuration for PiPlus_S_12L8A0G0H1W humanoid robot.

    This robot has 23 degrees of freedom:
    - 12 leg joints (6 per leg)
    - 1 waist yaw joint
    - 10 arm joints (5 per arm)
    """

    NUM_JOINTS = 23
    NUM_ACTIONS = 23

    # Joint order from IsaacSim (this is the simulation order)
    sim_joint_names = [  # NOTE: order matters. This list is the order in simulation.
        "l_hip_pitch_joint",  # 0
        "r_hip_pitch_joint",  # 1
        "waist_yaw_joint",  # 2
        "l_hip_roll_joint",  # 3
        "r_hip_roll_joint",  # 4
        "head_yaw_joint",  # 5
        "l_shoulder_pitch_joint",  # 6
        "r_shoulder_pitch_joint",  # 7
        "l_thigh_joint",  # 8
        "r_thigh_joint",  # 9
        "head_pitch_joint",  # 10
        "l_shoulder_roll_joint",  # 11
        "r_shoulder_roll_joint",  # 12
        "l_calf_joint",  # 13
        "r_calf_joint",  # 14
        "l_upper_arm_joint",  # 15
        "r_upper_arm_joint",  # 16
        "l_ankle_pitch_joint",  # 17
        "r_ankle_pitch_joint",  # 18
        "l_elbow_joint",  # 19
        "r_elbow_joint",  # 20
        "l_ankle_roll_joint",  # 21
        "r_ankle_roll_joint",  # 22
    ]

    # Joint order mapping from simulation to real robot
    # Maps sim_joint_names[i] to the index in real_joint_names (actual joint_states order)
    joint_map = [
        5,  # 0: l_hip_pitch_joint -> real[5]
        11,  # 1: r_hip_pitch_joint -> real[11]
        22,  # 2: waist_yaw_joint -> real[22]
        4,  # 3: l_hip_roll_joint -> real[4]
        10,  # 4: r_hip_roll_joint -> real[10]
        20,  # 5: head_yaw_joint -> real[20]
        12,  # 6: l_shoulder_pitch_joint -> real[12]
        16,  # 7: r_shoulder_pitch_joint -> real[16]
        3,  # 8: l_thigh_joint -> real[3]
        9,  # 9: r_thigh_joint -> real[9]
        21,  # 10: head_pitch_joint -> real[21]
        13,  # 11: l_shoulder_roll_joint -> real[13]
        17,  # 12: r_shoulder_roll_joint -> real[17]
        2,  # 13: l_calf_joint -> real[2]
        8,  # 14: r_calf_joint -> real[8]
        14,  # 15: l_upper_arm_joint -> real[14]
        18,  # 16: r_upper_arm_joint -> real[18]
        1,  # 17: l_ankle_pitch_joint -> real[1]
        7,  # 18: r_ankle_pitch_joint -> real[7]
        15,  # 19: l_elbow_joint -> real[15]
        19,  # 20: r_elbow_joint -> real[19]
        0,  # 21: l_ankle_roll_joint -> real[0]
        6,  # 22: r_ankle_roll_joint -> real[6]
    ]

    # Real robot joint names (actual order from hightorque_midware joint_states)
    real_joint_names = [
        "l_ankle_roll_joint",  # 0
        "l_ankle_pitch_joint",  # 1
        "l_calf_joint",  # 2
        "l_thigh_joint",  # 3
        "l_hip_roll_joint",  # 4
        "l_hip_pitch_joint",  # 5
        "r_ankle_roll_joint",  # 6
        "r_ankle_pitch_joint",  # 7
        "r_calf_joint",  # 8
        "r_thigh_joint",  # 9
        "r_hip_roll_joint",  # 10
        "r_hip_pitch_joint",  # 11
        "l_shoulder_pitch_joint",  # 12
        "l_shoulder_roll_joint",  # 13
        "l_upper_arm_joint",  # 14
        "l_elbow_joint",  # 15
        "r_shoulder_pitch_joint",  # 16
        "r_shoulder_roll_joint",  # 17
        "r_upper_arm_joint",  # 18
        "r_elbow_joint",  # 19
        "head_yaw_joint",  # 20
        "head_pitch_joint",  # 21
        "waist_yaw_joint",  # 22
    ]

    # Joint signs (1 for normal, -1 for reversed) - in IsaacSim order
    joint_signs = np.array(
        #[1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1], dtype=np.float32
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1],
        #0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22
        #15, 16可能反了
        dtype=np.float32,
    )

    # Joint limits (IsaacSim order)
    joint_limits_high = np.array(
        [
            2.94,  # 0: l_hip_pitch_joint
            2.94,  # 1: r_hip_pitch_joint
            3.14,  # 2: waist_yaw_joint
            3.14,  # 3: l_hip_roll_joint
            0.17,  # 4: r_hip_roll_joint
            3.14,  # 5: head_yaw_joint
            1.04,  # 6: l_shoulder_pitch_joint
            1.04,  # 7: r_shoulder_pitch_joint
            2.87,  # 8: l_thigh_joint
            2.87,  # 9: r_thigh_joint
            3.14,  # 10: head_pitch_joint
            3.00,  # 11: l_shoulder_roll_joint
            0.13,  # 12: r_shoulder_roll_joint
            2.35,  # 13: l_calf_joint
            2.35,  # 14: r_calf_joint
            2.16,  # 15: l_upper_arm_joint
            2.16,  # 16: r_upper_arm_joint
            0.92,  # 17: l_ankle_pitch_joint
            0.92,  # 18: r_ankle_pitch_joint
            2.00,  # 19: l_elbow_joint
            2.00,  # 20: r_elbow_joint
            0.78,  # 21: l_ankle_roll_joint
            0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    joint_limits_low = np.array(
        [
            -3.12,  # 0: l_hip_pitch_joint
            -3.12,  # 1: r_hip_pitch_joint
            -3.14,  # 2: waist_yaw_joint
            -0.17,  # 3: l_hip_roll_joint
            -3.14,  # 4: r_hip_roll_joint
            -3.14,  # 5: head_yaw_joint
            -4.18,  # 6: l_shoulder_pitch_joint
            -4.18,  # 7: r_shoulder_pitch_joint
            -2.87,  # 8: l_thigh_joint
            -2.87,  # 9: r_thigh_joint
            -3.14,  # 10: head_pitch_joint
            -0.13,  # 11: l_shoulder_roll_joint
            -3.00,  # 12: r_shoulder_roll_joint
            -2.14,  # 13: l_calf_joint
            -2.14,  # 14: r_calf_joint
            -2.16,  # 15: l_upper_arm_joint
            -2.16,  # 16: r_upper_arm_joint
            -0.97,  # 17: l_ankle_pitch_joint
            -0.97,  # 18: r_ankle_pitch_joint
            -2.00,  # 19: l_elbow_joint
            -2.00,  # 20: r_elbow_joint
            -0.78,  # 21: l_ankle_roll_joint
            -0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    # Torque limits (IsaacSim order)
    torque_limits = np.array(
        [
            20.0,  # 0: l_hip_pitch_joint
            20.0,  # 1: r_hip_pitch_joint
            20.0,  # 2: waist_yaw_joint
            20.0,  # 3: l_hip_roll_joint
            20.0,  # 4: r_hip_roll_joint
            10.0,  # 5: head_yaw_joint
            10.0,  # 6: l_shoulder_pitch_joint
            10.0,  # 7: r_shoulder_pitch_joint
            20.0,  # 8: l_thigh_joint
            20.0,  # 9: r_thigh_joint
            10.0,  # 10: head_pitch_joint
            10.0,  # 11: l_shoulder_roll_joint
            10.0,  # 12: r_shoulder_roll_joint
            20.0,  # 13: l_calf_joint
            20.0,  # 14: r_calf_joint
            10.0,  # 15: l_upper_arm_joint
            10.0,  # 16: r_upper_arm_joint
            20.0,  # 17: l_ankle_pitch_joint
            20.0,  # 18: r_ankle_pitch_joint
            10.0,  # 19: l_elbow_joint
            10.0,  # 20: r_elbow_joint
            20.0,  # 21: l_ankle_roll_joint
            20.0,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    turn_on_motor_mode = [0x01] * NUM_JOINTS
    mode_pr = 0
    mode_machine = 5

    # RealSense depth camera transform - depth_link is at camera_link with zero offset
    realsense_depth_link_transform = {
        "translation": (0.0, 0.0, 0.0),  # Zero offset from camera_link
        "rotation": (1.0, 0.0, 0.0, 0.0),  # No rotation (identity quaternion)
        "parent_frame": "camera_link",
        "child_frame": "realsense_depth_link",
    }


class PiPlus_S_12L8A0G0H1W_Lower:
    """Configuration for PiPlus_S_12L8A0G0H1W humanoid robot, lower-body-only policy.

    使用旧版 Isaac sim 关节顺序（与 PiPlus_S_12L8A0G0H1W 一致）。
    Policy 只控制 12 个下肢关节，但观测仍读取全部 23 个关节。

    sim_joint_names 顺序（23 joints，与 PiPlus_S_12L8A0G0H1W 完全一致）：
      [J00] l_hip_pitch_joint
      [J01] r_hip_pitch_joint
      [J02] waist_yaw_joint
      [J03] l_hip_roll_joint
      [J04] r_hip_roll_joint
      [J05] head_yaw_joint
      [J06] l_shoulder_pitch_joint
      [J07] r_shoulder_pitch_joint
      [J08] l_thigh_joint
      [J09] r_thigh_joint
      [J10] head_pitch_joint
      [J11] l_shoulder_roll_joint
      [J12] r_shoulder_roll_joint
      [J13] l_calf_joint
      [J14] r_calf_joint
      [J15] l_upper_arm_joint
      [J16] r_upper_arm_joint
      [J17] l_ankle_pitch_joint
      [J18] r_ankle_pitch_joint
      [J19] l_elbow_joint
      [J20] r_elbow_joint
      [J21] l_ankle_roll_joint
      [J22] r_ankle_roll_joint

    Action joint order (12 lower-body joints):
      [A00] l_hip_pitch_joint   (J00)
      [A01] r_hip_pitch_joint   (J01)
      [A02] l_hip_roll_joint    (J03)
      [A03] r_hip_roll_joint    (J04)
      [A04] l_thigh_joint       (J08)
      [A05] r_thigh_joint       (J09)
      [A06] l_calf_joint        (J13)
      [A07] r_calf_joint        (J14)
      [A08] l_ankle_pitch_joint (J17)
      [A09] r_ankle_pitch_joint (J18)
      [A10] l_ankle_roll_joint  (J21)
      [A11] r_ankle_roll_joint  (J22)
    """

    NUM_JOINTS = 23
    NUM_ACTIONS = 12  # 只控制下肢

    sim_joint_names = [
        "l_hip_pitch_joint",      # 0
        "r_hip_pitch_joint",      # 1
        "waist_yaw_joint",        # 2
        "l_hip_roll_joint",       # 3
        "r_hip_roll_joint",       # 4
        "head_yaw_joint",         # 5
        "l_shoulder_pitch_joint", # 6
        "r_shoulder_pitch_joint", # 7
        "l_thigh_joint",          # 8
        "r_thigh_joint",          # 9
        "head_pitch_joint",       # 10
        "l_shoulder_roll_joint",  # 11
        "r_shoulder_roll_joint",  # 12
        "l_calf_joint",           # 13
        "r_calf_joint",           # 14
        "l_upper_arm_joint",      # 15
        "r_upper_arm_joint",      # 16
        "l_ankle_pitch_joint",    # 17
        "r_ankle_pitch_joint",    # 18
        "l_elbow_joint",          # 19
        "r_elbow_joint",          # 20
        "l_ankle_roll_joint",     # 21
        "r_ankle_roll_joint",     # 22
    ]

    real_joint_names = [
        "l_ankle_roll_joint",     # 0
        "l_ankle_pitch_joint",    # 1
        "l_calf_joint",           # 2
        "l_thigh_joint",          # 3
        "l_hip_roll_joint",       # 4
        "l_hip_pitch_joint",      # 5
        "r_ankle_roll_joint",     # 6
        "r_ankle_pitch_joint",    # 7
        "r_calf_joint",           # 8
        "r_thigh_joint",          # 9
        "r_hip_roll_joint",       # 10
        "r_hip_pitch_joint",      # 11
        "l_shoulder_pitch_joint", # 12
        "l_shoulder_roll_joint",  # 13
        "l_upper_arm_joint",      # 14
        "l_elbow_joint",          # 15
        "r_shoulder_pitch_joint", # 16
        "r_shoulder_roll_joint",  # 17
        "r_upper_arm_joint",      # 18
        "r_elbow_joint",          # 19
        "head_yaw_joint",         # 20
        "head_pitch_joint",       # 21
        "waist_yaw_joint",        # 22
    ]

    joint_map = [
        5,   # 0:  l_hip_pitch_joint      -> real[5]
        11,  # 1:  r_hip_pitch_joint      -> real[11]
        22,  # 2:  waist_yaw_joint        -> real[22]
        4,   # 3:  l_hip_roll_joint       -> real[4]
        10,  # 4:  r_hip_roll_joint       -> real[10]
        20,  # 5:  head_yaw_joint         -> real[20]
        12,  # 6:  l_shoulder_pitch_joint -> real[12]
        16,  # 7:  r_shoulder_pitch_joint -> real[16]
        3,   # 8:  l_thigh_joint          -> real[3]
        9,   # 9:  r_thigh_joint          -> real[9]
        21,  # 10: head_pitch_joint       -> real[21]
        13,  # 11: l_shoulder_roll_joint  -> real[13]
        17,  # 12: r_shoulder_roll_joint  -> real[17]
        2,   # 13: l_calf_joint           -> real[2]
        8,   # 14: r_calf_joint           -> real[8]
        14,  # 15: l_upper_arm_joint      -> real[14]
        18,  # 16: r_upper_arm_joint      -> real[18]
        1,   # 17: l_ankle_pitch_joint    -> real[1]
        7,   # 18: r_ankle_pitch_joint    -> real[7]
        15,  # 19: l_elbow_joint          -> real[15]
        19,  # 20: r_elbow_joint          -> real[19]
        0,   # 21: l_ankle_roll_joint     -> real[0]
        6,   # 22: r_ankle_roll_joint     -> real[6]
    ]

    # policy 输出 action[i] 对应 sim_joint_names 中的索引
    action_sim_indices = [0, 1, 3, 4, 8, 9, 13, 14, 17, 18, 21, 22]

    joint_signs = np.array(
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        dtype=np.float32,
    )

    joint_limits_high = np.array(
        [
            2.94,  # 0:  l_hip_pitch_joint
            2.94,  # 1:  r_hip_pitch_joint
            3.14,  # 2:  waist_yaw_joint
            3.14,  # 3:  l_hip_roll_joint
            0.17,  # 4:  r_hip_roll_joint
            3.14,  # 5:  head_yaw_joint
            1.04,  # 6:  l_shoulder_pitch_joint
            1.04,  # 7:  r_shoulder_pitch_joint
            2.87,  # 8:  l_thigh_joint
            2.87,  # 9:  r_thigh_joint
            3.14,  # 10: head_pitch_joint
            3.00,  # 11: l_shoulder_roll_joint
            0.13,  # 12: r_shoulder_roll_joint
            2.35,  # 13: l_calf_joint
            2.35,  # 14: r_calf_joint
            2.16,  # 15: l_upper_arm_joint
            2.16,  # 16: r_upper_arm_joint
            0.92,  # 17: l_ankle_pitch_joint
            0.92,  # 18: r_ankle_pitch_joint
            2.00,  # 19: l_elbow_joint
            2.00,  # 20: r_elbow_joint
            0.78,  # 21: l_ankle_roll_joint
            0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    joint_limits_low = np.array(
        [
            -3.12,  # 0:  l_hip_pitch_joint
            -3.12,  # 1:  r_hip_pitch_joint
            -3.14,  # 2:  waist_yaw_joint
            -0.17,  # 3:  l_hip_roll_joint
            -3.14,  # 4:  r_hip_roll_joint
            -3.14,  # 5:  head_yaw_joint
            -4.18,  # 6:  l_shoulder_pitch_joint
            -4.18,  # 7:  r_shoulder_pitch_joint
            -2.87,  # 8:  l_thigh_joint
            -2.87,  # 9:  r_thigh_joint
            -3.14,  # 10: head_pitch_joint
            -0.13,  # 11: l_shoulder_roll_joint
            -3.00,  # 12: r_shoulder_roll_joint
            -2.14,  # 13: l_calf_joint
            -2.14,  # 14: r_calf_joint
            -2.16,  # 15: l_upper_arm_joint
            -2.16,  # 16: r_upper_arm_joint
            -0.97,  # 17: l_ankle_pitch_joint
            -0.97,  # 18: r_ankle_pitch_joint
            -2.00,  # 19: l_elbow_joint
            -2.00,  # 20: r_elbow_joint
            -0.78,  # 21: l_ankle_roll_joint
            -0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    torque_limits = np.array(
        [
            20.0,  # 0:  l_hip_pitch_joint
            20.0,  # 1:  r_hip_pitch_joint
            20.0,  # 2:  waist_yaw_joint
            20.0,  # 3:  l_hip_roll_joint
            20.0,  # 4:  r_hip_roll_joint
            10.0,  # 5:  head_yaw_joint
            10.0,  # 6:  l_shoulder_pitch_joint
            10.0,  # 7:  r_shoulder_pitch_joint
            20.0,  # 8:  l_thigh_joint
            20.0,  # 9:  r_thigh_joint
            10.0,  # 10: head_pitch_joint
            10.0,  # 11: l_shoulder_roll_joint
            10.0,  # 12: r_shoulder_roll_joint
            20.0,  # 13: l_calf_joint
            20.0,  # 14: r_calf_joint
            10.0,  # 15: l_upper_arm_joint
            10.0,  # 16: r_upper_arm_joint
            20.0,  # 17: l_ankle_pitch_joint
            20.0,  # 18: r_ankle_pitch_joint
            10.0,  # 19: l_elbow_joint
            10.0,  # 20: r_elbow_joint
            20.0,  # 21: l_ankle_roll_joint
            20.0,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    turn_on_motor_mode = [0x01] * NUM_JOINTS
    mode_pr = 0
    mode_machine = 5

    realsense_depth_link_transform = {
        "translation": (0.0, 0.0, 0.0),
        "rotation": (1.0, 0.0, 0.0, 0.0),
        "parent_frame": "camera_link",
        "child_frame": "realsense_depth_link",
    }
    
   
class PiPlus_S_12L8A0G2H1W:
    """Configuration for PiPlus_S_12L8A0G2H1W humanoid robot.

    This robot has 23 degrees of freedom:
    - 12 leg joints (6 per leg)
    - 1 waist yaw joint
    - 2 head joints
    - 8 arm joints (4 per arm, no wrist)

    sim_joint_names order matches the ACTUAL IsaacSim joint order dumped from
    the training env (play.py robot.data.joint_names), NOT a hand-written guess.
    joint_signs are all +1 by default and MUST be verified per-joint on the
    real robot before trusting them.
    """

    NUM_JOINTS = 23
    NUM_ACTIONS = 23

    # Joint order from IsaacSim during training (verified via play.py dump)
    sim_joint_names = [
        "l_hip_pitch_joint",      # 0
        "r_hip_pitch_joint",      # 1
        "waist_yaw_joint",        # 2
        "l_hip_roll_joint",       # 3
        "r_hip_roll_joint",       # 4
        "head_yaw_joint",         # 5
        "l_shoulder_pitch_joint", # 6
        "r_shoulder_pitch_joint", # 7
        "l_thigh_joint",          # 8
        "r_thigh_joint",          # 9
        "head_pitch_joint",       # 10
        "l_shoulder_roll_joint",  # 11
        "r_shoulder_roll_joint",  # 12
        "l_calf_joint",           # 13
        "r_calf_joint",           # 14
        "l_upper_arm_joint",      # 15
        "r_upper_arm_joint",      # 16
        "l_ankle_pitch_joint",    # 17
        "r_ankle_pitch_joint",    # 18
        "l_elbow_joint",          # 19
        "r_elbow_joint",          # 20
        "l_ankle_roll_joint",     # 21
        "r_ankle_roll_joint",     # 22
    ]

    # Real robot joint names (order from hightorque_midware /joint_states topic)
    # Unchanged: already verified against /joint_states.
    real_joint_names = [
        "l_ankle_roll_joint",     # 0
        "l_ankle_pitch_joint",    # 1
        "l_calf_joint",           # 2
        "l_thigh_joint",          # 3
        "l_hip_roll_joint",       # 4
        "l_hip_pitch_joint",      # 5
        "r_ankle_roll_joint",     # 6
        "r_ankle_pitch_joint",    # 7
        "r_calf_joint",           # 8
        "r_thigh_joint",          # 9
        "r_hip_roll_joint",       # 10
        "r_hip_pitch_joint",      # 11
        "l_shoulder_pitch_joint", # 12
        "l_shoulder_roll_joint",  # 13
        "l_upper_arm_joint",      # 14
        "l_elbow_joint",          # 15
        "r_shoulder_pitch_joint", # 16
        "r_shoulder_roll_joint",  # 17
        "r_upper_arm_joint",      # 18
        "r_elbow_joint",          # 19
        "head_yaw_joint",         # 20
        "head_pitch_joint",       # 21
        "waist_yaw_joint",        # 22
    ]

    # Maps sim_joint_names[i] -> index in real_joint_names
    # Recomputed for the corrected sim order above.
    joint_map = [
        5,   # 0:  l_hip_pitch_joint      -> real[5]
        11,  # 1:  r_hip_pitch_joint      -> real[11]
        22,  # 2:  waist_yaw_joint        -> real[22]
        4,   # 3:  l_hip_roll_joint       -> real[4]
        10,  # 4:  r_hip_roll_joint       -> real[10]
        20,  # 5:  head_yaw_joint         -> real[20]
        12,  # 6:  l_shoulder_pitch_joint -> real[12]
        16,  # 7:  r_shoulder_pitch_joint -> real[16]
        3,   # 8:  l_thigh_joint          -> real[3]
        9,   # 9:  r_thigh_joint          -> real[9]
        21,  # 10: head_pitch_joint       -> real[21]
        13,  # 11: l_shoulder_roll_joint  -> real[13]
        17,  # 12: r_shoulder_roll_joint  -> real[17]
        2,   # 13: l_calf_joint           -> real[2]
        8,   # 14: r_calf_joint           -> real[8]
        14,  # 15: l_upper_arm_joint      -> real[14]
        18,  # 16: r_upper_arm_joint      -> real[18]
        1,   # 17: l_ankle_pitch_joint    -> real[1]
        7,   # 18: r_ankle_pitch_joint    -> real[7]
        15,  # 19: l_elbow_joint          -> real[15]
        19,  # 20: r_elbow_joint          -> real[19]
        0,   # 21: l_ankle_roll_joint     -> real[0]
        6,   # 22: r_ankle_roll_joint     -> real[6]
    ]

    # Joint signs in sim_joint_names order (1=normal, -1=reversed).
    # ALL +1 for now. These are NOT verified — must be confirmed per-joint
    # on the real robot before trusting kick deployment.
    joint_signs = np.array(
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        dtype=np.float32,
    )

    # Joint limits in the corrected sim_joint_names order
    joint_limits_high = np.array(
        [
            2.94,  # 0:  l_hip_pitch_joint
            2.94,  # 1:  r_hip_pitch_joint
            3.14,  # 2:  waist_yaw_joint
            3.14,  # 3:  l_hip_roll_joint
            0.17,  # 4:  r_hip_roll_joint
            3.14,  # 5:  head_yaw_joint
            1.04,  # 6:  l_shoulder_pitch_joint
            1.04,  # 7:  r_shoulder_pitch_joint
            2.87,  # 8:  l_thigh_joint
            2.87,  # 9:  r_thigh_joint
            3.14,  # 10: head_pitch_joint
            3.00,  # 11: l_shoulder_roll_joint
            0.13,  # 12: r_shoulder_roll_joint
            2.35,  # 13: l_calf_joint
            2.35,  # 14: r_calf_joint
            2.16,  # 15: l_upper_arm_joint
            2.16,  # 16: r_upper_arm_joint
            0.92,  # 17: l_ankle_pitch_joint
            0.92,  # 18: r_ankle_pitch_joint
            2.00,  # 19: l_elbow_joint
            2.00,  # 20: r_elbow_joint
            0.78,  # 21: l_ankle_roll_joint
            0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    joint_limits_low = np.array(
        [
            -3.12,  # 0:  l_hip_pitch_joint
            -3.12,  # 1:  r_hip_pitch_joint
            -3.14,  # 2:  waist_yaw_joint
            -0.17,  # 3:  l_hip_roll_joint
            -3.14,  # 4:  r_hip_roll_joint
            -3.14,  # 5:  head_yaw_joint
            -4.18,  # 6:  l_shoulder_pitch_joint
            -4.18,  # 7:  r_shoulder_pitch_joint
            -2.87,  # 8:  l_thigh_joint
            -2.87,  # 9:  r_thigh_joint
            -3.14,  # 10: head_pitch_joint
            -0.13,  # 11: l_shoulder_roll_joint
            -3.00,  # 12: r_shoulder_roll_joint
            -2.14,  # 13: l_calf_joint
            -2.14,  # 14: r_calf_joint
            -2.16,  # 15: l_upper_arm_joint
            -2.16,  # 16: r_upper_arm_joint
            -0.97,  # 17: l_ankle_pitch_joint
            -0.97,  # 18: r_ankle_pitch_joint
            -2.00,  # 19: l_elbow_joint
            -2.00,  # 20: r_elbow_joint
            -0.78,  # 21: l_ankle_roll_joint
            -0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    # Torque limits in the corrected sim_joint_names order
    torque_limits = np.array(
        [
            20.0,  # 0:  l_hip_pitch_joint
            20.0,  # 1:  r_hip_pitch_joint
            20.0,  # 2:  waist_yaw_joint
            20.0,  # 3:  l_hip_roll_joint
            20.0,  # 4:  r_hip_roll_joint
            10.0,  # 5:  head_yaw_joint
            10.0,  # 6:  l_shoulder_pitch_joint
            10.0,  # 7:  r_shoulder_pitch_joint
            20.0,  # 8:  l_thigh_joint
            20.0,  # 9:  r_thigh_joint
            10.0,  # 10: head_pitch_joint
            10.0,  # 11: l_shoulder_roll_joint
            10.0,  # 12: r_shoulder_roll_joint
            20.0,  # 13: l_calf_joint
            20.0,  # 14: r_calf_joint
            10.0,  # 15: l_upper_arm_joint
            10.0,  # 16: r_upper_arm_joint
            20.0,  # 17: l_ankle_pitch_joint
            20.0,  # 18: r_ankle_pitch_joint
            10.0,  # 19: l_elbow_joint
            10.0,  # 20: r_elbow_joint
            20.0,  # 21: l_ankle_roll_joint
            20.0,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    turn_on_motor_mode = [0x01] * NUM_JOINTS
    mode_pr = 0
    mode_machine = 5

    realsense_depth_link_transform = {
        "translation": (0.0, 0.0, 0.0),
        "rotation": (1.0, 0.0, 0.0, 0.0),
        "parent_frame": "camera_link",
        "child_frame": "realsense_depth_link",
    }

class PiPlus_S_12L8A0G2H1W_Lower:
    """Configuration for PiPlus_S_12L8A0G2H1W humanoid robot, lower-body-only policy.

    This config is used with a policy trained on only 12 lower-body joints.
    The policy does NOT control: waist, head, shoulders, arms, elbows.

    Training env: pi_plus_waist_half_parkour_env_cfg.py
    - NUM_ACTIONS = 12 (lower body only)
    - obs joint_pos/vel: all 23 joints in Isaac sim order below

    Isaac sim joint order during training (from env cfg comments, 23 joints total):
      [J00] l_hip_pitch_joint
      [J01] r_hip_pitch_joint
      [J02] waist_yaw_joint
      [J03] l_hip_roll_joint
      [J04] r_hip_roll_joint
      [J05] l_thigh_joint
      [J06] r_thigh_joint
      [J07] l_shoulder_pitch_joint
      [J08] r_shoulder_pitch_joint
      [J09] l_calf_joint
      [J10] r_calf_joint
      [J11] l_shoulder_roll_joint
      [J12] r_shoulder_roll_joint
      [J13] l_ankle_pitch_joint
      [J14] r_ankle_pitch_joint
      [J15] l_upper_arm_joint
      [J16] r_upper_arm_joint
      [J17] l_ankle_roll_joint
      [J18] r_ankle_roll_joint
      [J19] l_elbow_joint
      [J20] r_elbow_joint
      [J21] l_wrist_joint   <- not on real robot, excluded
      [J22] r_wrist_joint   <- not on real robot, excluded

    Action joint order (12, from ActionsCfg joint_names pattern matching):
      [A00] l_hip_pitch_joint   (J00)
      [A01] r_hip_pitch_joint   (J01)
      [A02] l_hip_roll_joint    (J03)
      [A03] r_hip_roll_joint    (J04)
      [A04] l_thigh_joint       (J05)
      [A05] r_thigh_joint       (J06)
      [A06] l_calf_joint        (J09)
      [A07] r_calf_joint        (J10)
      [A08] l_ankle_pitch_joint (J13)
      [A09] r_ankle_pitch_joint (J14)
      [A10] l_ankle_roll_joint  (J17)
      [A11] r_ankle_roll_joint  (J18)
    """

    NUM_JOINTS = 23   # total joints read from robot for obs
    NUM_ACTIONS = 12  # only lower body joints are controlled by policy

    # Isaac sim joint order during training (21 joints on real robot, excludes wrist)
    # This order is used for obs (joint_pos, joint_vel) fed into policy
    sim_joint_names = [
        "l_hip_pitch_joint",      # 0  (J00)
        "r_hip_pitch_joint",      # 1  (J01)
        "waist_yaw_joint",        # 2  (J02)
        "l_hip_roll_joint",       # 3  (J03)
        "r_hip_roll_joint",       # 4  (J04)
        "l_thigh_joint",          # 5  (J05)
        "r_thigh_joint",          # 6  (J06)
        "l_shoulder_pitch_joint", # 7  (J07)
        "r_shoulder_pitch_joint", # 8  (J08)
        "l_calf_joint",           # 9  (J09)
        "r_calf_joint",           # 10 (J10)
        "l_shoulder_roll_joint",  # 11 (J11)
        "r_shoulder_roll_joint",  # 12 (J12)
        "l_ankle_pitch_joint",    # 13 (J13)
        "r_ankle_pitch_joint",    # 14 (J14)
        "l_upper_arm_joint",      # 15 (J15)
        "r_upper_arm_joint",      # 16 (J16)
        "l_ankle_roll_joint",     # 17 (J17)
        "r_ankle_roll_joint",     # 18 (J18)
        "l_elbow_joint",          # 19 (J19)
        "r_elbow_joint",          # 20 (J20)
        "head_yaw_joint",         # 21 (not in training obs, but on real robot)
        "head_pitch_joint",       # 22 (not in training obs, but on real robot)
    ]

    # Real robot joint names (order from hightorque_midware /joint_states topic)
    real_joint_names = [
        "l_ankle_roll_joint",     # 0
        "l_ankle_pitch_joint",    # 1
        "l_calf_joint",           # 2
        "l_thigh_joint",          # 3
        "l_hip_roll_joint",       # 4
        "l_hip_pitch_joint",      # 5
        "r_ankle_roll_joint",     # 6
        "r_ankle_pitch_joint",    # 7
        "r_calf_joint",           # 8
        "r_thigh_joint",          # 9
        "r_hip_roll_joint",       # 10
        "r_hip_pitch_joint",      # 11
        "l_shoulder_pitch_joint", # 12
        "l_shoulder_roll_joint",  # 13
        "l_upper_arm_joint",      # 14
        "l_elbow_joint",          # 15
        "r_shoulder_pitch_joint", # 16
        "r_shoulder_roll_joint",  # 17
        "r_upper_arm_joint",      # 18
        "r_elbow_joint",          # 19
        "head_yaw_joint",         # 20
        "head_pitch_joint",       # 21
        "waist_yaw_joint",        # 22
    ]

    # Maps sim_joint_names[i] -> index in real_joint_names
    joint_map = [
        5,   # 0:  l_hip_pitch_joint      -> real[5]
        11,  # 1:  r_hip_pitch_joint      -> real[11]
        22,  # 2:  waist_yaw_joint        -> real[22]
        4,   # 3:  l_hip_roll_joint       -> real[4]
        10,  # 4:  r_hip_roll_joint       -> real[10]
        3,   # 5:  l_thigh_joint          -> real[3]
        9,   # 6:  r_thigh_joint          -> real[9]
        12,  # 7:  l_shoulder_pitch_joint -> real[12]
        16,  # 8:  r_shoulder_pitch_joint -> real[16]
        2,   # 9:  l_calf_joint           -> real[2]
        8,   # 10: r_calf_joint           -> real[8]
        13,  # 11: l_shoulder_roll_joint  -> real[13]
        17,  # 12: r_shoulder_roll_joint  -> real[17]
        1,   # 13: l_ankle_pitch_joint    -> real[1]
        7,   # 14: r_ankle_pitch_joint    -> real[7]
        14,  # 15: l_upper_arm_joint      -> real[14]
        18,  # 16: r_upper_arm_joint      -> real[18]
        0,   # 17: l_ankle_roll_joint     -> real[0]
        6,   # 18: r_ankle_roll_joint     -> real[6]
        15,  # 19: l_elbow_joint          -> real[15]
        19,  # 20: r_elbow_joint          -> real[19]
        20,  # 21: head_yaw_joint         -> real[20]
        21,  # 22: head_pitch_joint       -> real[21]
    ]

    # Joint signs in sim_joint_names order (1=normal, -1=reversed)
    joint_signs = np.array(
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        dtype=np.float32,
    )

    # Action joint indices in sim_joint_names order
    # policy output action[i] controls sim_joint_names[action_sim_indices[i]]
    action_sim_indices = [0, 1, 3, 4, 5, 6, 9, 10, 13, 14, 17, 18]

    # Joint limits in sim_joint_names order
    joint_limits_high = np.array(
        [
            2.94,  # 0:  l_hip_pitch_joint
            2.94,  # 1:  r_hip_pitch_joint
            3.14,  # 2:  waist_yaw_joint
            3.14,  # 3:  l_hip_roll_joint
            0.17,  # 4:  r_hip_roll_joint
            2.87,  # 5:  l_thigh_joint
            2.87,  # 6:  r_thigh_joint
            1.04,  # 7:  l_shoulder_pitch_joint
            1.04,  # 8:  r_shoulder_pitch_joint
            2.35,  # 9:  l_calf_joint
            2.35,  # 10: r_calf_joint
            3.00,  # 11: l_shoulder_roll_joint
            0.13,  # 12: r_shoulder_roll_joint
            0.92,  # 13: l_ankle_pitch_joint
            0.92,  # 14: r_ankle_pitch_joint
            2.16,  # 15: l_upper_arm_joint
            2.16,  # 16: r_upper_arm_joint
            0.78,  # 17: l_ankle_roll_joint
            0.78,  # 18: r_ankle_roll_joint
            2.00,  # 19: l_elbow_joint
            2.00,  # 20: r_elbow_joint
            3.14,  # 21: head_yaw_joint
            3.14,  # 22: head_pitch_joint
        ],
        dtype=np.float32,
    )

    joint_limits_low = np.array(
        [
            -3.12,  # 0:  l_hip_pitch_joint
            -3.12,  # 1:  r_hip_pitch_joint
            -3.14,  # 2:  waist_yaw_joint
            -0.17,  # 3:  l_hip_roll_joint
            -3.14,  # 4:  r_hip_roll_joint
            -2.87,  # 5:  l_thigh_joint
            -2.87,  # 6:  r_thigh_joint
            -4.18,  # 7:  l_shoulder_pitch_joint
            -4.18,  # 8:  r_shoulder_pitch_joint
            -2.14,  # 9:  l_calf_joint
            -2.14,  # 10: r_calf_joint
            -0.13,  # 11: l_shoulder_roll_joint
            -3.00,  # 12: r_shoulder_roll_joint
            -0.97,  # 13: l_ankle_pitch_joint
            -0.97,  # 14: r_ankle_pitch_joint
            -2.16,  # 15: l_upper_arm_joint
            -2.16,  # 16: r_upper_arm_joint
            -0.78,  # 17: l_ankle_roll_joint
            -0.78,  # 18: r_ankle_roll_joint
            -2.00,  # 19: l_elbow_joint
            -2.00,  # 20: r_elbow_joint
            -3.14,  # 21: head_yaw_joint
            -3.14,  # 22: head_pitch_joint
        ],
        dtype=np.float32,
    )

    # Torque limits in sim_joint_names order
    torque_limits = np.array(
        [
            20.0,  # 0:  l_hip_pitch_joint
            20.0,  # 1:  r_hip_pitch_joint
            20.0,  # 2:  waist_yaw_joint
            20.0,  # 3:  l_hip_roll_joint
            20.0,  # 4:  r_hip_roll_joint
            20.0,  # 5:  l_thigh_joint
            20.0,  # 6:  r_thigh_joint
            10.0,  # 7:  l_shoulder_pitch_joint
            10.0,  # 8:  r_shoulder_pitch_joint
            20.0,  # 9:  l_calf_joint
            20.0,  # 10: r_calf_joint
            10.0,  # 11: l_shoulder_roll_joint
            10.0,  # 12: r_shoulder_roll_joint
            20.0,  # 13: l_ankle_pitch_joint
            20.0,  # 14: r_ankle_pitch_joint
            10.0,  # 15: l_upper_arm_joint
            10.0,  # 16: r_upper_arm_joint
            20.0,  # 17: l_ankle_roll_joint
            20.0,  # 18: r_ankle_roll_joint
            10.0,  # 19: l_elbow_joint
            10.0,  # 20: r_elbow_joint
            10.0,  # 21: head_yaw_joint
            10.0,  # 22: head_pitch_joint
        ],
        dtype=np.float32,
    )

    turn_on_motor_mode = [0x01] * NUM_JOINTS
    mode_pr = 0
    mode_machine = 5

    realsense_depth_link_transform = {
        "translation": (0.0, 0.0, 0.0),
        "rotation": (1.0, 0.0, 0.0, 0.0),
        "parent_frame": "camera_link",
        "child_frame": "realsense_depth_link",
    }



class PiPlus_S_12L8A0G2H1W_OneLeg:
    """Configuration for PiPlus_S_12L8A0G2H1W humanoid robot, one-leg (left leg) policy.
 
    Used with a policy trained on only 6 left-leg joints (左腿单腿站立任务).
    The policy does NOT control: right leg, waist, head, shoulders, arms, elbows.
 
    Training env: HT_lab/tasks/pi_plus_waist_1leg/...
    - NUM_ACTIONS = 6 (left leg only)
    - Observations read all 23 joints in Isaac sim order below
 
    ┌────────────────────────────────────────────────────────────────┐
    │  Isaac sim joint order during training (verified from          │
    │  isaaclab play output, 23 joints total):                       │
    │                                                                │
    │   [J00] l_hip_pitch_joint        ← LEFT LEG                    │
    │   [J01] r_hip_pitch_joint                                      │
    │   [J02] waist_yaw_joint                                        │
    │   [J03] l_hip_roll_joint         ← LEFT LEG                    │
    │   [J04] r_hip_roll_joint                                       │
    │   [J05] head_yaw_joint                                         │
    │   [J06] l_shoulder_pitch_joint                                 │
    │   [J07] r_shoulder_pitch_joint                                 │
    │   [J08] l_thigh_joint            ← LEFT LEG                    │
    │   [J09] r_thigh_joint                                          │
    │   [J10] head_pitch_joint                                       │
    │   [J11] l_shoulder_roll_joint                                  │
    │   [J12] r_shoulder_roll_joint                                  │
    │   [J13] l_calf_joint             ← LEFT LEG                    │
    │   [J14] r_calf_joint                                           │
    │   [J15] l_upper_arm_joint                                      │
    │   [J16] r_upper_arm_joint                                      │
    │   [J17] l_ankle_pitch_joint      ← LEFT LEG                    │
    │   [J18] r_ankle_pitch_joint                                    │
    │   [J19] l_elbow_joint                                          │
    │   [J20] r_elbow_joint                                          │
    │   [J21] l_ankle_roll_joint       ← LEFT LEG                    │
    │   [J22] r_ankle_roll_joint                                     │
    └────────────────────────────────────────────────────────────────┘
 
    ┌────────────────────────────────────────────────────────────────┐
    │  Action joint order (6 left leg joints):                       │
    │                                                                │
    │   [A00] l_hip_pitch_joint   (sim J00)                          │
    │   [A01] l_hip_roll_joint    (sim J03)                          │
    │   [A02] l_thigh_joint       (sim J08)                          │
    │   [A03] l_calf_joint        (sim J13)                          │
    │   [A04] l_ankle_pitch_joint (sim J17)                          │
    │   [A05] l_ankle_roll_joint  (sim J21)                          │
    └────────────────────────────────────────────────────────────────┘
    """
 
    NUM_JOINTS = 23   # observe all 23 joints
    NUM_ACTIONS = 6   # only left leg controlled by policy
 
    # Isaac sim joint order (verified from training)
    # !!! 这跟旧的 PiPlus_S_12L8A0G2H1W 顺序不一样 !!!
    sim_joint_names = [
        "l_hip_pitch_joint",      # 0
        "r_hip_pitch_joint",      # 1
        "waist_yaw_joint",        # 2
        "l_hip_roll_joint",       # 3
        "r_hip_roll_joint",       # 4
        "head_yaw_joint",         # 5
        "l_shoulder_pitch_joint", # 6
        "r_shoulder_pitch_joint", # 7
        "l_thigh_joint",          # 8
        "r_thigh_joint",          # 9
        "head_pitch_joint",       # 10
        "l_shoulder_roll_joint",  # 11
        "r_shoulder_roll_joint",  # 12
        "l_calf_joint",           # 13
        "r_calf_joint",           # 14
        "l_upper_arm_joint",      # 15
        "r_upper_arm_joint",      # 16
        "l_ankle_pitch_joint",    # 17
        "r_ankle_pitch_joint",    # 18
        "l_elbow_joint",          # 19
        "r_elbow_joint",          # 20
        "l_ankle_roll_joint",     # 21
        "r_ankle_roll_joint",     # 22
    ]
 
    # Real robot joint names (order from hightorque_midware /joint_states topic)
    # !!! 跟旧 PiPlus_S_12L8A0G2H1W 一致 !!!
    real_joint_names = [
        "l_ankle_roll_joint",     # 0
        "l_ankle_pitch_joint",    # 1
        "l_calf_joint",           # 2
        "l_thigh_joint",          # 3
        "l_hip_roll_joint",       # 4
        "l_hip_pitch_joint",      # 5
        "r_ankle_roll_joint",     # 6
        "r_ankle_pitch_joint",    # 7
        "r_calf_joint",           # 8
        "r_thigh_joint",          # 9
        "r_hip_roll_joint",       # 10
        "r_hip_pitch_joint",      # 11
        "l_shoulder_pitch_joint", # 12
        "l_shoulder_roll_joint",  # 13
        "l_upper_arm_joint",      # 14
        "l_elbow_joint",          # 15
        "r_shoulder_pitch_joint", # 16
        "r_shoulder_roll_joint",  # 17
        "r_upper_arm_joint",      # 18
        "r_elbow_joint",          # 19
        "head_yaw_joint",         # 20
        "head_pitch_joint",       # 21
        "waist_yaw_joint",        # 22
    ]
 
    # Maps sim_joint_names[i] -> index in real_joint_names
    # 重新按新 sim 顺序计算（用 real_joint_names.index 查表）
    joint_map = [
        5,   # 0:  l_hip_pitch_joint      -> real[5]
        11,  # 1:  r_hip_pitch_joint      -> real[11]
        22,  # 2:  waist_yaw_joint        -> real[22]
        4,   # 3:  l_hip_roll_joint       -> real[4]
        10,  # 4:  r_hip_roll_joint       -> real[10]
        20,  # 5:  head_yaw_joint         -> real[20]
        12,  # 6:  l_shoulder_pitch_joint -> real[12]
        16,  # 7:  r_shoulder_pitch_joint -> real[16]
        3,   # 8:  l_thigh_joint          -> real[3]
        9,   # 9:  r_thigh_joint          -> real[9]
        21,  # 10: head_pitch_joint       -> real[21]
        13,  # 11: l_shoulder_roll_joint  -> real[13]
        17,  # 12: r_shoulder_roll_joint  -> real[17]
        2,   # 13: l_calf_joint           -> real[2]
        8,   # 14: r_calf_joint           -> real[8]
        14,  # 15: l_upper_arm_joint      -> real[14]
        18,  # 16: r_upper_arm_joint      -> real[18]
        1,   # 17: l_ankle_pitch_joint    -> real[1]
        7,   # 18: r_ankle_pitch_joint    -> real[7]
        15,  # 19: l_elbow_joint          -> real[15]
        19,  # 20: r_elbow_joint          -> real[19]
        0,   # 21: l_ankle_roll_joint     -> real[0]
        6,   # 22: r_ankle_roll_joint     -> real[6]
    ]
 
    # policy 输出 action[i] 对应 sim_joint_names 中的索引
    # 6 个左腿关节，按训练 env.yaml 的 ActionsCfg.joint_names 顺序：
    #   l_hip_pitch -> l_hip_roll -> l_thigh -> l_calf -> l_ankle_pitch -> l_ankle_roll
    action_sim_indices = [0, 3, 8, 13, 17, 21]
 
    # Joint signs in sim_joint_names order (1=normal, -1=reversed)
    joint_signs = np.array(
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        dtype=np.float32,
    )
 
    # Joint limits in sim_joint_names order
    joint_limits_high = np.array(
        [
            2.94,  # 0:  l_hip_pitch_joint
            2.94,  # 1:  r_hip_pitch_joint
            3.14,  # 2:  waist_yaw_joint
            3.14,  # 3:  l_hip_roll_joint
            0.17,  # 4:  r_hip_roll_joint
            3.14,  # 5:  head_yaw_joint
            1.04,  # 6:  l_shoulder_pitch_joint
            1.04,  # 7:  r_shoulder_pitch_joint
            2.87,  # 8:  l_thigh_joint
            2.87,  # 9:  r_thigh_joint
            3.14,  # 10: head_pitch_joint
            3.00,  # 11: l_shoulder_roll_joint
            0.13,  # 12: r_shoulder_roll_joint
            2.35,  # 13: l_calf_joint
            2.35,  # 14: r_calf_joint
            2.16,  # 15: l_upper_arm_joint
            2.16,  # 16: r_upper_arm_joint
            0.92,  # 17: l_ankle_pitch_joint
            0.92,  # 18: r_ankle_pitch_joint
            2.00,  # 19: l_elbow_joint
            2.00,  # 20: r_elbow_joint
            0.78,  # 21: l_ankle_roll_joint
            0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )
 
    joint_limits_low = np.array(
        [
            -3.12,  # 0:  l_hip_pitch_joint
            -3.12,  # 1:  r_hip_pitch_joint
            -3.14,  # 2:  waist_yaw_joint
            -0.17,  # 3:  l_hip_roll_joint
            -3.14,  # 4:  r_hip_roll_joint
            -3.14,  # 5:  head_yaw_joint
            -4.18,  # 6:  l_shoulder_pitch_joint
            -4.18,  # 7:  r_shoulder_pitch_joint
            -2.87,  # 8:  l_thigh_joint
            -2.87,  # 9:  r_thigh_joint
            -3.14,  # 10: head_pitch_joint
            -0.13,  # 11: l_shoulder_roll_joint
            -3.00,  # 12: r_shoulder_roll_joint
            -2.14,  # 13: l_calf_joint
            -2.14,  # 14: r_calf_joint
            -2.16,  # 15: l_upper_arm_joint
            -2.16,  # 16: r_upper_arm_joint
            -0.97,  # 17: l_ankle_pitch_joint
            -0.97,  # 18: r_ankle_pitch_joint
            -2.00,  # 19: l_elbow_joint
            -2.00,  # 20: r_elbow_joint
            -0.78,  # 21: l_ankle_roll_joint
            -0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )
 
    # Torque limits in sim_joint_names order
    torque_limits = np.array(
        [
            20.0,  # 0:  l_hip_pitch_joint
            20.0,  # 1:  r_hip_pitch_joint
            20.0,  # 2:  waist_yaw_joint
            20.0,  # 3:  l_hip_roll_joint
            20.0,  # 4:  r_hip_roll_joint
            10.0,  # 5:  head_yaw_joint
            10.0,  # 6:  l_shoulder_pitch_joint
            10.0,  # 7:  r_shoulder_pitch_joint
            20.0,  # 8:  l_thigh_joint
            20.0,  # 9:  r_thigh_joint
            10.0,  # 10: head_pitch_joint
            10.0,  # 11: l_shoulder_roll_joint
            10.0,  # 12: r_shoulder_roll_joint
            20.0,  # 13: l_calf_joint
            20.0,  # 14: r_calf_joint
            10.0,  # 15: l_upper_arm_joint
            10.0,  # 16: r_upper_arm_joint
            20.0,  # 17: l_ankle_pitch_joint
            20.0,  # 18: r_ankle_pitch_joint
            10.0,  # 19: l_elbow_joint
            10.0,  # 20: r_elbow_joint
            20.0,  # 21: l_ankle_roll_joint
            20.0,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )
 
    turn_on_motor_mode = [0x01] * NUM_JOINTS
    mode_pr = 0
    mode_machine = 5
 
    realsense_depth_link_transform = {
        "translation": (0.0, 0.0, 0.0),
        "rotation": (1.0, 0.0, 0.0, 0.0),
        "parent_frame": "camera_link",
        "child_frame": "realsense_depth_link",
    }

class PiPlus_S_12L8A0G2H0W:
    """Configuration for PiPlus_S_12L8A0G2H0W humanoid robot (NO WAIST).
 
    22 degrees of freedom:
      - 12 leg joints (6 per leg)
      - 2 head joints (yaw + pitch)
      - 8 arm joints (4 per arm, no wrist)
      - NO waist yaw joint  (this is the only difference vs the H1W robot)
 
    sim_joint_names is the ACTUAL IsaacSim joint order taken verbatim from the
    training asset cfg (HT_lab/assets/PiPlus_S_12L8A0G2H0W.py docstring), and
    cross-checked against the 22-entry symmetric-augmentation mapping in that
    same file. Do NOT reorder it.
 
    !!! TWO THINGS STILL NEED ON-HARDWARE VERIFICATION before nodryrun !!!
      1. real_joint_names / joint_map — assumed to equal the H1W /joint_states
         order with the waist_yaw motor removed. Confirm with
         verify_joint_order.py against the real /joint_states topic.
      2. joint_signs — all +1 here, NOT verified. Confirm per-joint on the
         real robot (same as every other PiPlus config in this file).
    """
 
    NUM_JOINTS = 22
    NUM_ACTIONS = 22
 
    # IsaacSim joint order during training (authoritative — from the asset cfg)
    sim_joint_names = [
        "head_yaw_joint",          # 0
        "l_hip_pitch_joint",       # 1
        "l_shoulder_pitch_joint",  # 2
        "r_hip_pitch_joint",       # 3
        "r_shoulder_pitch_joint",  # 4
        "head_pitch_joint",        # 5
        "l_hip_roll_joint",        # 6
        "l_shoulder_roll_joint",   # 7
        "r_hip_roll_joint",        # 8
        "r_shoulder_roll_joint",   # 9
        "l_thigh_joint",           # 10
        "l_upper_arm_joint",       # 11
        "r_thigh_joint",           # 12
        "r_upper_arm_joint",       # 13
        "l_calf_joint",            # 14
        "l_elbow_joint",           # 15
        "r_calf_joint",            # 16
        "r_elbow_joint",           # 17
        "l_ankle_pitch_joint",     # 18
        "r_ankle_pitch_joint",     # 19
        "l_ankle_roll_joint",      # 20
        "r_ankle_roll_joint",      # 21
    ]
 
    # Real robot joint order from hightorque_midware /joint_states.
    # ASSUMED = H1W order minus waist_yaw. VERIFY ON HARDWARE.
    real_joint_names = [
        "l_ankle_roll_joint",      # 0
        "l_ankle_pitch_joint",     # 1
        "l_calf_joint",            # 2
        "l_thigh_joint",           # 3
        "l_hip_roll_joint",        # 4
        "l_hip_pitch_joint",       # 5
        "r_ankle_roll_joint",      # 6
        "r_ankle_pitch_joint",     # 7
        "r_calf_joint",            # 8
        "r_thigh_joint",           # 9
        "r_hip_roll_joint",        # 10
        "r_hip_pitch_joint",       # 11
        "l_shoulder_pitch_joint",  # 12
        "l_shoulder_roll_joint",   # 13
        "l_upper_arm_joint",       # 14
        "l_elbow_joint",           # 15
        "r_shoulder_pitch_joint",  # 16
        "r_shoulder_roll_joint",   # 17
        "r_upper_arm_joint",       # 18
        "r_elbow_joint",           # 19
        "head_yaw_joint",          # 20
        "head_pitch_joint",        # 21
    ]
 
    # Maps sim_joint_names[i] -> index in real_joint_names
    # (computed: real_joint_names.index(sim_joint_names[i]))
    joint_map = [
        20,  #  0: head_yaw_joint          -> real[20]
        5,   #  1: l_hip_pitch_joint       -> real[5]
        12,  #  2: l_shoulder_pitch_joint  -> real[12]
        11,  #  3: r_hip_pitch_joint       -> real[11]
        16,  #  4: r_shoulder_pitch_joint  -> real[16]
        21,  #  5: head_pitch_joint        -> real[21]
        4,   #  6: l_hip_roll_joint        -> real[4]
        13,  #  7: l_shoulder_roll_joint   -> real[13]
        10,  #  8: r_hip_roll_joint        -> real[10]
        17,  #  9: r_shoulder_roll_joint   -> real[17]
        3,   # 10: l_thigh_joint           -> real[3]
        14,  # 11: l_upper_arm_joint       -> real[14]
        9,   # 12: r_thigh_joint           -> real[9]
        18,  # 13: r_upper_arm_joint       -> real[18]
        2,   # 14: l_calf_joint            -> real[2]
        15,  # 15: l_elbow_joint           -> real[15]
        8,   # 16: r_calf_joint            -> real[8]
        19,  # 17: r_elbow_joint           -> real[19]
        1,   # 18: l_ankle_pitch_joint     -> real[1]
        7,   # 19: r_ankle_pitch_joint     -> real[7]
        0,   # 20: l_ankle_roll_joint      -> real[0]
        6,   # 21: r_ankle_roll_joint      -> real[6]
    ]
 
    # Joint signs in sim order (1=normal, -1=reversed). ALL +1, NOT verified.
    joint_signs = np.array(
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        dtype=np.float32,
    )
 
    # Joint limits in sim_joint_names order
    joint_limits_high = np.array(
        [
            3.14,  #  0: head_yaw_joint
            2.94,  #  1: l_hip_pitch_joint
            1.04,  #  2: l_shoulder_pitch_joint
            2.94,  #  3: r_hip_pitch_joint
            1.04,  #  4: r_shoulder_pitch_joint
            3.14,  #  5: head_pitch_joint
            3.14,  #  6: l_hip_roll_joint
            3.00,  #  7: l_shoulder_roll_joint
            0.17,  #  8: r_hip_roll_joint
            0.13,  #  9: r_shoulder_roll_joint
            2.87,  # 10: l_thigh_joint
            2.16,  # 11: l_upper_arm_joint
            2.87,  # 12: r_thigh_joint
            2.16,  # 13: r_upper_arm_joint
            2.35,  # 14: l_calf_joint
            2.00,  # 15: l_elbow_joint
            2.35,  # 16: r_calf_joint
            2.00,  # 17: r_elbow_joint
            0.92,  # 18: l_ankle_pitch_joint
            0.92,  # 19: r_ankle_pitch_joint
            0.78,  # 20: l_ankle_roll_joint
            0.78,  # 21: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )
 
    joint_limits_low = np.array(
        [
            -3.14,  #  0: head_yaw_joint
            -3.12,  #  1: l_hip_pitch_joint
            -4.18,  #  2: l_shoulder_pitch_joint
            -3.12,  #  3: r_hip_pitch_joint
            -4.18,  #  4: r_shoulder_pitch_joint
            -3.14,  #  5: head_pitch_joint
            -0.17,  #  6: l_hip_roll_joint
            -0.13,  #  7: l_shoulder_roll_joint
            -3.14,  #  8: r_hip_roll_joint
            -3.00,  #  9: r_shoulder_roll_joint
            -2.87,  # 10: l_thigh_joint
            -2.16,  # 11: l_upper_arm_joint
            -2.87,  # 12: r_thigh_joint
            -2.16,  # 13: r_upper_arm_joint
            -2.14,  # 14: l_calf_joint
            -2.00,  # 15: l_elbow_joint
            -2.14,  # 16: r_calf_joint
            -2.00,  # 17: r_elbow_joint
            -0.97,  # 18: l_ankle_pitch_joint
            -0.97,  # 19: r_ankle_pitch_joint
            -0.78,  # 20: l_ankle_roll_joint
            -0.78,  # 21: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )
 
    # Torque limits in sim_joint_names order
    torque_limits = np.array(
        [
            10.0,  #  0: head_yaw_joint
            20.0,  #  1: l_hip_pitch_joint
            10.0,  #  2: l_shoulder_pitch_joint
            20.0,  #  3: r_hip_pitch_joint
            10.0,  #  4: r_shoulder_pitch_joint
            10.0,  #  5: head_pitch_joint
            20.0,  #  6: l_hip_roll_joint
            10.0,  #  7: l_shoulder_roll_joint
            20.0,  #  8: r_hip_roll_joint
            10.0,  #  9: r_shoulder_roll_joint
            20.0,  # 10: l_thigh_joint
            10.0,  # 11: l_upper_arm_joint
            20.0,  # 12: r_thigh_joint
            10.0,  # 13: r_upper_arm_joint
            20.0,  # 14: l_calf_joint
            10.0,  # 15: l_elbow_joint
            20.0,  # 16: r_calf_joint
            10.0,  # 17: r_elbow_joint
            20.0,  # 18: l_ankle_pitch_joint
            20.0,  # 19: r_ankle_pitch_joint
            20.0,  # 20: l_ankle_roll_joint
            20.0,  # 21: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )
 
    turn_on_motor_mode = [0x01] * NUM_JOINTS
    mode_pr = 0
    mode_machine = 5
 
    realsense_depth_link_transform = {
        "translation": (0.0, 0.0, 0.0),
        "rotation": (1.0, 0.0, 0.0, 0.0),
        "parent_frame": "camera_link",
        "child_frame": "realsense_depth_link",
    }

#end


class PiPlus_S_12L8A0G2H0W_FallRecovery(PiPlus_S_12L8A0G2H0W):
    """H0W joint order used by the fall-recovery policy export.

    The existing PiPlus_S_12L8A0G2H0W order is kept unchanged for soccer/kick
    policies. This variant matches the fall_recovery branch's tracker policy
    order and reorders limits/torques from the base H0W config by joint name.
    """

    sim_joint_names = [
        "l_hip_pitch_joint",
        "r_hip_pitch_joint",
        "l_hip_roll_joint",
        "r_hip_roll_joint",
        "head_yaw_joint",
        "l_shoulder_pitch_joint",
        "r_shoulder_pitch_joint",
        "l_thigh_joint",
        "r_thigh_joint",
        "head_pitch_joint",
        "l_shoulder_roll_joint",
        "r_shoulder_roll_joint",
        "l_calf_joint",
        "r_calf_joint",
        "l_upper_arm_joint",
        "r_upper_arm_joint",
        "l_ankle_pitch_joint",
        "r_ankle_pitch_joint",
        "l_elbow_joint",
        "r_elbow_joint",
        "l_ankle_roll_joint",
        "r_ankle_roll_joint",
    ]

    joint_map = []
    _base_indices = []
    for _joint_name in sim_joint_names:
        joint_map.append(PiPlus_S_12L8A0G2H0W.real_joint_names.index(_joint_name))
        _base_indices.append(PiPlus_S_12L8A0G2H0W.sim_joint_names.index(_joint_name))
    del _joint_name
    joint_signs = PiPlus_S_12L8A0G2H0W.joint_signs[_base_indices].copy()
    joint_limits_high = PiPlus_S_12L8A0G2H0W.joint_limits_high[_base_indices].copy()
    joint_limits_low = PiPlus_S_12L8A0G2H0W.joint_limits_low[_base_indices].copy()
    torque_limits = PiPlus_S_12L8A0G2H0W.torque_limits[_base_indices].copy()
