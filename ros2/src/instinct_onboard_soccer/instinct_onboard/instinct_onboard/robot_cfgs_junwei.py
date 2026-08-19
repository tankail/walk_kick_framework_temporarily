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


class PiPlus_S_12L8A0G2H1W:
    """Configuration for PiPlus_S_12L8A0G2H1W humanoid robot.

    This robot has 23 degrees of freedom:
    - 12 leg joints (6 per leg)
    - 1 waist yaw joint
    - 8 arm joints (4 per arm)
    - 2 head joints
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

    joint_signs = np.array(
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
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


class PiPlus_S_12L8A0G2H1W_LSE:
    """Configuration for PiPlus_S_12L8A0G2H1W_LSE humanoid robot.

    This robot has 23 degrees of freedom:
    - 12 leg joints (6 per leg)
    - 1 waist yaw joint
    - 8 arm joints (4 per arm)
    - 2 head joints
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

    joint_signs = np.array(
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        dtype=np.float32,
    )

    # Joint limits (IsaacSim order)
    joint_limits_high = np.array(
        [
            2.94,  # 0: l_hip_pitch_joint
            2.94,  # 1: r_hip_pitch_joint
            2.70,  # 2: waist_yaw_joint
            3.14,  # 3: l_hip_roll_joint
            0.17,  # 4: r_hip_roll_joint
            3.14,  # 5: head_yaw_joint
            1.04,  # 6: l_shoulder_pitch_joint
            1.04,  # 7: r_shoulder_pitch_joint
            2.87,  # 8: l_thigh_joint
            2.87,  # 9: r_thigh_joint
            0.88,  # 10: head_pitch_joint
            3.34,  # 11: l_shoulder_roll_joint
            0.13,  # 12: r_shoulder_roll_joint
            2.35,  # 13: l_calf_joint
            2.35,  # 14: r_calf_joint
            2.16,  # 15: l_upper_arm_joint
            2.16,  # 16: r_upper_arm_joint
            0.97,  # 17: l_ankle_pitch_joint
            0.97,  # 18: r_ankle_pitch_joint
            1.73,  # 19: l_elbow_joint
            1.73,  # 20: r_elbow_joint
            0.78,  # 21: l_ankle_roll_joint
            0.78,  # 22: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    joint_limits_low = np.array(
        [
            -3.12,  # 0: l_hip_pitch_joint
            -3.12,  # 1: r_hip_pitch_joint
            -2.70,  # 2: waist_yaw_joint
            -0.17,  # 3: l_hip_roll_joint
            -3.14,  # 4: r_hip_roll_joint
            -3.14,  # 5: head_yaw_joint
            -4.18,  # 6: l_shoulder_pitch_joint
            -4.18,  # 7: r_shoulder_pitch_joint
            -2.87,  # 8: l_thigh_joint
            -2.87,  # 9: r_thigh_joint
            -0.94,  # 10: head_pitch_joint
            -0.13,  # 11: l_shoulder_roll_joint
            -3.34,  # 12: r_shoulder_roll_joint
            -2.14,  # 13: l_calf_joint
            -2.14,  # 14: r_calf_joint
            -2.16,  # 15: l_upper_arm_joint
            -2.16,  # 16: r_upper_arm_joint
            -0.92,  # 17: l_ankle_pitch_joint
            -0.92,  # 18: r_ankle_pitch_joint
            -1.73,  # 19: l_elbow_joint
            -1.73,  # 20: r_elbow_joint
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

# class PiPlus_S_12L8A0G2H0W:

#     """Configuration for PiPlus_S_12L8A0G2H0W humanoid robot.

#     This robot has 22 degrees of freedom:
#     - 12 leg joints (6 per leg)
#     - 8 arm joints (4 per arm)
#     - 2 head joints
#     """

#     NUM_JOINTS = 22
#     NUM_ACTIONS = 22

#     # Joint order from IsaacSim (this is the simulation order)
#     sim_joint_names = [  # NOTE: order matters. This list is the order in simulation.
#         "head_yaw_joint",  # 0
#         "l_hip_pitch_joint",  # 1
#         "l_shoulder_pitch_joint",  # 2
#         "r_hip_pitch_joint",  # 3
#         "r_shoulder_pitch_joint",  # 4
#         "head_pitch_joint",  # 5
#         "l_hip_roll_joint",  # 6
#         "l_shoulder_roll_joint",  # 7
#         "r_hip_roll_joint",  # 8
#         "r_shoulder_roll_joint",  # 9
#         "l_thigh_joint",  # 10
#         "l_upper_arm_joint",  # 11
#         "r_thigh_joint",  # 12
#         "r_upper_arm_joint",  # 13
#         "l_calf_joint",  # 14
#         "l_elbow_joint",  # 15
#         "r_calf_joint",  # 16
#         "r_elbow_joint",  # 17
#         "l_ankle_pitch_joint",  # 18
#         "r_ankle_pitch_joint",  # 19
#         "l_ankle_roll_joint",  # 20
#         "r_ankle_roll_joint",  # 21
#     ]

#     # Joint order mapping from simulation to real robot
#     # Maps sim_joint_names[i] to the index in real_joint_names (actual joint_states order)
#     joint_map = [
#         20,  # 0: head_yaw_joint -> real[20]
#         5,  # 1: l_hip_pitch_joint -> real[5]
#         12,  # 2: l_shoulder_pitch_joint -> real[12]
#         11,  # 3: r_hip_pitch_joint -> real[11]
#         16,  # 4: r_shoulder_pitch_joint -> real[16]
#         21,  # 5: head_pitch_joint -> real[21]
#         4,  # 6: l_hip_roll_joint -> real[4]
#         13,  # 7: l_shoulder_roll_joint -> real[13]
#         10,  # 8: r_hip_roll_joint -> real[10]
#         17,  # 9: r_shoulder_roll_joint -> real[17]
#         3,  # 10: l_thigh_joint -> real[3]
#         14,  # 11: l_upper_arm_joint -> real[14]
#         9,  # 12: r_thigh_joint -> real[9]
#         18,  # 13: r_upper_arm_joint -> real[18]
#         2,  # 14: l_calf_joint -> real[2]
#         15,  # 15: l_elbow_joint -> real[15]
#         8,  # 16: r_calf_joint -> real[8]
#         19,  # 17: r_elbow_joint -> real[19]
#         1,  # 18: l_ankle_pitch_joint -> real[1]
#         7,  # 19: r_ankle_pitch_joint -> real[7]
#         0,  # 20: l_ankle_roll_joint -> real[0]
#         6,  # 21: r_ankle_roll_joint -> real[6]
#     ]

#     # Real robot joint names (actual order from hightorque_midware joint_states)
#     real_joint_names = [
#         "l_ankle_roll_joint",  # 0
#         "l_ankle_pitch_joint",  # 1
#         "l_calf_joint",  # 2
#         "l_thigh_joint",  # 3
#         "l_hip_roll_joint",  # 4
#         "l_hip_pitch_joint",  # 5
#         "r_ankle_roll_joint",  # 6
#         "r_ankle_pitch_joint",  # 7
#         "r_calf_joint",  # 8
#         "r_thigh_joint",  # 9
#         "r_hip_roll_joint",  # 10
#         "r_hip_pitch_joint",  # 11
#         "l_shoulder_pitch_joint",  # 12
#         "l_shoulder_roll_joint",  # 13
#         "l_upper_arm_joint",  # 14
#         "l_elbow_joint",  # 15
#         "r_shoulder_pitch_joint",  # 16
#         "r_shoulder_roll_joint",  # 17
#         "r_upper_arm_joint",  # 18
#         "r_elbow_joint",  # 19
#         "head_yaw_joint",  # 20
#         "head_pitch_joint",  # 21
#     ]

#     joint_signs = np.array(
#         [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
#         dtype=np.float32,
#     )

#     # Joint limits (IsaacSim order)
#     joint_limits_high = np.array(
#         [
#             3.14,  # 0: head_yaw_joint
#             2.94,  # 1: l_hip_pitch_joint
#             1.04,  # 2: l_shoulder_pitch_joint
#             2.94,  # 3: r_hip_pitch_joint
#             1.04,  # 4: r_shoulder_pitch_joint
#             0.88,  # 5: head_pitch_joint
#             3.14,  # 6: l_hip_roll_joint
#             3.34,  # 7: l_shoulder_roll_joint
#             0.17,  # 8: r_hip_roll_joint
#             0.13,  # 9: r_shoulder_roll_joint
#             2.87,  # 10: l_thigh_joint
#             2.16,  # 11: l_upper_arm_joint
#             2.87,  # 12: r_thigh_joint
#             2.16,  # 13: r_upper_arm_joint
#             2.35,  # 14: l_calf_joint
#             1.73,  # 15: l_elbow_joint
#             2.35,  # 16: r_calf_joint
#             1.73,  # 17: r_elbow_joint
#             0.97,  # 18: l_ankle_pitch_joint
#             0.97,  # 19: r_ankle_pitch_joint
#             0.78,  # 20: l_ankle_roll_joint
#             0.78,  # 21: r_ankle_roll_joint
#         ],
#         dtype=np.float32,
#     )

#     joint_limits_low = np.array(
#         [
#             -3.14,  # 0: head_yaw_joint
#             -3.12,  # 1: l_hip_pitch_joint
#             -4.18,  # 2: l_shoulder_pitch_joint
#             -3.12,  # 3: r_hip_pitch_joint
#             -4.18,  # 4: r_shoulder_pitch_joint
#             -0.94,  # 5: head_pitch_joint
#             -0.17,  # 6: l_hip_roll_joint
#             -0.13,  # 7: l_shoulder_roll_joint
#             -3.14,  # 8: r_hip_roll_joint
#             -3.34,  # 9: r_shoulder_roll_joint
#             -2.87,  # 10: l_thigh_joint
#             -2.16,  # 11: l_upper_arm_joint
#             -2.87,  # 12: r_thigh_joint
#             -2.16,  # 13: r_upper_arm_joint
#             -2.14,  # 14: l_calf_joint
#             -1.73,  # 15: l_elbow_joint
#             -2.14,  # 16: r_calf_joint
#             -1.73,  # 17: r_elbow_joint
#             -0.92,  # 18: l_ankle_pitch_joint
#             -0.92,  # 19: r_ankle_pitch_joint
#             -0.78,  # 20: l_ankle_roll_joint
#             -0.78,  # 21: r_ankle_roll_joint
#         ],
#         dtype=np.float32,
#     )

#     # Torque limits (IsaacSim order)
#     torque_limits = np.array(
#         [
#             10.0,  # 0: head_yaw_joint
#             20.0,  # 1: l_hip_pitch_joint
#             10.0,  # 2: l_shoulder_pitch_joint
#             20.0,  # 3: r_hip_pitch_joint
#             10.0,  # 4: r_shoulder_pitch_joint
#             10.0,  # 5: head_pitch_joint
#             20.0,  # 6: l_hip_roll_joint
#             10.0,  # 7: l_shoulder_roll_joint
#             20.0,  # 8: r_hip_roll_joint
#             10.0,  # 9: r_shoulder_roll_joint
#             20.0,  # 10: l_thigh_joint
#             10.0,  # 11: l_upper_arm_joint
#             20.0,  # 12: r_thigh_joint
#             10.0,  # 13: r_upper_arm_joint
#             20.0,  # 14: l_calf_joint
#             10.0,  # 15: l_elbow_joint
#             20.0,  # 16: r_calf_joint
#             10.0,  # 17: r_elbow_joint
#             20.0,  # 18: l_ankle_pitch_joint
#             20.0,  # 19: r_ankle_pitch_joint
#             20.0,  # 20: l_ankle_roll_joint
#             20.0,  # 21: r_ankle_roll_joint
#         ],
#         dtype=np.float32,
#     )

#     turn_on_motor_mode = [0x01] * NUM_JOINTS
#     mode_pr = 0
#     mode_machine = 5

#     # RealSense depth camera transform - depth_link is at camera_link with zero offset
#     realsense_depth_link_transform = {
#         "translation": (0.0, 0.0, 0.0),  # Zero offset from camera_link
#         "rotation": (1.0, 0.0, 0.0, 0.0),  # No rotation (identity quaternion)
#         "parent_frame": "camera_link",
#         "child_frame": "realsense_depth_link",
#     }

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
    # Joint limits in sim_joint_names order  (== URDF)
    joint_limits_high = np.array(
        [
            1.43,   #  0: head_yaw_joint          (was 3.14)
            1.76,   #  1: l_hip_pitch_joint       (was 2.94)
            1.04,   #  2: l_shoulder_pitch_joint
            1.76,   #  3: r_hip_pitch_joint       (was 2.94)
            1.04,   #  4: r_shoulder_pitch_joint
            0.88,   #  5: head_pitch_joint        (was 3.14)
            2.46,   #  6: l_hip_roll_joint        (was 3.14)
            3.34,   #  7: l_shoulder_roll_joint   (was 3.00)
            0.17,   #  8: r_hip_roll_joint
            0.13,   #  9: r_shoulder_roll_joint
            2.87,   # 10: l_thigh_joint
            2.16,   # 11: l_upper_arm_joint
            2.87,   # 12: r_thigh_joint
            2.16,   # 13: r_upper_arm_joint
            2.35,   # 14: l_calf_joint
            2.00,   # 15: l_elbow_joint
            2.35,   # 16: r_calf_joint
            2.00,   # 17: r_elbow_joint
            0.92,   # 18: l_ankle_pitch_joint
            0.92,   # 19: r_ankle_pitch_joint
            0.78,   # 20: l_ankle_roll_joint
            0.78,   # 21: r_ankle_roll_joint
        ],
        dtype=np.float32,
    )

    joint_limits_low = np.array(
        [
            -1.43,  #  0: head_yaw_joint          (was -3.14)
            -1.76,  #  1: l_hip_pitch_joint       (was -3.12)
            -4.18,  #  2: l_shoulder_pitch_joint
            -1.76,  #  3: r_hip_pitch_joint       (was -3.12)
            -4.18,  #  4: r_shoulder_pitch_joint
            -0.94,  #  5: head_pitch_joint        (was -3.14)
            -0.17,  #  6: l_hip_roll_joint
            -0.13,  #  7: l_shoulder_roll_joint
            -2.46,  #  8: r_hip_roll_joint        (was -3.14)
            -3.34,  #  9: r_shoulder_roll_joint   (was -3.00)
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

    # Torque limits in sim_joint_names order  (== URDF effort)
    torque_limits = np.array(
        [
            3.0,    #  0: head_yaw_joint          (was 10.0)
            20.0,   #  1: l_hip_pitch_joint
            10.0,   #  2: l_shoulder_pitch_joint
            20.0,   #  3: r_hip_pitch_joint
            10.0,   #  4: r_shoulder_pitch_joint
            3.6,    #  5: head_pitch_joint        (was 10.0)
            20.0,   #  6: l_hip_roll_joint
            10.0,   #  7: l_shoulder_roll_joint
            20.0,   #  8: r_hip_roll_joint
            10.0,   #  9: r_shoulder_roll_joint
            20.0,   # 10: l_thigh_joint
            10.0,   # 11: l_upper_arm_joint
            20.0,   # 12: r_thigh_joint
            10.0,   # 13: r_upper_arm_joint
            20.0,   # 14: l_calf_joint
            10.0,   # 15: l_elbow_joint
            20.0,   # 16: r_calf_joint
            10.0,   # 17: r_elbow_joint
            20.0,   # 18: l_ankle_pitch_joint
            20.0,   # 19: r_ankle_pitch_joint
            20.0,   # 20: l_ankle_roll_joint
            20.0,   # 21: r_ankle_roll_joint
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

