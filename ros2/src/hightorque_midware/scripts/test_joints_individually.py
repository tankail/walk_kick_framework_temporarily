#!/usr/bin/env python3
"""
测试脚本：逐个控制指定关节
每个关节先移动+0.2，保持1秒，然后回到原位置
"""

import time

import rclpy
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ReleaseControl, RequestControl
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointTester(Node):
    def __init__(self):
        super().__init__('joint_tester')

        # 指定要测试的关节顺序
        self.target_joints = [
            'l_hip_pitch_joint',
            'l_shoulder_pitch_joint',
            'r_hip_pitch_joint',
            'r_shoulder_pitch_joint',
            'l_hip_roll_joint',
            'l_shoulder_roll_joint',
            'r_hip_roll_joint',
            'r_shoulder_roll_joint',
            'l_thigh_joint',
            'l_upper_arm_joint',
            'r_thigh_joint',
            'r_upper_arm_joint',
            'l_calf_joint',
            'l_elbow_joint',
            'r_calf_joint',
            'r_elbow_joint',
            'l_ankle_pitch_joint',
            'l_wrist_joint',
            'r_ankle_pitch_joint',
            'r_wrist_joint',
            'l_ankle_roll_joint',
            'r_ankle_roll_joint',
        ]

        self.joint_state = None
        self.control_uuid = None
        self.actual_joint_names = []
        self.current_positions = []
        self.test_joints = []  # 实际要测试的关节（交集）
        self.test_joint_indices = []  # 测试关节在实际系统中的索引

        # 订阅 joint_states
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # 创建控制命令发布器
        self.control_pub = self.create_publisher(MotorControlCommand, 'control_command', 10)

        # 创建服务客户端
        self.request_control_client = self.create_client(RequestControl, 'request_control')

        self.release_control_client = self.create_client(ReleaseControl, 'release_control')

        self.get_logger().info('Joint Tester initialized')

    def joint_state_callback(self, msg):
        """接收关节状态"""
        self.joint_state = msg
        self.actual_joint_names = list(msg.name)
        self.current_positions = list(msg.position)

    def wait_for_joint_state(self, timeout=5.0):
        """等待接收到关节状态"""
        self.get_logger().info('Waiting for joint_states...')
        start_time = time.time()

        while self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for joint_states')
                return False

        self.get_logger().info(f'Received joint_states with {len(self.actual_joint_names)} joints')
        return True

    def find_joint_intersection(self):
        """找出目标关节和实际关节的交集"""
        self.test_joints = []
        self.test_joint_indices = []

        for joint_name in self.target_joints:
            if joint_name in self.actual_joint_names:
                idx = self.actual_joint_names.index(joint_name)
                self.test_joints.append(joint_name)
                self.test_joint_indices.append(idx)

        self.get_logger().info(f'Found {len(self.test_joints)} joints to test:')
        for joint in self.test_joints:
            self.get_logger().info(f'  - {joint}')

        return len(self.test_joints) > 0

    def request_control(self):
        """申请电机控制权"""
        self.get_logger().info('Requesting motor control...')

        if not self.request_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Request control service not available')
            return False

        request = RequestControl.Request()
        request.node_name = self.get_name()
        request.motor_ids = self.test_joint_indices
        request.control_mode = RequestControl.Request.POSITION_MODE
        request.timeout_ms = 120000  # 120秒超时
        request.default_kp = [60.0] * len(self.test_joint_indices)
        request.default_kd = [1.1] * len(self.test_joint_indices)

        future = self.request_control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.control_uuid = response.uuid
                self.get_logger().info(f'Control granted, UUID: {self.control_uuid}')
                return True
            else:
                self.get_logger().error(f'Control request failed: {response.message}')
                return False
        else:
            self.get_logger().error('Control request timeout')
            return False

    def release_control(self):
        """释放电机控制权"""
        if self.control_uuid is None:
            return True

        self.get_logger().info('Releasing motor control...')

        if not self.release_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Release control service not available')
            return False

        request = ReleaseControl.Request()
        request.node_name = self.get_name()
        request.uuid = self.control_uuid
        request.release_mode = 0  # DAMPING_MODE

        future = self.release_control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Control released')
                self.control_uuid = None
                return True
            else:
                self.get_logger().error(f'Control release failed: {response.message}')
                return False
        else:
            self.get_logger().error('Control release timeout')
            return False

    def send_control_command(self, positions):
        """发送控制命令"""
        msg = MotorControlCommand()
        msg.uuid = self.control_uuid
        msg.motor_ids = self.test_joint_indices
        msg.positions = positions
        msg.velocities = [0.0] * len(positions)
        msg.kp = [60.0] * len(positions)
        msg.kd = [1.1] * len(positions)

        self.control_pub.publish(msg)

    def test_joint(self, joint_idx):
        """测试单个关节"""
        joint_name = self.test_joints[joint_idx]
        actual_idx = self.test_joint_indices[joint_idx]

        # 获取当前位置
        rclpy.spin_once(self, timeout_sec=0.1)
        current_pos = self.current_positions[actual_idx]

        self.get_logger().info(f'\n=== Testing joint {joint_idx + 1}/{len(self.test_joints)}: {joint_name} ===')
        self.get_logger().info(f'Current position: {current_pos:.3f}')

        # 准备控制命令（所有关节保持当前位置）
        # 修改为所需要的关节顺序
        target_positions = [self.current_positions[idx] for idx in self.test_joint_indices]

        # 第一阶段：移动到 +0.1
        target_positions[joint_idx] = current_pos + 0.2
        self.get_logger().info(f'Moving to {target_positions[joint_idx]:.3f} (+0.2)')

        # 发送命令并保持1秒
        start_time = time.time()
        while time.time() - start_time < 1.5:
            self.send_control_command(target_positions)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        # 第二阶段：回到原位置
        target_positions[joint_idx] = current_pos
        self.get_logger().info(f'Moving back to {current_pos:.3f}')

        # 发送命令并保持1秒
        start_time = time.time()
        while time.time() - start_time < 1.5:
            self.send_control_command(target_positions)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        self.get_logger().info(f'Joint {joint_name} test completed')

    def run(self):
        """运行测试"""
        try:
            # 1. 等待接收关节状态
            if not self.wait_for_joint_state():
                return False

            # 2. 找出要测试的关节
            if not self.find_joint_intersection():
                self.get_logger().error('No joints to test')
                return False

            # 3. 申请控制权
            if not self.request_control():
                return False

            # 4. 逐个测试关节
            self.get_logger().info(f'\nStarting to test {len(self.test_joints)} joints...\n')

            for i in range(len(self.test_joints)):
                self.test_joint(i)
                time.sleep(0.5)  # 关节之间间隔0.5秒

            self.get_logger().info('\n=== All joints tested successfully ===\n')

            # 5. 释放控制权
            self.release_control()

            return True

        except Exception as e:
            self.get_logger().error(f'Error during test: {str(e)}')
            # 确保释放控制权
            if self.control_uuid is not None:
                self.release_control()
            return False


def main(args=None):
    rclpy.init(args=args)

    tester = JointTester()

    try:
        success = tester.run()
        if success:
            tester.get_logger().info('Test completed successfully')
        else:
            tester.get_logger().error('Test failed')
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
        if tester.control_uuid is not None:
            tester.release_control()
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
