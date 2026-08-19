#!/usr/bin/env python3
"""
全身电机测试脚本
自动检测所有电机并依次执行正弦波测试
"""

import sys
import time

import numpy as np
import rclpy
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import GetAvailableMotors, ReleaseControl, RequestControl
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AllMotorsTester(Node):
    def __init__(self):
        super().__init__('all_motors_tester')

        # 服务客户端
        self.get_motors_client = self.create_client(GetAvailableMotors, 'get_available_motors')
        self.request_control_client = self.create_client(RequestControl, 'request_control')
        self.release_control_client = self.create_client(ReleaseControl, 'release_control')

        # 话题
        self.cmd_pub = self.create_publisher(MotorControlCommand, 'control_command', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # 状态变量
        self.motor_ids = []
        self.motor_names = []
        self.num_motors = 0
        self.current_positions = None
        self.current_motor_name = None
        self.uuid = None
        self.joint_state_received = False

        self.get_logger().info('全身电机测试节点已启动')

    def joint_state_callback(self, msg):
        """接收电机状态"""
        if not self.joint_state_received:
            self.joint_state_received = True
            self.motor_names = list(msg.name)
            self.get_logger().info(f'首次接收到电机状态，共 {len(msg.position)} 个电机')

        self.current_positions = np.array(msg.position)

    def wait_for_joint_states(self, timeout=5.0):
        """等待接收电机状态"""
        self.get_logger().info('等待电机状态反馈...')
        start_time = time.time()

        while not self.joint_state_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('超时：未接收到电机状态反馈')
                return False

        return True

    def get_available_motors(self):
        """获取可用电机列表"""
        self.get_logger().info('查询可用电机...')

        if not self.get_motors_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务不可用: get_available_motors')
            return False

        req = GetAvailableMotors.Request()
        future = self.get_motors_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            self.motor_ids = result.motor_ids
            self.num_motors = len(self.motor_ids)
            self.get_logger().info(f'检测到 {self.num_motors} 个电机: {self.motor_ids}')
            return True
        else:
            self.get_logger().error('获取电机列表失败')
            return False

    def request_motor_control(self, motor_id):
        """申请单个电机控制权"""
        if not self.request_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务不可用: request_control')
            return False

        # 获取电机名称
        self.current_motor_name = (
            self.motor_names[motor_id] if motor_id < len(self.motor_names) else f"motor_{motor_id}"
        )

        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = [motor_id]
        req.control_mode = RequestControl.Request.POSITION_MODE
        req.default_behavior = RequestControl.Request.DAMPING_MODE
        req.default_kp = [80.0]
        req.default_kd = [1.1]
        req.timeout_ms = 5000

        future = self.request_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.uuid = future.result().uuid
            self.get_logger().info(f'[{self.current_motor_name}] 控制权申请成功')
            return True
        else:
            msg = future.result().message if future.result() else 'Unknown error'
            self.get_logger().error(f'[{self.current_motor_name}] 控制权申请失败: {msg}')
            return False

    def release_control(self, release_mode=2):
        """释放控制权"""
        if self.uuid is None:
            return

        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.uuid
        req.release_mode = release_mode

        future = self.release_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'[{self.current_motor_name}] 控制权已释放')
            self.uuid = None
        else:
            self.get_logger().error(f'[{self.current_motor_name}] 释放控制权失败')

    def test_motor_sine_wave(self, motor_id, duration=5.0, amplitude=0.2, frequency_hz=0.5):
        """
        对单个电机执行正弦波测试

        Args:
            motor_id: 电机ID
            duration: 测试持续时间（秒）
            amplitude: 正弦波幅度（弧度）
            frequency_hz: 正弦波频率（Hz）
        """
        if self.uuid is None or self.current_positions is None:
            return False

        # 获取中心位置
        center_position = self.current_positions[motor_id]
        self.get_logger().info(f'[{self.current_motor_name}] 中心位置: {center_position:.3f} rad')

        control_freq = 100.0  # Hz
        num_steps = int(duration * control_freq)
        dt = 1.0 / control_freq

        self.get_logger().info(
            f'[{self.current_motor_name}] 开始正弦波测试: 幅度={amplitude}rad, 频率={frequency_hz}Hz, 时长={duration}s'
        )

        for step in range(num_steps + 1):
            t = step * dt

            # 计算正弦波位置
            target_position = center_position + amplitude * np.sin(2 * np.pi * frequency_hz * t)

            # 发送控制命令
            cmd = MotorControlCommand()
            cmd.uuid = self.uuid
            cmd.motor_ids = [motor_id]
            cmd.positions = [target_position]
            cmd.velocities = []
            cmd.kp = [80.0]
            cmd.kd = [1.1]
            cmd.torques = [0.0]

            self.cmd_pub.publish(cmd)

            # 打印进度
            if step % int(control_freq) == 0:
                progress = (step / num_steps) * 100
                self.get_logger().info(
                    f'[{self.current_motor_name}]   进度: {progress:.0f}%, 位置: {target_position:.3f} rad'
                )

            time.sleep(dt)

        self.get_logger().info(f'[{self.current_motor_name}] 测试完成')
        return True

    def move_to_position(self, motor_id, target_position, duration=1.0):
        """平滑移动到指定位置"""
        if self.uuid is None or self.current_positions is None:
            return False

        start_position = self.current_positions[motor_id]
        control_freq = 100.0
        num_steps = int(duration * control_freq)
        dt = 1.0 / control_freq

        for step in range(num_steps + 1):
            t = step / num_steps
            smooth_t = 3 * t**2 - 2 * t**3
            current_target = start_position + (target_position - start_position) * smooth_t

            cmd = MotorControlCommand()
            cmd.uuid = self.uuid
            cmd.motor_ids = [motor_id]
            cmd.positions = [current_target]
            cmd.velocities = []
            cmd.kp = [80.0]
            cmd.kd = [1.1]
            cmd.torques = [0.0]

            self.cmd_pub.publish(cmd)
            time.sleep(dt)

        return True

    def run(self):
        """主运行函数"""
        try:
            # 1. 等待电机状态
            if not self.wait_for_joint_states():
                return

            # 2. 获取可用电机列表
            if not self.get_available_motors():
                return

            if self.num_motors == 0:
                self.get_logger().error('没有可用电机')
                return

            # 3. 依次测试每个电机
            for i, motor_id in enumerate(self.motor_ids):
                motor_name = self.motor_names[motor_id] if motor_id < len(self.motor_names) else f"motor_{motor_id}"
                self.get_logger().info(
                    f'\n========== 测试电机 {motor_id} - {motor_name} ({i+1}/{self.num_motors}) =========='
                )

                # 申请控制权
                if not self.request_motor_control(motor_id):
                    self.get_logger().warn(f'跳过电机 {motor_id} - {motor_name}')
                    continue

                # 记录初始位置
                initial_position = self.current_positions[motor_id]

                # 执行正弦波测试
                self.test_motor_sine_wave(motor_id, duration=2.0, amplitude=0.2, frequency_hz=0.5)

                # 返回初始位置
                self.get_logger().info(f'[{self.current_motor_name}] 返回初始位置...')
                self.move_to_position(motor_id, initial_position, duration=0.2)

                # 释放控制权（零力矩模式）
                self.release_control(release_mode=ReleaseControl.Request.KEEP_MODE)

                # 短暂等待
                time.sleep(0.1)

            self.get_logger().info('\n========== 全部电机测试完成 ==========')

        except KeyboardInterrupt:
            self.get_logger().info('收到中断信号')
        except Exception as e:
            self.get_logger().error(f'发生错误: {str(e)}')
        finally:
            if self.uuid is not None:
                self.release_control(release_mode=ReleaseControl.Request.KEEP_MODE)


def main(args=None):
    rclpy.init(args=args)

    tester = AllMotorsTester()

    try:
        tester.run()
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
