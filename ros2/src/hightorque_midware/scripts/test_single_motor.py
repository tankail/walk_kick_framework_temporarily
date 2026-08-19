#!/usr/bin/env python3
"""
单电机测试脚本
功能：申请单个电机控制权，进行简单的正弦波运动测试
"""

import sys
import time

import numpy as np
import rclpy
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ReleaseControl, RequestControl
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SingleMotorTester(Node):
    def __init__(self, motor_id=0):
        super().__init__('single_motor_tester')

        self.motor_id = motor_id

        # 创建服务客户端
        self.request_control_client = self.create_client(RequestControl, 'request_control')
        self.release_control_client = self.create_client(ReleaseControl, 'release_control')

        # 创建发布者和订阅者
        self.cmd_pub = self.create_publisher(MotorControlCommand, 'control_command', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # 等待服务可用
        self.get_logger().info('等待电机管理器服务...')
        self.request_control_client.wait_for_service(timeout_sec=5.0)
        self.release_control_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('服务已就绪')

        # 状态变量
        self.uuid = None
        self.current_position = None
        self.joint_name = None

    def joint_state_callback(self, msg):
        """接收电机状态反馈"""
        if len(msg.position) > self.motor_id:
            self.current_position = msg.position[self.motor_id]
            self.joint_name = msg.name[self.motor_id] if len(msg.name) > self.motor_id else f"motor_{self.motor_id}"

    def request_motor_control(self):
        """申请单个电机控制权"""
        self.get_logger().info(f'申请电机 {self.motor_id} 控制权...')

        # 等待接收到状态反馈
        timeout = 5.0
        start_time = time.time()
        while self.current_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('超时：未接收到电机状态反馈')
                return False

        self.get_logger().info(f'电机ID: {self.motor_id}, 名称: {self.joint_name}')
        self.get_logger().info(f'当前位置: {self.current_position:.3f} rad')

        # 申请控制权
        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = [self.motor_id]
        req.control_mode = RequestControl.Request.POSITION_MODE
        req.default_behavior = RequestControl.Request.DAMPING_MODE
        req.timeout_ms = 5000
        req.default_kp = [80.0]
        req.default_kd = [1.0]

        future = self.request_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('申请控制权超时')
            return False

        result = future.result()
        if result.success:
            self.uuid = result.uuid
            self.get_logger().info(f'[{self.joint_name}] 控制权申请成功: UUID={self.uuid}')
            return True
        else:
            self.get_logger().error(f'[{self.joint_name}] 控制权申请失败: {result.message}')
            return False

    def sine_wave_motion(self, amplitude=0.2, frequency_hz=0.5, duration=5.0, control_freq=100.0):
        """
        正弦波运动

        Args:
            amplitude: 振幅（rad）
            frequency_hz: 正弦波频率（Hz）
            duration: 持续时间（秒）
            control_freq: 控制频率（Hz）
        """
        if self.uuid is None:
            self.get_logger().error('未获得控制权')
            return False

        center_position = self.current_position
        self.get_logger().info(f'[{self.joint_name}] 开始正弦波运动...')
        self.get_logger().info(f'[{self.joint_name}] 中心位置: {center_position:.3f} rad')
        self.get_logger().info(f'[{self.joint_name}] 振幅: {amplitude:.3f} rad')
        self.get_logger().info(f'[{self.joint_name}] 频率: {frequency_hz:.2f} Hz')
        self.get_logger().info(f'[{self.joint_name}] 持续时间: {duration:.1f} 秒')

        num_steps = int(duration * control_freq)
        dt = 1.0 / control_freq

        for step in range(num_steps):
            t = step * dt

            # 计算正弦波位置
            target_position = center_position + amplitude * np.sin(2 * np.pi * frequency_hz * t)

            # 发送控制命令
            cmd = MotorControlCommand()
            cmd.uuid = self.uuid
            cmd.motor_ids = [self.motor_id]
            cmd.positions = [target_position]
            cmd.velocities = []
            cmd.torques = []
            cmd.kp = [80.0]
            cmd.kd = [1.1]
            cmd.torques = [0.0]

            self.cmd_pub.publish(cmd)

            # 打印进度
            if step % int(control_freq) == 0:
                progress = (step / num_steps) * 100
                self.get_logger().info(f'[{self.joint_name}] 进度: {progress:.1f}%, 位置: {target_position:.3f} rad')

            time.sleep(dt)

        self.get_logger().info(f'[{self.joint_name}] 正弦波运动完成！')
        return True

    def move_to_position(self, target_position, duration=1.0, frequency=100.0):
        """
        移动到指定位置

        Args:
            target_position: 目标位置（rad）
            duration: 移动持续时间（秒）
            frequency: 控制频率（Hz）
        """
        if self.uuid is None:
            self.get_logger().error('未获得控制权')
            return False

        start_position = self.current_position
        self.get_logger().info(f'[{self.joint_name}] 移动: {start_position:.3f} → {target_position:.3f} rad')

        num_steps = int(duration * frequency)
        dt = 1.0 / frequency

        for step in range(num_steps + 1):
            t = step / num_steps
            # 平滑插值
            smooth_t = 3 * t**2 - 2 * t**3
            current_target = start_position + (target_position - start_position) * smooth_t

            cmd = MotorControlCommand()
            cmd.uuid = self.uuid
            cmd.motor_ids = [self.motor_id]
            cmd.positions = [current_target]
            cmd.velocities = []
            cmd.kp = [80.0]
            cmd.kd = [1.1]
            cmd.torques = [0.0]

            self.cmd_pub.publish(cmd)
            time.sleep(dt)

        self.get_logger().info(f'[{self.joint_name}] 移动完成！')
        return True

    def release_control(self, release_mode=0):
        """释放控制权"""
        if self.uuid is None:
            return

        self.get_logger().info(f'[{self.joint_name}] 释放控制权 (mode={release_mode})...')

        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.uuid
        req.release_mode = release_mode

        future = self.release_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'[{self.joint_name}] 控制权已释放')
            self.uuid = None
        else:
            self.get_logger().error(f'[{self.joint_name}] 释放控制权失败')

    def run(self):
        """主运行函数"""
        try:
            # 1. 申请电机控制权
            if not self.request_motor_control():
                return

            # 2. 执行正弦波运动
            self.sine_wave_motion(amplitude=0.2, frequency_hz=0.5, duration=2.0)

            # 3. 回到中心位置
            self.get_logger().info(f'[{self.joint_name}] 返回初始位置...')
            initial_position = self.current_position
            self.move_to_position(initial_position, duration=0.5)

        except KeyboardInterrupt:
            self.get_logger().info('收到中断信号')
        except Exception as e:
            self.get_logger().error(f'发生错误: {str(e)}')
        finally:
            # 4. 释放控制权
            self.release_control(release_mode=ReleaseControl.Request.KEEP_MODE)


def main(args=None):
    rclpy.init(args=args)

    # 从命令行参数获取电机ID
    motor_id = 0
    if len(sys.argv) > 1:
        try:
            motor_id = int(sys.argv[1])
        except ValueError:
            print(f"无效的电机ID: {sys.argv[1]}, 使用默认值 0")

    print(f"测试电机 ID: {motor_id}")

    controller = SingleMotorTester(motor_id=motor_id)

    try:
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
