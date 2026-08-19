#!/usr/bin/env python3
"""
电机归零脚本
功能：申请全身电机控制权，读取当前位置，平滑过渡到零位
"""

import time

import numpy as np
import rclpy
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ReleaseControl, RequestControl
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MoveToZeroController(Node):
    def __init__(self):
        super().__init__('move_to_zero_controller')

        # 创建服务客户端
        self.request_control_client = self.create_client(RequestControl, 'request_control')
        self.release_control_client = self.create_client(ReleaseControl, 'release_control')

        # 创建发布者和订阅者
        self.cmd_pub = self.create_publisher(MotorControlCommand, 'control_command', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # 等待服务可用
        self.get_logger().info('等待电机管理器服务...')
        request_ready = self.request_control_client.wait_for_service(timeout_sec=5.0)
        release_ready = self.release_control_client.wait_for_service(timeout_sec=5.0)
        if not request_ready or not release_ready:
            self.get_logger().error(
                '电机管理器服务未就绪: '
                f'request_control={request_ready}, release_control={release_ready}'
            )
            self.get_logger().error(
                '请确认当前终端与 bringup 使用相同的 ROS2 环境 '
                '(ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY / RMW_IMPLEMENTATION)'
            )
            raise RuntimeError('Motor manager services not available')
        self.get_logger().info('服务已就绪')

        # 状态变量
        self.uuid = None
        self.current_positions = None
        self.motor_ids = None
        self.joint_names = None

    def joint_state_callback(self, msg):
        """接收电机状态反馈"""
        if len(msg.position) > 0:
            self.current_positions = np.array(msg.position)
            self.joint_names = msg.name

    def request_all_motors(self):
        """申请全身电机控制权"""
        self.get_logger().info('申请全身电机控制权...')

        # 等待接收到第一次状态反馈，以确定电机数量
        timeout = 5.0
        start_time = time.time()
        while self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('超时：未接收到电机状态反馈')
                return False

        # 获取所有电机ID
        num_motors = len(self.current_positions)
        self.motor_ids = list(range(num_motors))

        self.get_logger().info(f'检测到 {num_motors} 个电机')

        # 申请控制权
        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = self.motor_ids
        req.control_mode = RequestControl.Request.POSITION_MODE
        req.default_behavior = 0  # HOLD_POSITION
        req.timeout_ms = 5000
        req.default_kp = [60.0] * num_motors
        req.default_kd = [1.0] * num_motors

        future = self.request_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('申请控制权超时')
            return False

        result = future.result()
        if result.success:
            self.uuid = result.uuid
            self.get_logger().info(f'控制权申请成功: UUID={self.uuid}')
            return True
        else:
            self.get_logger().error(f'控制权申请失败: {result.message}')
            return False

    def move_to_zero(self, duration=2.0, frequency=100.0):
        """
        平滑移动到零位

        Args:
            duration: 移动持续时间（秒）
            frequency: 控制频率（Hz）
        """
        if self.uuid is None:
            self.get_logger().error('未获得控制权')
            return False

        # 等待并获取最新的位置反馈
        self.get_logger().info('等待最新位置反馈...')
        old_positions = self.current_positions.copy() if self.current_positions is not None else None

        # 等待位置更新（最多等待1秒）
        timeout = 1.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.current_positions is not None:
                if old_positions is None or not np.array_equal(self.current_positions, old_positions):
                    break

        # 获取起始位置
        start_positions = self.current_positions.copy()
        target_positions = np.zeros(len(start_positions))
        target_by_joint = {
            'l_shoulder_roll_joint': 0.2,
            'r_shoulder_roll_joint': -0.2,
        }
        for joint_name, target in target_by_joint.items():
            if self.joint_names is not None and joint_name in self.joint_names:
                target_positions[list(self.joint_names).index(joint_name)] = target
            else:
                self.get_logger().warn(f'未找到关节 {joint_name}，保持目标为 0')

        self.get_logger().info(f'开始移动到零位，肩 roll 稍微外展...')
        self.get_logger().info(f'起始位置: {np.round(start_positions, 3)}')
        self.get_logger().info(f'目标位置: {target_positions}')
        self.get_logger().info(f'持续时间: {duration}秒, 频率: {frequency}Hz')

        # 计算步数
        num_steps = int(duration * frequency)
        dt = 1.0 / frequency

        # 执行平滑移动
        for step in range(num_steps + 1):
            # 计算插值系数（使用平滑的S曲线）
            t = step / num_steps
            # self.get_logger().info(f'计算插值系数: t={t:.3f}, step={step}/{num_steps}')
            # 使用平滑步函数：3t^2 - 2t^3
            smooth_t = 3 * t**2 - 2 * t**3

            # 计算当前目标位置
            current_target = start_positions + (target_positions - start_positions) * smooth_t
            # self.get_logger().info(
            # f'id={self.motor_ids}\nstart={start_positions}\n, target={target_positions}\n, current_target={current_target.tolist()}\n, current_target={current_target}'
            # )

            # 发送控制命令
            cmd = MotorControlCommand()
            cmd.uuid = self.uuid
            cmd.motor_ids = self.motor_ids
            cmd.positions = current_target.tolist()
            cmd.velocities = [0.0] * len(self.motor_ids)
            cmd.kp = [80.0] * len(self.motor_ids)  # 使用较高的kp确保跟踪
            cmd.kd = [1.0] * len(self.motor_ids)
            cmd.torques = [0.0] * len(self.motor_ids)

            self.cmd_pub.publish(cmd)

            # 打印进度
            if step % int(frequency) == 0:  # 每秒打印一次
                progress = (step / num_steps) * 100
                self.get_logger().info(f'进度: {progress:.1f}%')

            # 等待下一个控制周期
            time.sleep(dt)

        self.get_logger().info('移动完成！')

        # 最后再发送一次确保到达目标位置
        cmd = MotorControlCommand()
        cmd.uuid = self.uuid
        cmd.motor_ids = self.motor_ids
        cmd.positions = target_positions.tolist()
        cmd.velocities = [0.0] * len(self.motor_ids)
        cmd.kp = [80.0] * len(self.motor_ids)  # 使用较高的kp确保跟踪
        cmd.kd = [1.0] * len(self.motor_ids)
        cmd.torques = [0.0] * len(self.motor_ids)
        self.cmd_pub.publish(cmd)

        return True

    def release_control(self, release_mode=0):
        """
        释放控制权

        Args:
            release_mode: 0=保持原状, 1=阻尼模式, 2=不控制
        """
        if self.uuid is None:
            return

        self.get_logger().info(f'释放控制权 (mode={release_mode})...')

        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.uuid
        req.release_mode = release_mode

        future = self.release_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info('控制权已释放')
            self.uuid = None
        else:
            self.get_logger().error('释放控制权失败')

    def run(self):
        """主运行函数"""
        try:
            # 1. 申请全身电机控制权
            if not self.request_all_motors():
                return

            # 2. 移动到零位
            self.move_to_zero(duration=2.0, frequency=100.0)

        except KeyboardInterrupt:
            self.get_logger().info('收到中断信号')
        except Exception as e:
            self.get_logger().error(f'发生错误: {str(e)}')
        finally:
            # 4. 释放控制权（保持原状）
            self.release_control(release_mode=0)


def main(args=None):
    rclpy.init(args=args)

    controller = MoveToZeroController()

    try:
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
