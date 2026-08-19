#!/usr/bin/env python3
"""
电机状态监控脚本
功能：实时监控所有电机的状态（位置、速度、力矩）
"""

import sys
import time

import numpy as np
import rclpy
from hightorque_msgs.msg import SessionStatus
from hightorque_msgs.srv import GetAvailableMotors
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MotorMonitor(Node):
    def __init__(self):
        super().__init__('motor_monitor')

        # 创建服务客户端
        self.get_motors_client = self.create_client(GetAvailableMotors, 'get_available_motors')

        # 创建订阅者
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.session_status_sub = self.create_subscription(
            SessionStatus, 'session_status', self.session_status_callback, 10
        )

        # 等待服务可用
        self.get_logger().info('等待电机管理器服务...')
        service_ready = self.get_motors_client.wait_for_service(timeout_sec=5.0)
        if not service_ready:
            self.get_logger().error('电机管理器服务未就绪: get_available_motors=False')
            self.get_logger().error(
                '请确认当前终端与 bringup 使用相同的 ROS2 环境 '
                '(ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY / RMW_IMPLEMENTATION)'
            )
            raise RuntimeError('Motor manager service not available')
        self.get_logger().info('服务已就绪')

        # 状态变量
        self.motor_ids = []
        self.motor_names = []
        self.motor_status = []
        self.current_state = None
        self.last_update_time = None
        self.session_status = None
        self.last_motor_query_time = 0

    def get_available_motors(self):
        """查询可用电机"""
        self.get_logger().info('查询可用电机...')

        req = GetAvailableMotors.Request()
        future = self.get_motors_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('查询电机超时')
            return False

        result = future.result()
        self.motor_ids = result.motor_ids
        self.motor_names = result.motor_names
        self.motor_status = result.motor_status

        self.get_logger().info(f'找到 {len(self.motor_ids)} 个电机')
        return True

    def joint_state_callback(self, msg):
        """接收电机状态反馈"""
        self.current_state = msg
        self.last_update_time = time.time()

    def session_status_callback(self, msg):
        """接收会话状态"""
        self.session_status = msg

    def print_motor_info(self):
        """打印电机信息"""
        print("\n" + "=" * 80)
        print("电机信息")
        print("=" * 80)
        print(f"{'ID':<5} {'名称':<30} {'状态':<10}")
        print("-" * 80)

        status_map = {0: 'IDLE', 1: 'OCCUPIED', 2: 'ERROR'}

        for i, motor_id in enumerate(self.motor_ids):
            name = self.motor_names[i] if i < len(self.motor_names) else f"motor_{motor_id}"
            status = status_map.get(self.motor_status[i], 'UNKNOWN') if i < len(self.motor_status) else 'UNKNOWN'
            print(f"{motor_id:<5} {name:<30} {status:<10}")

        print("=" * 80)

    def print_current_state(self, show_all=False):
        """打印当前状态"""
        if self.current_state is None:
            print("等待电机状态数据...")
            return

        # 每秒更新一次电机占用状态
        current_time = time.time()
        if current_time - self.last_motor_query_time >= 1.0:
            self.get_available_motors()
            self.last_motor_query_time = current_time

        # 清屏（可选）
        if not show_all:
            print("\033[2J\033[H", end='')  # ANSI清屏

        print("\n" + "=" * 100)
        print(f"电机状态监控 - {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 100)

        # 显示会话信息
        if self.session_status is not None:
            num_sessions = len(self.session_status.active_uuids)
            print(f"\n活跃会话数: {num_sessions}")
            if num_sessions > 0:
                print("-" * 100)
                for i in range(num_sessions):
                    uuid = (
                        self.session_status.active_uuids[i][:8] if i < len(self.session_status.active_uuids) else "N/A"
                    )
                    node = self.session_status.node_names[i] if i < len(self.session_status.node_names) else "N/A"
                    count = self.session_status.motor_counts[i] if i < len(self.session_status.motor_counts) else 0
                    print(f"  [{i+1}] UUID: {uuid}... | 节点: {node} | 电机数: {count}")
                print("-" * 100)
        else:
            print("\n活跃会话数: 等待数据...")
            print("-" * 100)

        print(f"\n{'ID':<5} {'名称':<25} {'位置(rad)':<12} {'速度(rad/s)':<15} {'力矩(N·m)':<18} {'状态':<20}")
        print("-" * 100)

        for i in range(len(self.current_state.name)):
            name = self.current_state.name[i] if i < len(self.current_state.name) else f"motor_{i}"
            pos = self.current_state.position[i] if i < len(self.current_state.position) else 0.0
            vel = self.current_state.velocity[i] if i < len(self.current_state.velocity) else 0.0
            eff = self.current_state.effort[i] if i < len(self.current_state.effort) else 0.0

            # 判断电机状态
            if abs(pos) > 10:
                status_str = "ERROR"
            elif i in self.motor_ids:
                status_str = "IDLE"
            else:
                status_str = "OCCUPIED"

            print(f"{i:<5} {name:<25} {pos:>11.4f} {vel:>14.4f} {eff:>17.4f} {status_str:>15}")

        print("=" * 100)

        # 统计信息
        if len(self.current_state.position) > 0:
            positions = np.array(self.current_state.position)
            velocities = (
                np.array(self.current_state.velocity)
                if len(self.current_state.velocity) > 0
                else np.zeros_like(positions)
            )
            efforts = (
                np.array(self.current_state.effort) if len(self.current_state.effort) > 0 else np.zeros_like(positions)
            )

            print(f"\n统计信息:")
            print(f"  位置范围: [{positions.min():.4f}, {positions.max():.4f}] rad")
            print(f"  速度范围: [{velocities.min():.4f}, {velocities.max():.4f}] rad/s")
            print(f"  力矩范围: [{efforts.min():.4f}, {efforts.max():.4f}] N·m")
            print(f"  位置均值: {positions.mean():.4f} rad")
            print(f"  位置标准差: {positions.std():.4f} rad")

        if self.last_update_time is not None:
            age = time.time() - self.last_update_time
            print(f"\n数据更新: {age:.3f} 秒前")

        print("\n按 Ctrl+C 退出")

    def monitor_continuous(self, update_rate=2.0):
        """持续监控模式"""
        self.get_logger().info(f'开始监控 (更新频率: {update_rate} Hz)...')

        dt = 1.0 / update_rate

        try:
            while rclpy.ok():
                self.print_current_state(show_all=False)
                time.sleep(dt)
                rclpy.spin_once(self, timeout_sec=0.01)

        except KeyboardInterrupt:
            self.get_logger().info('监控已停止')

    def monitor_once(self):
        """单次监控模式"""
        # 等待接收状态
        timeout = 5.0
        start_time = time.time()
        while self.current_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('超时：未接收到电机状态')
                return

        self.print_current_state(show_all=True)

    def run(self, mode='continuous'):
        """
        主运行函数

        Args:
            mode: 'continuous' 持续监控, 'once' 单次查询, 'info' 仅显示电机信息
        """
        try:
            # 查询可用电机
            if not self.get_available_motors():
                return

            if mode == 'info':
                # 仅显示电机信息
                self.print_motor_info()

            elif mode == 'once':
                # 单次查询
                self.print_motor_info()
                self.monitor_once()

            elif mode == 'continuous':
                # 持续监控
                self.print_motor_info()
                time.sleep(1.0)
                self.monitor_continuous(update_rate=2.0)

        except Exception as e:
            self.get_logger().error(f'发生错误: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    # 从命令行参数获取模式
    mode = 'continuous'
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode not in ['continuous', 'once', 'info']:
            print(f"无效的模式: {mode}")
            print("可用模式: continuous, once, info")
            print("  continuous - 持续监控（默认）")
            print("  once       - 单次查询")
            print("  info       - 仅显示电机信息")
            return

    monitor = MotorMonitor()

    try:
        monitor.run(mode=mode)
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
