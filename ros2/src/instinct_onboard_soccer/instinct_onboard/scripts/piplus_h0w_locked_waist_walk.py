import queue
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import RequestControl
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.agents.walk_agent import WalkAgent
from instinct_onboard.ros_nodes.hightorque import HightorqueNode

MAIN_LOOP_DT = 0.02
MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500


class H0WLockedWaistHightorqueNode(HightorqueNode):
    """Run a 23-DOF H1W policy on H0W hardware by omitting the waist motor.

    The policy still sees a 23-D joint/action vector. The waist entry is kept at
    zero for observations and actions, but no command is sent to motor id 22.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.waist_sim_idx = self.sim_joint_names.index("waist_yaw_joint")
        self.waist_real_idx = self.joint_map[self.waist_sim_idx]
        self.active_motor_ids = sorted(
            {
                int(self.joint_map[sim_idx])
                for sim_idx in range(self.NUM_JOINTS)
                if sim_idx != self.waist_sim_idx
            }
        )

    def _request_motor_control(self):
        self.get_logger().info(
            "Requesting motor control without waist: "
            f"skip sim[{self.waist_sim_idx}] -> real[{self.waist_real_idx}], "
            f"active motors={self.active_motor_ids}"
        )

        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = self.active_motor_ids
        req.control_mode = 0
        req.default_behavior = 1
        req.timeout_ms = self.control_timeout_ms
        req.default_kp = [0.0] * len(req.motor_ids)
        req.default_kd = [0.0] * len(req.motor_ids)

        future = self.request_control_client.call_async(req)
        start_time = time.time()
        timeout = 10.0
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            raise RuntimeError("Failed to request motor control: timeout")

        result = future.result()
        if not result.success:
            raise RuntimeError(f"Failed to request motor control: {result.message}")

        self.control_uuid = result.uuid
        self.get_logger().info(f"Motor control granted: UUID={self.control_uuid}")

    def send_action(self, action, action_offset=0.0, action_scale=1.0, p_gains=0.0, d_gains=0.0):
        action = np.asarray(action, dtype=np.float32).copy()
        action[self.waist_sim_idx] = 0.0
        super().send_action(
            action,
            action_offset=action_offset,
            action_scale=action_scale,
            p_gains=p_gains,
            d_gains=d_gains,
        )

    def _publish_motor_cmd(self, target_joint_pos, p_gains, d_gains):
        target_joint_pos = np.asarray(target_joint_pos, dtype=np.float32).copy()
        if hasattr(self, "apply_external_head_targets"):
            target_joint_pos = self.apply_external_head_targets(target_joint_pos)
        if np.isnan(target_joint_pos).any():
            self.get_logger().error("Target joint positions contain NaN, skip sending.")
            return
        if self.control_uuid is None and not self.dryrun:
            self.get_logger().error("control_uuid is None, skip sending.")
            return

        cmd = MotorControlCommand()
        cmd.uuid = self.control_uuid or "dryrun"
        cmd.motor_ids = []
        cmd.positions = []
        cmd.velocities = []
        cmd.kp = []
        cmd.kd = []
        cmd.torques = []

        for sim_idx in range(self.NUM_JOINTS):
            if sim_idx == self.waist_sim_idx:
                continue
            real_idx = int(self.joint_map[sim_idx])
            cmd.motor_ids.append(real_idx)
            cmd.positions.append(float(target_joint_pos[sim_idx] * self.joint_signs[sim_idx]))
            cmd.velocities.append(0.0)
            cmd.kp.append(float(p_gains[sim_idx]))
            cmd.kd.append(float(d_gains[sim_idx]))
            cmd.torques.append(0.0)

        self.low_cmd_publisher.publish(cmd)

    def _low_state_callback(self, msg):
        super()._low_state_callback(msg)
        # H0W hardware has no waist motor. Keep the 23-D H1W policy's waist
        # observation fixed at zero instead of reading a nonexistent joint.
        self.joint_pos_[self.waist_sim_idx] = 0.0
        self.joint_vel_[self.waist_sim_idx] = 0.0


class PiPlusH0WLockedWaistWalkNode(H0WLockedWaistHightorqueNode):
    def __init__(
        self,
        *args,
        cmd_vel_topic=None,
        cmd_vel_timeout=0.5,
        head_joint_topic=None,
        head_joint_timeout=0.5,
        head_yaw_limit=1.5,
        head_pitch_limit=1.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.available_agents = {}
        self.current_agent_name = None
        self.cmd_vel_topic = cmd_vel_topic
        self.cmd_vel_timeout = float(cmd_vel_timeout)
        self.cmd_vel_command = np.zeros(3, dtype=np.float32)
        self.cmd_vel_time = None
        self.head_joint_topic = head_joint_topic
        self.head_joint_timeout = float(head_joint_timeout)
        self.head_joint_time = None
        self.head_joint_targets = {}
        self.head_sim_indices = {
            "head_yaw_joint": self.sim_joint_names.index("head_yaw_joint"),
            "head_pitch_joint": self.sim_joint_names.index("head_pitch_joint"),
        }
        self.head_joint_limits = {
            "head_yaw_joint": abs(float(head_yaw_limit)),
            "head_pitch_joint": abs(float(head_pitch_limit)),
        }
        self.prev_r1_pressed = False

    def _get_imu_ang_vel_obs(self):
        return self._get_base_ang_vel_obs()

    def _get_imu_orientation_obs(self):
        return self._get_quat_w_obs()

    def _get_imu_projected_gravity_obs(self):
        return self._get_projected_gravity_obs()

    def register_agent(self, name, agent):
        self.available_agents[name] = agent

    def start_ros_handlers(self):
        super().start_ros_handlers()
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.cmd_vel_topic:
            self.cmd_vel_subscriber = self.create_subscription(
                Twist,
                self.cmd_vel_topic,
                self._cmd_vel_callback,
                10,
            )
            self.get_logger().info(
                f"Using external velocity command topic '{self.cmd_vel_topic}' "
                f"with timeout {self.cmd_vel_timeout:.2f}s."
            )
        if self.head_joint_topic:
            self.head_joint_subscriber = self.create_subscription(
                JointState,
                self.head_joint_topic,
                self._head_joint_callback,
                10,
            )
            self.get_logger().info(
                f"Using external head joint topic '{self.head_joint_topic}' "
                f"with timeout {self.head_joint_timeout:.2f}s."
            )
        self.main_loop_timer = self.create_timer(MAIN_LOOP_DT, self.main_loop_callback)
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_timer_counter = 0
            self.main_loop_timer_counter_time = time.time()
            self.main_loop_callback_time_consumptions = queue.Queue(maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)
        self.get_logger().info("PiPlus H0W locked-waist walk node is ready.")

    def _cmd_vel_callback(self, msg):
        self.cmd_vel_command[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.cmd_vel_time = time.time()

    def _head_joint_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.head_sim_indices:
                limit = self.head_joint_limits[name]
                self.head_joint_targets[name] = float(np.clip(position, -limit, limit))
        if self.head_joint_targets:
            self.head_joint_time = time.time()

    def get_external_cmd_vel(self):
        if not self.cmd_vel_topic or self.cmd_vel_time is None:
            return None
        if time.time() - self.cmd_vel_time > self.cmd_vel_timeout:
            return np.zeros(3, dtype=np.float32)
        return self.cmd_vel_command.copy()

    def apply_external_head_targets(self, target_joint_pos):
        if not self.head_joint_topic or self.head_joint_time is None:
            return target_joint_pos
        if time.time() - self.head_joint_time > self.head_joint_timeout:
            return target_joint_pos
        for name, target in self.head_joint_targets.items():
            target_joint_pos[self.head_sim_indices[name]] = target
        return target_joint_pos

    def main_loop_callback(self):
        t0 = time.time()

        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        agent = self.available_agents[self.current_agent_name]
        action, done = agent.step()
        self.send_action(action, agent.action_offset, agent.action_scale, agent.p_gains, agent.d_gains)

        r1_pressed = bool(self.joy_stick_data.R1)
        r1_pressed_edge = r1_pressed and not self.prev_r1_pressed

        if self.current_agent_name == "cold_start":
            if done:
                self.get_logger().info(
                    "ColdStartAgent done, press R1 to switch to walk agent.",
                    throttle_duration_sec=10.0,
                )
            if done and r1_pressed_edge:
                self.get_logger().info("R1 pressed, switching to walk agent.")
                self.current_agent_name = "walk"
                self.available_agents[self.current_agent_name].reset()
        elif self.current_agent_name == "walk" and r1_pressed_edge:
            self.get_logger().info("R1 pressed, resetting walk agent.")
            agent.reset()

        self.prev_r1_pressed = r1_pressed

        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_callback_time_consumptions.put(time.time() - t0)
            self.main_loop_timer_counter += 1
            if self.main_loop_timer_counter % MAIN_LOOP_FREQUENCY_CHECK_INTERVAL == 0:
                times = [self.main_loop_callback_time_consumptions.get() for _ in range(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)]
                self.get_logger().info(
                    f"Actual main loop frequency: "
                    f"{MAIN_LOOP_FREQUENCY_CHECK_INTERVAL / (time.time() - self.main_loop_timer_counter_time):.2f} Hz. "
                    f"Mean time consumption: {np.mean(times):.4f} s."
                )
                self.main_loop_timer_counter = 0
                self.main_loop_timer_counter_time = time.time()


class CmdVelWalkAgent(WalkAgent):
    """Walk agent that can take ROS2 geometry_msgs/Twist commands.

    When cmd_vel is enabled and fresh, linear.x/y and angular.z are used as the
    policy velocity command. Otherwise the original joystick mapping is used.
    """

    def __init__(self, *args, max_cmd_vel=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_cmd_vel = np.asarray(max_cmd_vel or [0.5, 0.5, 1.0], dtype=np.float32)

    def _get_base_velocity_command_cmd_obs(self):
        cmd_vel = self.ros_node.get_external_cmd_vel()
        if cmd_vel is not None:
            return np.clip(cmd_vel, -self.max_cmd_vel, self.max_cmd_vel)
        return super()._get_base_velocity_command_cmd_obs()


def create_walk_agent(logdir, **kwargs):
    from instinct_onboard.agents import base as _base

    agent_cls = kwargs.pop("agent_cls", WalkAgent)
    max_cmd_vel = kwargs.pop("max_cmd_vel", None)
    if agent_cls is CmdVelWalkAgent:
        kwargs["max_cmd_vel"] = max_cmd_vel
    original_parse = _base.OnboardAgent._parse_obs_config

    def patched_parse(self):
        self.cfg["observations"]["policy"].pop("concatenate_dim", None)
        return original_parse(self)

    _base.OnboardAgent._parse_obs_config = patched_parse
    try:
        return agent_cls(logdir=logdir, **kwargs)
    finally:
        _base.OnboardAgent._parse_obs_config = original_parse


def main(args):
    rclpy.init()

    node = PiPlusH0WLockedWaistWalkNode(
        robot_class_name="PiPlus_S_12L8A0G2H1W",
        joint_pos_protect_ratio=2.0,
        dryrun=not args.nodryrun,
        computer_clip_torque=False,
        cmd_vel_topic=args.cmd_vel_topic,
        cmd_vel_timeout=args.cmd_vel_timeout,
        head_joint_topic=args.head_joint_topic,
        head_joint_timeout=args.head_joint_timeout,
        head_yaw_limit=args.head_yaw_limit,
        head_pitch_limit=args.head_pitch_limit,
    )

    walk_agent = create_walk_agent(
        logdir=args.logdir,
        ros_node=node,
        x_vel_scale=args.x_vel_scale,
        y_vel_scale=args.y_vel_scale,
        yaw_vel_scale=args.yaw_vel_scale,
        agent_cls=CmdVelWalkAgent if args.cmd_vel_topic else WalkAgent,
        max_cmd_vel=args.max_cmd_vel,
    )
    node.register_agent("walk", walk_agent)

    cold_start_agent = ColdStartAgent(
        startup_step_size=args.startup_step_size,
        ros_node=node,
        joint_target_pos=walk_agent.default_joint_pos,
        action_scale=walk_agent.action_scale,
        action_offset=walk_agent.action_offset,
        p_gains=walk_agent.p_gains * args.kpkd_factor,
        d_gains=walk_agent.d_gains * args.kpkd_factor,
    )
    node.register_agent("cold_start", cold_start_agent)

    print(f"logdir (walk): {args.logdir}")
    print(f"waist lock   : policy sim joint '{node.sim_joint_names[node.waist_sim_idx]}' is held at action=0 and not sent")
    print(f"cmd_vel      : {args.cmd_vel_topic or 'disabled, joystick controls velocity'}")
    print(f"head topic   : {args.head_joint_topic or 'disabled, policy controls head'}")
    print(f"dryrun       : {not args.nodryrun}")

    node.start_ros_handlers()
    if args.skip_cold_start:
        node.current_agent_name = "walk"
        node.available_agents["walk"].reset()
        node.get_logger().info("Skipping cold start, starting with walk agent directly.")

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
        description="Run a 23-DOF PiPlus H1W walk policy on H0W hardware with waist locked/omitted."
    )
    parser.add_argument("--logdir", type=str, required=True, help="Walk policy directory containing exported/actor.onnx")
    parser.add_argument("--startup_step_size", type=float, default=0.2)
    parser.add_argument("--kpkd_factor", type=float, default=1.0)
    parser.add_argument("--x_vel_scale", type=float, default=0.5)
    parser.add_argument("--y_vel_scale", type=float, default=-0.5)
    parser.add_argument("--yaw_vel_scale", type=float, default=-1.0)
    parser.add_argument(
        "--cmd_vel_topic",
        type=str,
        default=None,
        help="Optional ROS2 geometry_msgs/Twist topic. If set, fresh cmd_vel overrides joystick velocity.",
    )
    parser.add_argument(
        "--cmd_vel_timeout",
        type=float,
        default=0.5,
        help="Seconds before stale cmd_vel is treated as zero velocity.",
    )
    parser.add_argument(
        "--max_cmd_vel",
        type=float,
        nargs=3,
        default=[0.5, 0.5, 1.0],
        metavar=("VX", "VY", "WZ"),
        help="Safety clip for external cmd_vel [linear.x linear.y angular.z].",
    )
    parser.add_argument(
        "--head_joint_topic",
        type=str,
        default=None,
        help="Optional ROS2 sensor_msgs/JointState topic for head_yaw_joint/head_pitch_joint targets.",
    )
    parser.add_argument(
        "--head_joint_timeout",
        type=float,
        default=0.5,
        help="Seconds before stale external head joint targets are ignored.",
    )
    parser.add_argument("--head_yaw_limit", type=float, default=1.5, help="Safety clip for external head yaw target.")
    parser.add_argument("--head_pitch_limit", type=float, default=1.0, help="Safety clip for external head pitch target.")
    parser.add_argument("--nodryrun", action="store_true", default=False)
    parser.add_argument("--skip_cold_start", action="store_true", default=False)
    args = parser.parse_args()
    main(args)
