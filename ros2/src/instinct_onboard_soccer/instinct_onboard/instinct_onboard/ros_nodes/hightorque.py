import numpy as np
import rclpy
from hightorque_msgs.msg import MotorControlCommand
from hightorque_msgs.srv import ReleaseControl, RequestControl
from sensor_msgs.msg import Imu, JointState, Joy

import instinct_onboard.robot_cfgs as robot_cfgs
from instinct_onboard import utils
from instinct_onboard.ros_nodes.base import RealNode


class HightorqueNode(RealNode):
    """ROS interface implementation for hightorque humanoid robot.

    This node handles communication with the PiPlus robot hardware, including:
    - Low-level state feedback (joint positions, velocities, IMU data)
    - Motor command publishing (position, velocity, torque control)
    - Wireless controller input handling
    - Safety monitoring and emergency shutdown

    """

    def __init__(
        self,
        computer_clip_torque: bool = False,
        low_state_topic: str = "joint_states",
        low_cmd_topic: str = "control_command",
        imu_state_topic: str = "/imu/data",
        joy_stick_topic: str = "/joy",
        node_name: str = "hightorque_node",
        control_timeout_ms: int = 5000,
        kp_clip: float = 500,
        kd_clip: float = 50,
        **kwargs,
    ):
        """Initialize Hightorque ROS node.

        Args:
            low_state_topic: Topic for receiving robot state feedback (JointState)
            low_cmd_topic: Topic for publishing motor commands (MotorControlCommand)
            imu_state_topic: Topic for receiving IMU data
            joy_stick_topic: Topic for receiving wireless controller input
            motor_manager_namespace: Namespace for motor manager services
            control_timeout_ms: Timeout for motor control session (milliseconds)
            **kwargs: Additional arguments passed to RealNode
        """
        super().__init__(node_name=node_name, **kwargs)
        self.low_state_topic = low_state_topic
        self.imu_state_topic = imu_state_topic
        # Generate a unique cmd topic so that the low_cmd will not send to the robot's motor in dryrun mode
        self.low_cmd_topic = (
            low_cmd_topic if not self.dryrun else low_cmd_topic + "_dryrun_" + str(np.random.randint(0, 65535))
        )
        self.joy_stick_topic = joy_stick_topic
        self.control_timeout_ms = control_timeout_ms

        # UUID for motor control session
        self.control_uuid = None
        self.computer_clip_torque = computer_clip_torque
        self.kp_clip = kp_clip
        self.kd_clip = kd_clip
        self.max_abs_valid_joint_state = 20.0
        self.max_invalid_joint_state_frames = 25
        self.invalid_joint_state_counts = np.zeros(self.NUM_JOINTS, dtype=np.int32)

    def parse_config(self):
        """Parse robot-specific configurations from robot_cfgs."""
        super().parse_config()

        # Load PiPlus-specific configurations
        self.joint_map = getattr(robot_cfgs, self.robot_class_name).joint_map
        self.real_joint_names = getattr(robot_cfgs, self.robot_class_name).real_joint_names
        self.joint_signs = getattr(robot_cfgs, self.robot_class_name).joint_signs
        self.turn_on_motor_mode = getattr(robot_cfgs, self.robot_class_name).turn_on_motor_mode
        self.mode_pr = getattr(robot_cfgs, self.robot_class_name).mode_pr

    def start_ros_handlers(self):
        """After initializing the env and policy, register ros related callbacks and topics"""
        super().start_ros_handlers()

        # Service clients for motor control
        self.request_control_client = self.create_client(RequestControl, "request_control")
        self.release_control_client = self.create_client(ReleaseControl, "release_control")

        # Publisher for motor commands
        self.low_cmd_publisher = self.create_publisher(MotorControlCommand, self.low_cmd_topic, 10)
        # ROS subscribers
        self.low_state_subscriber = self.create_subscription(
            JointState, self.low_state_topic, self._low_state_callback, 10
        )
        self.torso_imu_subscriber = self.create_subscription(
            Imu, self.imu_state_topic, self._torso_imu_state_callback, 10
        )
        self.joy_stick_subscriber = self.create_subscription(Joy, self.joy_stick_topic, self._joy_stick_callback, 10)

        self.get_logger().info(
            "ROS handlers started, waiting to receive critical low state and wireless controller messages."
        )

        # Wait for motor manager services
        if not self.dryrun:
            self.get_logger().info("Waiting for hightorque_midware services...")
            if not self.request_control_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service {self.request_control_client.srv_name} not available")
                raise RuntimeError("Motor manager services not available")
            if not self.release_control_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service {self.release_control_client.srv_name} not available")
                raise RuntimeError("Motor manager services not available")
            self.get_logger().info("Hightorque midware services available")

        if not self.dryrun:
            self.get_logger().warn(
                f"You are running the code in no-dryrun mode and publishing to '{self.low_cmd_topic}', Please keep safe."
            )
        else:
            self.get_logger().warn(
                f"You are publishing low cmd to '{self.low_cmd_topic}' because of dryrun mode, Please check and be safe."
            )
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.check_buffers_ready():
                break
        self.get_logger().info("All necessary buffers received, the robot is ready to go.")

        # Request motor control after buffers are ready. Dryrun nodes must not
        # acquire a real control UUID; they publish to a random dryrun topic.
        if not self.dryrun:
            self._request_motor_control()

    def check_buffers_ready(self):
        """Check if all the necessary buffers are ready to use. Only used at the the end of the start_ros_handlers."""
        buffer_ready = hasattr(self, "low_state_buffer") and self.joy_stick_data.lx is not None
        if self.imu_state_topic is not None:
            buffer_ready = buffer_ready and hasattr(self, "torso_imu_buffer")
        return buffer_ready

    def _request_motor_control(self):
        """Request control of all motors from motor manager."""
        self.get_logger().info("Requesting motor control...")

        # Request control of all motors
        motor_ids = list(range(self.NUM_JOINTS))

        req = RequestControl.Request()
        req.node_name = self.get_name()
        req.motor_ids = motor_ids
        req.control_mode = 0  # POSITION_MODE (matching hightorque_controller)
        req.default_behavior = 1  # DAMPING_MODE
        req.timeout_ms = self.control_timeout_ms

        # Convert default gains to float list (populate with default values for each motor)
        req.default_kp = [0.0] * len(motor_ids)
        req.default_kd = [0.0] * len(motor_ids)

        self.get_logger().info(f"Requesting control for {len(motor_ids)} motors: {motor_ids}")
        self.get_logger().info(f"Control mode: {req.control_mode}, timeout: {req.timeout_ms}ms")

        future = self.request_control_client.call_async(req)

        # Poll for result instead of using spin_until_future_complete to avoid deadlock
        import time

        start_time = time.time()
        timeout = 10.0
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            self.get_logger().error("Motor control request timed out after 10 seconds")
            self.get_logger().error("Check if hightorque_midware_node is responding properly")
            raise RuntimeError("Failed to request motor control")

        result = future.result()
        if result.success:
            self.control_uuid = result.uuid
            self.get_logger().info(f"Motor control granted: UUID={self.control_uuid}")
        else:
            self.get_logger().error(f"Motor control request failed: {result.message}")
            raise RuntimeError(f"Failed to request motor control: {result.message}")

    """
    ROS callbacks and handlers that update the buffer
    """

    def _low_state_callback(self, msg):
        """store and handle proprioception data"""
        self.get_logger().info("Low state data received.", once=True)
        self.low_state_buffer = msg  # keep the latest low state

        # refresh joint_pos and joint_vel
        valid_position = np.zeros(self.NUM_JOINTS, dtype=bool)
        for sim_idx in range(self.NUM_JOINTS):
            real_idx = self.joint_map[sim_idx]
            if real_idx < len(msg.position):
                raw_position = float(msg.position[real_idx])
                if not np.isfinite(raw_position) or abs(raw_position) > self.max_abs_valid_joint_state:
                    self.invalid_joint_state_counts[sim_idx] += 1
                    self.get_logger().warn(
                        f"Ignoring invalid joint state position real[{real_idx}]={raw_position:.3f} "
                        f"for sim[{sim_idx}] {self.sim_joint_names[sim_idx]} "
                        f"({self.invalid_joint_state_counts[sim_idx]}/{self.max_invalid_joint_state_frames})",
                        throttle_duration_sec=2.0,
                    )
                    if self.invalid_joint_state_counts[sim_idx] >= self.max_invalid_joint_state_frames:
                        self.get_logger().error(
                            f"Joint {sim_idx}(sim), {real_idx}(real) has invalid feedback for "
                            f"{self.invalid_joint_state_counts[sim_idx]} consecutive frames."
                        )
                        self.get_logger().error("The motors and this process shuts down.")
                        self._turn_off_motors()
                        raise SystemExit()
                    continue
                self.invalid_joint_state_counts[sim_idx] = 0
                self.joint_pos_[sim_idx] = raw_position * self.joint_signs[sim_idx]
                valid_position[sim_idx] = True

        for sim_idx in range(self.NUM_JOINTS):
            real_idx = self.joint_map[sim_idx]
            if real_idx < len(msg.velocity):
                raw_velocity = float(msg.velocity[real_idx])
                if np.isfinite(raw_velocity):
                    self.joint_vel_[sim_idx] = raw_velocity * self.joint_signs[sim_idx]

        # automatic safety check
        for sim_idx in range(self.NUM_JOINTS):
            if not valid_position[sim_idx]:
                continue
            real_idx = self.joint_map[sim_idx]
            if (
                self.joint_pos_[sim_idx] > self.joint_pos_protect_high[sim_idx]
                or self.joint_pos_[sim_idx] < self.joint_pos_protect_low[sim_idx]
            ):
                self.get_logger().error(
                    f"Joint {sim_idx}(sim), {real_idx}(real) position out of range at"
                    f" {msg.position[real_idx] if real_idx < len(msg.position) else 'N/A'}"
                )
                self.get_logger().error("The motors and this process shuts down.")
                self._turn_off_motors()
                raise SystemExit()

    def _torso_imu_state_callback(self, msg):
        """store and handle torso imu data"""
        self.get_logger().info("Torso IMU data received.", once=True)
        self.torso_imu_buffer = msg

    def _joy_stick_callback(self, msg):
        self.get_logger().info(
            f"[joydbg_cb] axes={len(msg.axes)} buttons={len(msg.buttons)} lx={msg.axes[0] if msg.axes else 'none'}",
            throttle_duration_sec=1.0,
        )
        self.get_logger().info("Wireless controller data received.", once=True)

        # Ensure we have enough axes and buttons
        if len(msg.axes) < 8 or len(msg.buttons) < 8:
            self.get_logger().warn(
                f"Unexpected Joy message format: {len(msg.axes)} axes, {len(msg.buttons)} buttons", once=True
            )
            return

        beitong_layout = len(msg.buttons) >= 16
        l1_index = 6 if beitong_layout else 4
        r1_index = 7 if beitong_layout else 5
        select_index = 10 if beitong_layout else 6
        start_index = 11 if beitong_layout else 7

        # Map buttons. 16-button BEITONG controllers expose L1/R1 as BtnTL/BtnTR at 6/7.
        self.joy_stick_data.A = bool(msg.buttons[0])
        self.joy_stick_data.B = bool(msg.buttons[1])
        self.joy_stick_data.X = bool(msg.buttons[2])
        self.joy_stick_data.Y = bool(msg.buttons[3])
        self.joy_stick_data.L1 = bool(msg.buttons[l1_index])
        self.joy_stick_data.R1 = bool(msg.buttons[r1_index])
        self.joy_stick_data.select = bool(msg.buttons[select_index])
        self.joy_stick_data.start = bool(msg.buttons[start_index])

        # Map analog sticks (0:lx, 1:ly, 2:rz/ry, 3:rx)
        self.joy_stick_data.lx = msg.axes[0]
        self.joy_stick_data.ly = msg.axes[1]
        self.joy_stick_data.ry = msg.axes[2]
        self.joy_stick_data.rx = msg.axes[3]

        # 扳机映射: 北通 A2P3A DInput 模式 (16 键) 下, jstest 实测:
        #   物理 LT -> axis5(Brake), 物理 RT -> axis4(Gas); 静止位=+1.0, 按下=-1.0。
        # XBox 模式 (11 键) 下 LT=axis2, RT=axis5, 静止=0.0, 按下=+1.0。
        # 用按钮数判断模式, 并用"符号"区分静止与按下(不能再 abs(), 否则静止恒真)。
        if len(msg.buttons) >= 16:
            # 北通 DInput: 静止 +1.0, 按下 -1.0
            self.joy_stick_data.L2 = bool(msg.axes[5] < -0.5)
            self.joy_stick_data.R2 = bool(msg.axes[4] < -0.5)
            lt_axis, rt_axis, lt_dir, rt_dir = 5, 4, -1.0, -1.0
        else:
            # XBox/XInput: 静止 0.0, 按下 +1.0
            self.joy_stick_data.L2 = bool(msg.axes[2] > 0.5)
            self.joy_stick_data.R2 = bool(msg.axes[5] > 0.5)
            lt_axis, rt_axis, lt_dir, rt_dir = 2, 5, 1.0, 1.0

        # Map D-pad (6:horizontal, 7:vertical)
        self.joy_stick_data.left = bool(msg.axes[6] > 0.5)
        self.joy_stick_data.right = bool(msg.axes[6] < -0.5)
        self.joy_stick_data.up = bool(msg.axes[7] > 0.5)
        self.joy_stick_data.down = bool(msg.axes[7] < -0.5)

        # Emergency shutdown: LT + RT + START (triple combo).
        # Single LT/RT no longer kills motors so LT/RT can be used in the
        # enter/exit combos (LT+RT+A / LT+RT+B) handled by soccer_control_node.
        # lt_dir/rt_dir 归一化两种模式的按下方向, 静止位(+1.0/0.0)不会触发。
        if (
            (msg.axes[lt_axis] * lt_dir > 0.5)
            and (msg.axes[rt_axis] * rt_dir > 0.5)
            and self.joy_stick_data.start
        ):
            self.get_logger().error("LT+RT+START pressed: emergency motor shutdown.")
            self._turn_off_motors()
            raise SystemExit()

    """
    Refresh observation buffer and corresponding sub-functions
    NOTE: everything will be NON-batchwise. There is NO batch dimension in the observation.
    """

    def _get_quat_w_obs(self):
        """Get the quaternion in wxyz format from the torso IMU or low state buffer."""
        if hasattr(self, "torso_imu_buffer"):
            # sensor_msgs/Imu uses xyzw format, convert to wxyz
            return np.array(
                [
                    self.torso_imu_buffer.orientation.w,
                    self.torso_imu_buffer.orientation.x,
                    self.torso_imu_buffer.orientation.y,
                    self.torso_imu_buffer.orientation.z,
                ],
                dtype=np.float32,
            )
        else:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def _get_base_ang_vel_obs(self):
        if hasattr(self, "torso_imu_buffer"):
            return np.array(
                [
                    self.torso_imu_buffer.angular_velocity.x,
                    self.torso_imu_buffer.angular_velocity.y,
                    self.torso_imu_buffer.angular_velocity.z,
                ],
                dtype=np.float32,
            )
        else:
            return np.zeros(3, dtype=np.float32)

    def _get_imu_ang_vel_obs(self):
        if hasattr(self, "torso_imu_buffer"):
            return np.array(
                [
                    self.torso_imu_buffer.angular_velocity.x,
                    self.torso_imu_buffer.angular_velocity.y,
                    self.torso_imu_buffer.angular_velocity.z,
                ],
                dtype=np.float32,
            )
        else:
            return np.zeros(3, dtype=np.float32)

    def _get_projected_gravity_obs(self):
        if hasattr(self, "torso_imu_buffer"):
            quat_wxyz = np.quaternion(
                self.torso_imu_buffer.orientation.w,
                self.torso_imu_buffer.orientation.x,
                self.torso_imu_buffer.orientation.y,
                self.torso_imu_buffer.orientation.z,
            )
        else:
            quat_wxyz = np.quaternion(1.0, 0.0, 0.0, 0.0)

        return utils.quat_rotate_inverse(
            quat_wxyz,
            self.gravity_vec,
        ).astype(np.float32)

    """
    Control related functions
    """

    """
    Functions that actually publish the commands and take effect
    """

    def _publish_motor_cmd(
        self,
        target_joint_pos: np.array,  # shape (NUM_JOINTS,), in simulation order
        p_gains: np.ndarray,  # In the order of simulation joints, not real joints
        d_gains: np.ndarray,  # In the order of simulation joints, not real joints
    ):
        """Publish the joint commands to the robot motors in robot coordinates system.
        robot_coordinates_action: shape (NUM_JOINTS,), in simulation order.
        """
        if np.isnan(target_joint_pos).any():
            self.get_logger().error("Robot coordinates action contain NaN, Skip sending the action to the robot.")
            return

        if self.control_uuid is None and not self.dryrun:
            self.get_logger().error("control_uuid is None:.")
            # without control UUID, skip publishing
            return

        # Prepare MotorControlCommand message
        cmd = MotorControlCommand()
        cmd.uuid = self.control_uuid or "dryrun"
        cmd.motor_ids = []
        cmd.positions = []
        cmd.velocities = []
        cmd.kp = []
        cmd.kd = []
        cmd.torques = []

        for sim_idx in range(self.NUM_JOINTS):
            real_idx = self.joint_map[sim_idx]
            cmd.motor_ids.append(real_idx)
            # Convert to float for message compatibility
            cmd.positions.append(float(target_joint_pos[sim_idx] * self.joint_signs[sim_idx]))
            cmd.velocities.append(0.0)
            cmd.kp.append(float(p_gains[sim_idx]))
            cmd.kd.append(float(d_gains[sim_idx]))
            cmd.torques.append(0.0)

        self.low_cmd_publisher.publish(cmd)

    def _turn_off_motors(self):
        """Turn off the motors"""
        if self.control_uuid is None:
            return

        self.get_logger().warn(f"[{self.get_name()}] Releasing motor control, UUID: {self.control_uuid}")

        req = ReleaseControl.Request()
        req.node_name = self.get_name()
        req.uuid = self.control_uuid
        req.release_mode = 1  # DAMPING_MODE (matching hightorque_controller) 0: keep, 1: damping, 2: free

        future = self.release_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Motor control released successfully")
        else:
            self.get_logger().error("Failed to release motor control")

        self.control_uuid = None

    def __del__(self):
        """Cleanup: release motor control on node destruction."""
        if hasattr(self, "control_uuid") and self.control_uuid is not None:
            self._turn_off_motors()
