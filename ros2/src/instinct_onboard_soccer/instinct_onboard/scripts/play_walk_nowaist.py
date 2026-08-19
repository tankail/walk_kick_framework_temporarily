import queue
import time

import numpy as np
import rclpy

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.agents.walk_agent import WalkAgent
from instinct_onboard.ros_nodes.hightorque import HightorqueNode

MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500

"""
PiPlus Walk Node

A ROS2 node for controlling PiPlus_S_12L8A0G2H0W using only a walking policy.

Command-Line Arguments:
    Required:
        --walk_logdir PATH     Directory containing the trained walk agent model
                              (must contain exported/actor.onnx)

    Optional:
        --startup_step_size FLOAT
                              Startup step size for cold start agent (default: 0.2)
        --nodryrun            Disable dry run mode (default: False, runs in dry run mode)
        --kpkd_factor FLOAT   KP/KD gain multiplier for cold start agent (default: 1.0)
        --debug               Enable debug mode with debugpy (listens on 0.0.0.0:6789)

Agent Workflow:
    1. Cold Start Agent
       - Automatically starts when node launches
       - Transitions robot to the walk policy default pose
       - Automatically switches to walk when finished

    2. Walk Agent
       - Executes continuous walking policy
       - Velocity command comes from joystick input

Example Usage:
    python piplus_walk.py --walk_logdir /path/to/walk/model

    Real robot control (disable dry run):
        python piplus_walk.py --walk_logdir /path/to/walk/model --nodryrun
"""


class PiPlusWalkNode(HightorqueNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.available_agents = dict()
        self.current_agent_name: str | None = None

    def register_agent(self, name: str, agent):
        self.available_agents[name] = agent

    def start_ros_handlers(self):
        super().start_ros_handlers()
        main_loop_duration = 0.02
        self.get_logger().info(f"Starting main loop with duration: {main_loop_duration} seconds.")
        self.main_loop_timer = self.create_timer(main_loop_duration, self.main_loop_callback)
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_timer_counter: int = 0
            self.main_loop_timer_counter_time = time.time()
            self.main_loop_callback_time_consumptions = queue.Queue(maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)

    def main_loop_callback(self):
        main_loop_callback_start_time = time.time()

        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        if self.current_agent_name == "cold_start":
            action, done = self.available_agents[self.current_agent_name].step()
            if done:
                self.get_logger().info(
                    "ColdStartAgent done, switching to walk agent.",
                    throttle_duration_sec=10.0,
                )
            self.send_action(
                action,
                self.available_agents[self.current_agent_name].action_offset,
                self.available_agents[self.current_agent_name].action_scale,
                self.available_agents[self.current_agent_name].p_gains,
                self.available_agents[self.current_agent_name].d_gains,
            )
            if done:
                self.current_agent_name = "walk"
                self.available_agents[self.current_agent_name].reset()

        elif self.current_agent_name == "walk":
            action, done = self.available_agents[self.current_agent_name].step()
            self.send_action(
                action,
                self.available_agents[self.current_agent_name].action_offset,
                self.available_agents[self.current_agent_name].action_scale,
                self.available_agents[self.current_agent_name].p_gains,
                self.available_agents[self.current_agent_name].d_gains,
            )

        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_callback_time_consumptions.put(time.time() - main_loop_callback_start_time)
            self.main_loop_timer_counter += 1
            if self.main_loop_timer_counter % MAIN_LOOP_FREQUENCY_CHECK_INTERVAL == 0:
                time_consumptions = [
                    self.main_loop_callback_time_consumptions.get() for _ in range(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)
                ]
                self.get_logger().info(
                    f"Actual main loop frequency: {(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL / (time.time() - self.main_loop_timer_counter_time)):.2f} Hz. Mean time consumption: {np.mean(time_consumptions):.4f} s."
                )
                self.main_loop_timer_counter = 0
                self.main_loop_timer_counter_time = time.time()


def main(args):
    rclpy.init()

    node = PiPlusWalkNode(
        joint_pos_protect_ratio=2.0,
        node_name="piplus_node",
        robot_class_name="PiPlus_S_12L8A0G2H0W",
        dryrun=not args.nodryrun,
    )

    walk_agent = WalkAgent(
        logdir=args.walk_logdir,
        ros_node=node,
        x_vel_pos_scale=0.5,
        x_vel_neg_scale=1.5,
        y_vel_scale=0.5,
        yaw_vel_scale=1.0,
    )
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
    node.register_agent("walk", walk_agent)

    node.start_ros_handlers()
    node.get_logger().info("PiPlusWalkNode is ready to run.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="PiPlus Walk Node")
    parser.add_argument(
        "--walk_logdir",
        type=str,
        required=True,
        help="Directory to load the walk agent from",
    )
    parser.add_argument(
        "--startup_step_size",
        type=float,
        default=0.2,
        help="Startup step size for the cold start agent (default: 0.2)",
    )
    parser.add_argument(
        "--nodryrun",
        action="store_true",
        default=False,
        help="Run the node without dry run mode (default: False)",
    )
    parser.add_argument(
        "--kpkd_factor",
        type=float,
        default=1.0,
        help="KPKD factor for the cold start agent (default: 1.0)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Enable debug mode (default: False)",
    )

    args = parser.parse_args()

    if args.debug:
        import debugpy

        ip_address = ("0.0.0.0", 6789)
        print("Process: " + " ".join(sys.argv[:]))
        print("Is waiting for attach at address: %s:%d" % ip_address, flush=True)
        debugpy.listen(ip_address)
        debugpy.wait_for_client()
        debugpy.breakpoint()

    main(args)
