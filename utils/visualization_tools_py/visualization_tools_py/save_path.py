import time
import argparse
from enum import Enum
from typing import List
import matplotlib.pyplot as plt
from numpy.core.shape_base import block
import rclpy 
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


JOINT_VEL_MOVE_THRESHOLD = 1e-2
INVALID_TIME = -1


class LoggingState(Enum):
    IDLE = 0
    LOGGING = 1


class TrajectoriesVisualizer(Node):
    def __init__(self):
        super().__init__("trajectories_visualizer")
        self._robot_joint_state_sub = self.create_subscription(
            msg_type=JointState,
            topic='arm_joint_states',
            callback=self._arm_joint_state_callback,
            qos_profile=QoSProfile(depth=10),
        )
        self._generated_trajectory_sub = self.create_subscription(
            msg_type=JointTrajectory,
            topic='trajectory',
            callback=self._generated_trajectory_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._trajectory = None
        self._valid_trajectory = False
        self._logged_joint_states: List[JointState] = []
        self._logging_state = LoggingState.IDLE
        self._logging_start_time = INVALID_TIME

    def _arm_joint_state_callback(self, joint_state: JointState):
        self.get_logger().debug('Received joint states', throttle_duration_sec=0.5,
                                throttle_time_source_type=self.get_clock())

        if self._logging_state == LoggingState.IDLE:
            if self._moving(joint_state) and self._logging_start_time == INVALID_TIME and self._valid_trajectory:
                self.get_logger().info('Started logging')
                self._valid_trajectory = False
                self._logging_state = LoggingState.LOGGING
                self._logging_start_time = time.time()
                self._logged_joint_states.clear()
                self._logged_joint_states.append(joint_state)
        elif self._logging_state == LoggingState.LOGGING:
            ros_duration = Duration.from_msg(
                self._trajectory.points[-1].time_from_start)
            duration_sec = ros_duration.nanoseconds / 1e9
            elapsed_time = time.time() - self._logging_start_time
            if elapsed_time <= duration_sec:
                self.get_logger().debug('Saving current joint state')
                self._logged_joint_states.append(joint_state)
            else:
                self.get_logger().info('Stopped logging')
                self._logging_state = LoggingState.IDLE
                self._logging_start_time = INVALID_TIME
                # Save to JSON and display TODO: Check wheth
                self._visualize_trajectories(
                    self._trajectory, self._logged_joint_states)
                # self._save_to_json(self._trajectory, self._logged_joint_states)

    def _generated_trajectory_callback(self, trajectory: JointTrajectory):
        self.get_logger().info('Received trajectory')
        self._valid_trajectory = True
        self._trajectory = trajectory

    def _moving(self, joint_state: JointState) -> bool:
        for _, joint_vel in enumerate(joint_state.velocity):
            if abs(joint_vel) > JOINT_VEL_MOVE_THRESHOLD:
                return True
        return False

    def _visualize_trajectories(self, generated_trajectory: JointTrajectory, logged_joint_states: List[JointState]):
        self.get_logger().info(
            f'{len(generated_trajectory.points)}, {len(logged_joint_states)}')
        nr_joints = len(generated_trajectory.joint_names)
        # Prepare time axis for joint states
        start_time = Time.from_msg(logged_joint_states[0].header.stamp).nanoseconds / 1e9
        joint_states_times = []
        for joint_state in logged_joint_states:
            current_time = Time.from_msg(joint_state.header.stamp).nanoseconds / 1e9 
            joint_states_times.append(current_time - start_time) 

        # Prepare time axis for generated trajectory
        trajectory_times = []
        for trajectory_point in generated_trajectory.points:
            time_from_start = Duration.from_msg(trajectory_point.time_from_start).nanoseconds / 1e9
            trajectory_times.append(time_from_start) 

        fig, ax = plt.subplots(nrows=nr_joints, ncols=3, sharex=True)
        for joint_idx in range(nr_joints):
            joint_states = [joint_state.position[joint_idx] for joint_state in logged_joint_states]
            trajectory = [trajectory_point.positions[joint_idx] for trajectory_point in generated_trajectory.points]
            ax[joint_idx, 0].plot(joint_states_times, joint_states, label='robot')
            ax[joint_idx, 0].plot(trajectory_times, trajectory, label='trajectory')
            ax[joint_idx, 0].grid()
        
        handles, labels = ax.get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper center')
        fig.legend()

        # plt.ion()
        # plt.draw()
        plt.show()
        # plt.pause(0.001)

    def _save_to_json(self, generated_trajectory: JointTrajectory, logged_joint_states: List[JointState]):
        raise NotImplementedError('Implement saving to JSON fool!')


def main():
    parser = argparse.ArgumentParser(description='Trajectories visualizer',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-l', '--log', type=str, default='info',
                        help='log level for node')
    args = parser.parse_args()
    log_level = args.log.lower()

    rclpy.init(args=None)
    trajectory_sub = TrajectoriesVisualizer()
    log_level_enum = LoggingSeverity.INFO
    if log_level == 'debug':
        log_level_enum = LoggingSeverity.DEBUG
    elif log_level == 'info':
        log_level_enum = LoggingSeverity.INFO
    elif log_level == 'warn':
        log_level_enum = LoggingSeverity.WARN
    elif log_level == 'error':
        log_level_enum = LoggingSeverity.ERROR
    elif log_level == 'fatal':
        log_level_enum = LoggingSeverity.FATAL
    print(trajectory_sub.get_name(), log_level_enum)
    set_logger_level(trajectory_sub.get_name(), log_level_enum)

    trajectory_sub.get_logger().info('Initializing trajectories visualizer')
    rclpy.spin(trajectory_sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
