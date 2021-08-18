import os
import time
import argparse
import json
from collections import defaultdict, deque
from datetime import datetime
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
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


JOINT_VEL_MOVE_THRESHOLD = 1e-2
JOINT_POS_MOVE_THRESHOLD = 1e-3
INVALID_TIME = -1


class LoggingState(Enum):
    WAITING_FOR_NEW_TRAJECTORY = 0
    WAITING_TO_MOVE = 1
    LOGGING = 2
    LOGGING_NEXT_JOINT_STATES = 3
    VISUALIZE_LOGS = 4


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
        self._prev_joint_state = None
        self._prev_joint_states = deque(maxlen=10)
        self._next_joint_states = []
        self._trajectory = None
        self._logged_joint_states: List[JointState] = []
        self._logging_state = LoggingState.WAITING_FOR_NEW_TRAJECTORY
        self._logging_start_time = INVALID_TIME

    def _arm_joint_state_callback(self, joint_state: JointState):
        self.get_logger().debug(f'Received joint states. State: {self._logging_state}', throttle_duration_sec=1, throttle_time_source_type=self.get_clock())

        if self._logging_state == LoggingState.WAITING_TO_MOVE:
            if self._moving(joint_state, self._prev_joint_state) and self._logging_start_time == INVALID_TIME:
                self.get_logger().info('Started logging')
                self._logging_state = LoggingState.LOGGING
                self._logging_start_time = time.time()
                self._logged_joint_states.clear()
                self._logged_joint_states.append(joint_state)
            else:
                self._prev_joint_states.append(joint_state)
        elif self._logging_state == LoggingState.LOGGING:
            ros_duration = Duration.from_msg(self._trajectory.points[-1].time_from_start)
            duration_sec = ros_duration.nanoseconds / 1e9
            elapsed_time = time.time() - self._logging_start_time
            if elapsed_time <= duration_sec:
                self.get_logger().debug('Saving current joint state', throttle_duration_sec=0.5, throttle_time_source_type=self.get_clock())
                self._logged_joint_states.append(joint_state)
            else:
                self.get_logger().info('Logging a few next samples with joint states to synchronize with generated trajectory')
                self._logging_state = LoggingState.LOGGING_NEXT_JOINT_STATES
                self._logging_start_time = INVALID_TIME
                self._next_joint_states.clear()
        elif self._logging_state == LoggingState.LOGGING_NEXT_JOINT_STATES:
            if len(self._next_joint_states) < 10:
                self._next_joint_states.append(joint_state)
            else:
                self._logging_state = LoggingState.VISUALIZE_LOGS
        elif self._logging_state == LoggingState.VISUALIZE_LOGS:
            self.get_logger().info('Stopped logging. Visualizing data')
            self._logging_state = LoggingState.WAITING_FOR_NEW_TRAJECTORY
            # Synchronize logged joint states with trajectory
            trajectory_first_configuration = self._trajectory.points[0].positions
            all_logged_joint_states = list(self._prev_joint_states) + self._logged_joint_states + self._next_joint_states
            
            found_synchronized_log = False
            start_idx = -1
            for configuration_idx, joint_states in enumerate(all_logged_joint_states):
                for j_idx, joint_pos in enumerate(joint_states.position):
                    dist = trajectory_first_configuration[j_idx] - joint_pos
                    if abs(dist) > JOINT_POS_MOVE_THRESHOLD:
                        break
                else:
                    found_synchronized_log = True
                    start_idx = configuration_idx
                if found_synchronized_log:
                    break
            self.get_logger().debug(f'Starting index: {start_idx}')
            synchronized_joint_states = all_logged_joint_states[start_idx:len(self._trajectory.points)]
            self.get_logger().warn(f'{len(synchronized_joint_states)}, {len(all_logged_joint_states)}, {len(self._prev_joint_states)}, {len(self._logged_joint_states)}, {len(self._next_joint_states)}, {len(self._trajectory.points)}')
            serialized_logs = self._serialize_logs(self._trajectory, synchronized_joint_states)
            self._save_to_json(serialized_logs)
            self._visualize_trajectories(serialized_logs)
            self._prev_joint_states.clear()
        self._prev_joint_state = joint_state

    def _generated_trajectory_callback(self, trajectory: JointTrajectory):
        self.get_logger().info('Received trajectory')
        if self._logging_state == LoggingState.WAITING_FOR_NEW_TRAJECTORY:
            self._logging_state = LoggingState.WAITING_TO_MOVE
            self._trajectory = trajectory
        else:
            self.get_logger().debug('Invalid state to save new trajectory')

    def _moving(self, joint_state: JointState, prev_joint_state: JointState) -> bool:
        if joint_state is None or prev_joint_state is None:
            return False
        if len(joint_state.position) == 0 or len(prev_joint_state.position) == 0 or len(joint_state.position) != len(prev_joint_state.position):
            return False
        nr_joints = len(joint_state.position)
        for joint_idx in range(nr_joints):
            joint_diff = joint_state.position[joint_idx] - prev_joint_state.position[joint_idx]
            if abs(joint_diff) > JOINT_POS_MOVE_THRESHOLD:
                return True
        return False

    def _visualize_trajectories(self, serialized_logs: dict):
        nr_joints = serialized_logs['nr_joints']
        joint_states_times = serialized_logs['robot_trajectory']['time']
        trajectory_times = serialized_logs['generated_trajectory']['time']
        fig, ax = plt.subplots(nrows=nr_joints, ncols=2, sharex=True)
        for joint_idx in range(nr_joints):
            joint_states_pos = serialized_logs['robot_trajectory']['position'][joint_idx]
            joint_states_vel = serialized_logs['robot_trajectory']['velocity'][joint_idx]
            trajectory_pos = serialized_logs['generated_trajectory']['position'][joint_idx]
            trajectory_vel = serialized_logs['generated_trajectory']['velocity'][joint_idx]

            # print(f'{joint_idx + 1} -> {len(joint_states_pos)}')
            # print(f'{joint_idx + 1} -> {len(joint_states_vel)}')
            # print(f'{joint_idx + 1} -> {len(trajectory_pos)}')
            # print(f'{joint_idx + 1} -> {len(trajectory_vel)}')
            # print('---')

            ax[joint_idx, 0].plot(joint_states_times, joint_states_pos, label='robot_trajectory', linestyle='-')
            ax[joint_idx, 0].plot(trajectory_times, trajectory_pos, label='generated_trajectory', linestyle='-')
            ax[joint_idx, 0].set_title(f'Joint {joint_idx + 1} position [rad]')
            ax[joint_idx, 0].grid()

            ax[joint_idx, 1].plot(joint_states_times, joint_states_vel, label='robot_trajectory', linestyle='-')
            ax[joint_idx, 1].plot(trajectory_times, trajectory_vel, label='generated_trajectory', linestyle='-')
            ax[joint_idx, 1].set_title(f'Joint {joint_idx + 1} velocity [rad/s]')
            ax[joint_idx, 1].grid()

        handles, labels = ax.flatten()[-1].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right')
        fig.suptitle('Generated and robot trajectories')
        fig.supxlabel('Time [s]')
        plt.show()

    def _save_to_json(self, serialized_logs: dict):
        base_path = get_package_share_directory('visualization_tools_py')
        logs_path = os.path.join(base_path, 'trajectories_logs')
        os.makedirs(logs_path, exist_ok=True)
        current_time = datetime.now().isoformat()
        filepath = os.path.join(logs_path, f'{current_time}.json')
        with open(filepath, 'w') as f:
            json.dump(serialized_logs, f)

    def _serialize_logs(self, generated_trajectory: JointTrajectory, logged_joint_states: List[JointState]) -> dict:
        self.get_logger().debug(f'Generated trajectory length: {len(generated_trajectory.points)}. Logged joint states length: {len(logged_joint_states)}')
        out_serialized_logs = {}
        out_serialized_logs['robot_trajectory'] = {}
        out_serialized_logs['generated_trajectory'] = {}
        nr_joints = len(generated_trajectory.joint_names)
        out_serialized_logs['nr_joints'] = nr_joints

        # Prepare time axis for joint states
        start_time = Time.from_msg(logged_joint_states[0].header.stamp).nanoseconds / 1e9
        joint_states_times = []
        for joint_state in logged_joint_states:
            current_time = Time.from_msg(joint_state.header.stamp).nanoseconds / 1e9 
            joint_states_times.append(current_time - start_time) 
        out_serialized_logs['robot_trajectory']['time'] = joint_states_times
        # Prepare time axis for generated trajectory
        trajectory_times = []
        for trajectory_point in generated_trajectory.points:
            time_from_start = Duration.from_msg(trajectory_point.time_from_start).nanoseconds / 1e9
            trajectory_times.append(time_from_start) 
        out_serialized_logs['generated_trajectory']['time'] = trajectory_times

        # Save trajectories per joint
        out_serialized_logs['robot_trajectory']['position'] = []
        out_serialized_logs['robot_trajectory']['velocity'] = []
        out_serialized_logs['robot_trajectory']['effort'] = []
        out_serialized_logs['generated_trajectory']['position'] = []        
        out_serialized_logs['generated_trajectory']['velocity'] = []
        out_serialized_logs['generated_trajectory']['acceleration'] = []
        out_serialized_logs['generated_trajectory']['effort'] = []
        for joint_idx in range(nr_joints):
            joint_states_pos = []
            joint_states_vel = []
            joint_states_effort = []
            for joint_state in logged_joint_states:
                if len(joint_state.position) == nr_joints:
                    joint_states_pos.append(joint_state.position[joint_idx])
                if len(joint_state.velocity) == nr_joints:
                    joint_states_vel.append(joint_state.velocity[joint_idx])
                if len(joint_state.effort) == nr_joints:
                    joint_states_effort.append(joint_state.effort[joint_idx])
            out_serialized_logs['robot_trajectory']['position'].append(joint_states_pos)
            out_serialized_logs['robot_trajectory']['velocity'].append(joint_states_vel)
            out_serialized_logs['robot_trajectory']['effort'].append(joint_states_effort)
            
            trajectory_pos = []
            trajectory_vel = []
            trajectory_acc = []
            trajectory_eff = []
            for trajectory_point in generated_trajectory.points:
                if len(trajectory_point.positions) == nr_joints:
                    trajectory_pos.append(trajectory_point.positions[joint_idx])
                if len(trajectory_point.velocities) == nr_joints:
                    trajectory_vel.append(trajectory_point.velocities[joint_idx])
                if len(trajectory_point.accelerations) == nr_joints:
                    trajectory_acc.append(trajectory_point.accelerations[joint_idx])
                if len(trajectory_point.effort) == nr_joints:
                    trajectory_eff.append(trajectory_point.effort[joint_idx])
            out_serialized_logs['generated_trajectory']['position'].append(trajectory_pos)        
            out_serialized_logs['generated_trajectory']['velocity'].append(trajectory_vel)        
            out_serialized_logs['generated_trajectory']['acceleration'].append(trajectory_acc)        
            out_serialized_logs['generated_trajectory']['effort'].append(trajectory_eff)   
        return out_serialized_logs



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
