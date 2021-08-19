import matplotlib.pyplot as plt


def visualize_trajectories(serialized_logs: dict):
    nr_joints = serialized_logs['nr_joints']
    joint_states_times = serialized_logs['robot_trajectory']['time']
    trajectory_times = serialized_logs['generated_trajectory']['time']
    fig, ax = plt.subplots(nrows=nr_joints, ncols=2, sharex=True)
    for joint_idx in range(nr_joints):
        joint_states_pos = serialized_logs['robot_trajectory']['position'][joint_idx]
        joint_states_vel = serialized_logs['robot_trajectory']['velocity'][joint_idx]
        trajectory_pos = serialized_logs['generated_trajectory']['position'][joint_idx]
        trajectory_vel = serialized_logs['generated_trajectory']['velocity'][joint_idx]

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