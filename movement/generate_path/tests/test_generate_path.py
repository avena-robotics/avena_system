import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from custom_interfaces.action import GeneratePathPose, SimpleAction


def load_camera_poses(path: str):
    try:
        with open(path, 'r') as f:
            camera_poses = json.load(f)
    except FileNotFoundError as e:
        print(f'Cannot load file from "{path}". File does not exist. Exiting...')
        exit(-1)
    return camera_poses


def run_simple_action(node: Node, action_client: ActionClient):
    goal = SimpleAction.Goal()
    send_goal_future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        return False
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    result = get_result_future.result()
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        return True
    else:
        return False


def send_single_camera_pose(node: Node, generate_path_action_client: ActionClient, camera_pose: dict) -> bool:
    goal_msg = GeneratePathPose.Goal()
    goal_msg.end_effector_pose.position.x = camera_pose['position']['x']
    goal_msg.end_effector_pose.position.y = camera_pose['position']['y']
    goal_msg.end_effector_pose.position.z = camera_pose['position']['z']
    goal_msg.end_effector_pose.orientation.x = camera_pose['orientation']['x']
    goal_msg.end_effector_pose.orientation.y = camera_pose['orientation']['y']
    goal_msg.end_effector_pose.orientation.z = camera_pose['orientation']['z']
    goal_msg.end_effector_pose.orientation.w = camera_pose['orientation']['w']

    send_goal_future = generate_path_action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    
    if not goal_handle.accepted:
        return False

    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    result = get_result_future.result()
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        return True
    else:
        return False


def execute_single_camera_move(node: Node, action_clients: dict, camera_pose: dict) -> bool:
    if not send_single_camera_pose(node, action_clients['generate_path_pose'], camera_pose):
        return False
    if not run_simple_action(node, action_clients['generate_trajectory']):
        return False
    if not run_simple_action(node, action_clients['execute_move']):
        return False
    return True


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('testing_move')

    camera_poses = load_camera_poses('camera_poses.json')
    
    action_clients = {}
    action_clients['generate_path_pose'] = ActionClient(node, GeneratePathPose, 'generate_path_pose')
    action_clients['generate_trajectory'] = ActionClient(node, SimpleAction, 'generate_trajectory')
    action_clients['execute_move'] = ActionClient(node, SimpleAction, 'execute_move')

    for action_client_name, action_client in action_clients.items():
        node.get_logger().info(f'Waiting for action server for {action_client_name}')    
        if not action_client.wait_for_server(5.0):
            node.get_logger().error(f'Failed to connect to action server')
            exit(1)
        
    success_cnt = 0
    all_poses_cnt = 0
    for camera_name, camera_poses in camera_poses.items():
        node.get_logger().info(f'Processing camera poses for "{camera_name}"')
        for i, camera_pose in enumerate(camera_poses):
            all_poses_cnt += 1
            node.get_logger().info(f'Pose {i + 1}/{len(camera_poses)}')

            start_time = time.perf_counter()
            if execute_single_camera_move(node, action_clients, camera_pose):
            # if send_single_camera_pose(node, action_client, camera_pose):
                node.get_logger().info('Goal succeeded')
                success_cnt += 1
            else:
                node.get_logger().error('Goal failed')
            elapsed_time_ms = (time.perf_counter() - start_time) * 1000
            node.get_logger().info(f'Elapsed time: {elapsed_time_ms:.2f} [ms]')
            input('Press any key to continue...')
            time.sleep(0.01)

    if success_cnt != all_poses_cnt:
        node.get_logger().error(f'Test for IK failed: {success_cnt}/{all_poses_cnt} succeeded')
    else:
        node.get_logger().info(f'Test for IK succeeded: {success_cnt}/{all_poses_cnt} succeeded')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
