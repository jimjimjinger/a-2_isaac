from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="isaac_ai", executable="vision_ai_node", output="screen"),
        Node(package="isaac_ai", executable="object_pose_node", output="screen"),
        Node(package="isaac_ai", executable="rl_policy_node", output="screen"),
        Node(package="isaac_nodes", executable="state_collector_node", output="screen"),
        Node(package="isaac_nodes", executable="task_manager_node", output="screen"),
        Node(package="isaac_nodes", executable="robot_executor_node", output="screen"),
    ])

