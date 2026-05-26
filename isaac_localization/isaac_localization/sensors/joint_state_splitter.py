#!/usr/bin/env python3

from typing import Iterable, List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


DEFAULT_WHEEL_JOINTS = [
    "FL_Drive_Continuous",
    "FR_Drive_Continuous",
    "CL_Drive_Continuous",
    "CR_Drive_Continuous",
    "RL_Drive_Continuous",
    "RR_Drive_Continuous",
    "FL_Steer_Revolute",
    "FR_Steer_Revolute",
    "RL_Steer_Revolute",
    "RR_Steer_Revolute",
]

DEFAULT_ARM_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "finger_joint",
    "left_inner_knuckle_joint",
    "left_outer_knuckle_joint",
    "right_inner_knuckle_joint",
    "right_inner_finger_joint",
    "left_inner_finger_joint",
]


class JointStateSplitter(Node):
    """Split Isaac's full articulation JointState into rover wheel and arm topics."""

    def __init__(self) -> None:
        super().__init__("joint_state_splitter")

        self.declare_parameter("input_topic", "/joint_states_raw")
        self.declare_parameter("wheel_topic", "/rover/wheel_states")
        self.declare_parameter("arm_topic", "/joint_states")
        self.declare_parameter("wheel_joint_names", DEFAULT_WHEEL_JOINTS)
        self.declare_parameter("arm_joint_names", DEFAULT_ARM_JOINTS)

        self.input_topic = self.get_parameter("input_topic").value
        self.wheel_topic = self.get_parameter("wheel_topic").value
        self.arm_topic = self.get_parameter("arm_topic").value
        self.wheel_joint_names = list(self.get_parameter("wheel_joint_names").value)
        self.arm_joint_names = list(self.get_parameter("arm_joint_names").value)

        self._warned_missing = set()

        self.create_subscription(
            JointState,
            self.input_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.wheel_pub = self.create_publisher(JointState, self.wheel_topic, 10)
        self.arm_pub = self.create_publisher(JointState, self.arm_topic, 10)

        self.get_logger().info(f"Subscribing: {self.input_topic}")
        self.get_logger().info(f"Publishing wheel states: {self.wheel_topic}")
        self.get_logger().info(f"Publishing arm states  : {self.arm_topic}")

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_index = {name: index for index, name in enumerate(msg.name)}

        self.wheel_pub.publish(
            self._filter_joint_state(msg, self.wheel_joint_names, name_to_index)
        )
        self.arm_pub.publish(
            self._filter_joint_state(msg, self.arm_joint_names, name_to_index)
        )

    def _filter_joint_state(
        self,
        msg: JointState,
        target_names: Iterable[str],
        name_to_index: dict,
    ) -> JointState:
        out = JointState()
        out.header = msg.header

        for name in target_names:
            index = name_to_index.get(name)
            if index is None:
                self._warn_missing_once(name)
                continue

            out.name.append(name)
            self._append_if_available(out.position, msg.position, index)
            self._append_if_available(out.velocity, msg.velocity, index)
            self._append_if_available(out.effort, msg.effort, index)

        return out

    def _warn_missing_once(self, joint_name: str) -> None:
        if joint_name in self._warned_missing:
            return
        self._warned_missing.add(joint_name)
        self.get_logger().warn(
            f"Joint '{joint_name}' was not found in {self.input_topic}."
        )

    @staticmethod
    def _append_if_available(output: List[float], values, index: int) -> None:
        if values and index < len(values):
            output.append(values[index])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateSplitter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
