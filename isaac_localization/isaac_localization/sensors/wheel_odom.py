#!/usr/bin/env python3

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class WheelOdometryNode(Node):
    """
    Wheel Odometry Node

    역할:
    - /joint_states에서 wheel joint angular velocity를 읽는다.
    - AAU rover의 6개 drive wheel 속도와 front steer angle을 읽는다.
    - Ackermann bicycle model로 x, y, yaw를 적분한다.
    - /rover/wheel_odom으로 nav_msgs/Odometry를 publish한다.

    주의:
    - wheel joint 이름, wheel_radius는 실제 USD/로버 모델에 맞게 수정해야 한다.
    """

    def __init__(self):
        super().__init__("wheel_odom_node")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/rover/wheel_odom")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self.declare_parameter("wheel_radius", 0.10)
        self.declare_parameter("wheelbase_length", 0.849)
        self.declare_parameter("track_width", 0.894)

        # AAU rover: 6 drive joints + 4 steer joints.
        # Names are based on /joint_states published from Isaac Sim.
        self.declare_parameter(
            "left_wheel_joints",
            [
                "FL_Drive_Continuous",
                "CL_Drive_Continuous",
                "RL_Drive_Continuous",
            ],
        )
        self.declare_parameter(
            "right_wheel_joints",
            [
                "FR_Drive_Continuous",
                "CR_Drive_Continuous",
                "RR_Drive_Continuous",
            ],
        )
        self.declare_parameter(
            "front_steer_joints",
            [
                "FL_Steer_Revolute",
                "FR_Steer_Revolute",
            ],
        )

        # 조향각이 작으면 좌우 wheel 속도 차이 기반 yaw rate를 사용한다.
        self.declare_parameter("steer_deadband", 1.0e-3)

        # 바퀴 회전 방향 보정용
        self.declare_parameter("left_wheel_sign", 1.0)
        self.declare_parameter("right_wheel_sign", 1.0)

        # 비정상 dt 방지
        self.declare_parameter("min_dt", 1.0e-4)
        self.declare_parameter("max_dt", 0.5)

        self.joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        self.wheel_radius = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )
        self.wheelbase_length = (
            self.get_parameter("wheelbase_length").get_parameter_value().double_value
        )
        self.track_width = (
            self.get_parameter("track_width").get_parameter_value().double_value
        )

        self.left_wheel_joints = list(self.get_parameter("left_wheel_joints").value)
        self.right_wheel_joints = list(self.get_parameter("right_wheel_joints").value)
        self.front_steer_joints = list(self.get_parameter("front_steer_joints").value)

        self.steer_deadband = (
            self.get_parameter("steer_deadband").get_parameter_value().double_value
        )

        self.left_wheel_sign = (
            self.get_parameter("left_wheel_sign").get_parameter_value().double_value
        )
        self.right_wheel_sign = (
            self.get_parameter("right_wheel_sign").get_parameter_value().double_value
        )

        self.min_dt = self.get_parameter("min_dt").get_parameter_value().double_value
        self.max_dt = self.get_parameter("max_dt").get_parameter_value().double_value

        if self.track_width <= 0.0:
            raise ValueError("track_width must be greater than 0.0")

        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be greater than 0.0")

        if self.wheelbase_length <= 0.0:
            raise ValueError("wheelbase_length must be greater than 0.0")

        # -----------------------------
        # Internal State
        # -----------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_time: Optional[Time] = None

        self.warned_missing_joints = set()

        # -----------------------------
        # ROS Interfaces
        # -----------------------------
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )

        self.get_logger().info("Wheel Odometry Node initialized.")
        self.get_logger().info(f"Subscribing: {self.joint_state_topic}")
        self.get_logger().info(f"Publishing : {self.odom_topic}")
        self.get_logger().info(f"Left wheels : {self.left_wheel_joints}")
        self.get_logger().info(f"Right wheels: {self.right_wheel_joints}")
        self.get_logger().info(f"Front steer : {self.front_steer_joints}")
        self.get_logger().info(
            f"wheel_radius={self.wheel_radius}, "
            f"wheelbase_length={self.wheelbase_length}, track_width={self.track_width}"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        current_time = self.get_message_time(msg)

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds * 1.0e-9

        if dt < self.min_dt:
            return

        if dt > self.max_dt:
            self.get_logger().warn(
                f"Large dt detected: {dt:.3f}s. Skipping this odometry update."
            )
            self.prev_time = current_time
            return

        left_w = self.get_average_wheel_angular_velocity(
            msg=msg,
            target_joint_names=self.left_wheel_joints,
            side_sign=self.left_wheel_sign,
        )

        right_w = self.get_average_wheel_angular_velocity(
            msg=msg,
            target_joint_names=self.right_wheel_joints,
            side_sign=self.right_wheel_sign,
        )

        if left_w is None or right_w is None:
            self.prev_time = current_time
            return

        # angular velocity [rad/s] -> linear wheel velocity [m/s]
        v_left = self.wheel_radius * left_w
        v_right = self.wheel_radius * right_w

        linear_v = 0.5 * (v_right + v_left)

        steer_angle = self.get_average_joint_position(
            msg=msg,
            target_joint_names=self.front_steer_joints,
        )

        if steer_angle is not None and abs(steer_angle) > self.steer_deadband:
            # AAU rover의 일반 주행은 Ackermann steering에 가깝다.
            angular_w = linear_v * math.tan(steer_angle) / self.wheelbase_length
        else:
            # point-turn 또는 steering joint position이 없을 때의 보조 계산.
            angular_w = (v_right - v_left) / self.track_width

        self.integrate_pose(linear_v, angular_w, dt)

        self.publish_odometry(
            current_time=current_time,
            linear_v=linear_v,
            angular_w=angular_w,
        )

        self.prev_time = current_time

    def get_message_time(self, msg: JointState) -> Time:
        """
        JointState header stamp가 있으면 사용하고,
        없으면 node clock을 사용한다.
        """
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            return Time.from_msg(msg.header.stamp)

        return self.get_clock().now()

    def get_average_wheel_angular_velocity(
        self,
        msg: JointState,
        target_joint_names: List[str],
        side_sign: float,
    ) -> Optional[float]:
        """
        지정한 wheel joint들의 angular velocity 평균을 계산한다.
        """
        if not msg.name:
            self.get_logger().warn("JointState.name is empty.")
            return None

        if len(msg.velocity) != len(msg.name):
            self.get_logger().warn("JointState.velocity size does not match name size.")
            return None

        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        wheel_velocities = []

        for joint_name in target_joint_names:
            if joint_name not in name_to_index:
                if joint_name not in self.warned_missing_joints:
                    self.get_logger().warn(
                        f"Wheel joint '{joint_name}' not found in /joint_states."
                    )
                    self.warned_missing_joints.add(joint_name)
                continue

            idx = name_to_index[joint_name]
            velocity = msg.velocity[idx]

            if velocity is None:
                continue

            if not math.isfinite(velocity):
                continue

            wheel_velocities.append(side_sign * velocity)

        if not wheel_velocities:
            return None

        return sum(wheel_velocities) / len(wheel_velocities)

    def get_average_joint_position(
        self,
        msg: JointState,
        target_joint_names: List[str],
    ) -> Optional[float]:
        """
        지정한 steering joint들의 position 평균을 계산한다.
        """
        if len(msg.position) != len(msg.name):
            return None

        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        positions = []

        for joint_name in target_joint_names:
            if joint_name not in name_to_index:
                if joint_name not in self.warned_missing_joints:
                    self.get_logger().warn(
                        f"Steering joint '{joint_name}' not found in /joint_states."
                    )
                    self.warned_missing_joints.add(joint_name)
                continue

            position = msg.position[name_to_index[joint_name]]

            if math.isfinite(position):
                positions.append(position)

        if not positions:
            return None

        return sum(positions) / len(positions)

    def integrate_pose(self, linear_v: float, angular_w: float, dt: float) -> None:
        """
        midpoint integration으로 x, y, yaw를 적분한다.
        """
        delta_yaw = angular_w * dt
        mid_yaw = self.yaw + 0.5 * delta_yaw

        self.x += linear_v * math.cos(mid_yaw) * dt
        self.y += linear_v * math.sin(mid_yaw) * dt
        self.yaw = self.normalize_angle(self.yaw + delta_yaw)

    def publish_odometry(
        self,
        current_time: Time,
        linear_v: float,
        angular_w: float,
    ) -> None:
        odom = Odometry()

        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = self.quaternion_from_yaw(self.yaw)

        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_w

        self.odom_pub.publish(odom)

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()

        half_yaw = 0.5 * yaw
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)

        return q

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)

    node = WheelOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
