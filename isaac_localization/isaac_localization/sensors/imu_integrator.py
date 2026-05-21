#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3


class IMUIntegratorNode(Node):
    """
    IMU Integrator Node

    역할:
    - /imu/data를 subscribe한다.
    - IMU orientation이 유효하면 해당 orientation을 사용한다.
    - orientation이 없거나 사용하지 않는 경우 angular_velocity를 적분해 자세를 추정한다.
    - /rover/imu_odom으로 nav_msgs/Odometry를 publish한다.

    기본 설계:
    - 위치 x, y, z는 wheel odometry / TRN / EKF에서 주로 다룬다.
    - IMU Integrator는 roll, pitch, yaw와 angular velocity 정보를 제공하는 역할로 둔다.
    """

    def __init__(self):
        super().__init__("imu_integrator_node")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_odom_topic", "/rover/imu_odom")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        # IMU message에 orientation이 있으면 그대로 사용할지 여부
        self.declare_parameter("use_orientation_msg", True)

        # orientation이 없을 때 gyro 적분으로 orientation을 추정할지 여부
        self.declare_parameter("integrate_orientation_from_gyro", True)

        self.declare_parameter("min_dt", 1.0e-4)
        self.declare_parameter("max_dt", 0.5)

        self.imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.imu_odom_topic = (
            self.get_parameter("imu_odom_topic").get_parameter_value().string_value
        )

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        self.use_orientation_msg = (
            self.get_parameter("use_orientation_msg").get_parameter_value().bool_value
        )
        self.integrate_orientation_from_gyro = (
            self.get_parameter("integrate_orientation_from_gyro")
            .get_parameter_value()
            .bool_value
        )

        self.min_dt = self.get_parameter("min_dt").get_parameter_value().double_value
        self.max_dt = self.get_parameter("max_dt").get_parameter_value().double_value

        # -----------------------------
        # Internal State
        # -----------------------------
        self.prev_time: Optional[Time] = None

        # quaternion: x, y, z, w
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        self.last_angular_velocity = Vector3()
        self.last_linear_acceleration = Vector3()

        # -----------------------------
        # ROS Interfaces
        # -----------------------------
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.imu_odom_pub = self.create_publisher(
            Odometry,
            self.imu_odom_topic,
            10,
        )

        self.get_logger().info("IMU Integrator Node initialized.")
        self.get_logger().info(f"Subscribing: {self.imu_topic}")
        self.get_logger().info(f"Publishing : {self.imu_odom_topic}")
        self.get_logger().info(f"use_orientation_msg={self.use_orientation_msg}")

    def imu_callback(self, msg: Imu) -> None:
        current_time = self.get_message_time(msg)

        if self.prev_time is None:
            self.prev_time = current_time
            self.initialize_orientation_if_available(msg)
            self.update_last_measurements(msg)
            self.publish_imu_odometry(current_time)
            return

        dt = (current_time - self.prev_time).nanoseconds * 1.0e-9

        if dt < self.min_dt:
            return

        if dt > self.max_dt:
            self.get_logger().warn(
                f"Large dt detected: {dt:.3f}s. Skipping this IMU integration update."
            )
            self.prev_time = current_time
            self.update_last_measurements(msg)
            return

        self.update_orientation(msg, dt)

        self.update_last_measurements(msg)
        self.publish_imu_odometry(current_time)

        self.prev_time = current_time

    def get_message_time(self, msg: Imu) -> Time:
        """
        IMU header stamp가 있으면 사용하고,
        없으면 node clock을 사용한다.
        """
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            return Time.from_msg(msg.header.stamp)

        return self.get_clock().now()

    def initialize_orientation_if_available(self, msg: Imu) -> None:
        """
        첫 IMU 메시지에서 orientation이 유효하면 내부 quaternion을 초기화한다.
        """
        if self.use_orientation_msg and self.is_valid_orientation(msg):
            q = msg.orientation
            self.qx, self.qy, self.qz, self.qw = self.normalize_quaternion(
                q.x, q.y, q.z, q.w
            )

    def update_orientation(self, msg: Imu, dt: float) -> None:
        """
        orientation 업데이트 방식:
        1. IMU orientation이 유효하고 use_orientation_msg=True이면 그대로 사용
        2. 아니면 angular_velocity를 적분해서 quaternion 업데이트
        """
        if self.use_orientation_msg and self.is_valid_orientation(msg):
            q = msg.orientation
            self.qx, self.qy, self.qz, self.qw = self.normalize_quaternion(
                q.x, q.y, q.z, q.w
            )
            return

        if self.integrate_orientation_from_gyro:
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z
            self.integrate_gyro(wx, wy, wz, dt)

    def integrate_gyro(self, wx: float, wy: float, wz: float, dt: float) -> None:
        """
        Gyro angular velocity를 quaternion으로 적분한다.

        q_dot = 0.5 * q ⊗ omega
        omega = [0, wx, wy, wz]
        """
        qx, qy, qz, qw = self.qx, self.qy, self.qz, self.qw

        # q ⊗ omega
        # quaternion format: x, y, z, w
        dq_x = 0.5 * (qw * wx + qy * wz - qz * wy)
        dq_y = 0.5 * (qw * wy + qz * wx - qx * wz)
        dq_z = 0.5 * (qw * wz + qx * wy - qy * wx)
        dq_w = 0.5 * (-qx * wx - qy * wy - qz * wz)

        qx += dq_x * dt
        qy += dq_y * dt
        qz += dq_z * dt
        qw += dq_w * dt

        self.qx, self.qy, self.qz, self.qw = self.normalize_quaternion(
            qx, qy, qz, qw
        )

    def update_last_measurements(self, msg: Imu) -> None:
        self.last_angular_velocity = msg.angular_velocity
        self.last_linear_acceleration = msg.linear_acceleration

    def publish_imu_odometry(self, current_time: Time) -> None:
        odom = Odometry()

        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # 위치와 선속도는 IMU 단독으로 신뢰하지 않음
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.pose.pose.orientation = Quaternion(
            x=self.qx,
            y=self.qy,
            z=self.qz,
            w=self.qw,
        )

        odom.twist.twist.angular.x = self.last_angular_velocity.x
        odom.twist.twist.angular.y = self.last_angular_velocity.y
        odom.twist.twist.angular.z = self.last_angular_velocity.z

        odom.pose.covariance = self.make_pose_covariance()
        odom.twist.covariance = self.make_twist_covariance()

        self.imu_odom_pub.publish(odom)

    @staticmethod
    def is_valid_orientation(msg: Imu) -> bool:
        """
        sensor_msgs/Imu에서 orientation_covariance[0] == -1이면
        orientation estimate가 없다는 의미로 사용된다.
        """
        if len(msg.orientation_covariance) > 0 and msg.orientation_covariance[0] == -1.0:
            return False

        q = msg.orientation

        if not all(math.isfinite(v) for v in [q.x, q.y, q.z, q.w]):
            return False

        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)

        if norm < 1.0e-6:
            return False

        return True

    @staticmethod
    def normalize_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> Tuple[float, float, float, float]:
        norm = math.sqrt(x * x + y * y + z * z + w * w)

        if norm < 1.0e-12:
            return 0.0, 0.0, 0.0, 1.0

        return x / norm, y / norm, z / norm, w / norm

    def make_pose_covariance(self) -> List[float]:
        """
        6x6 covariance matrix flattened row-major.

        위치는 IMU 단독으로 신뢰하지 않기 때문에 큰 covariance를 둔다.
        orientation은 상대적으로 낮은 covariance를 둔다.
        """
        covariance = [0.0] * 36

        covariance[0] = 999.0    # x
        covariance[7] = 999.0    # y
        covariance[14] = 999.0   # z

        covariance[21] = 0.05        # roll
        covariance[28] = 0.05        # pitch
        covariance[35] = 0.05        # yaw

        return covariance

    def make_twist_covariance(self) -> List[float]:
        """
        angular velocity는 IMU에서 직접 들어오므로 낮은 covariance.
        linear velocity는 IMU odom에서 제공하지 않으므로 큰 covariance.
        """
        covariance = [0.0] * 36

        covariance[0] = 999.0    # linear x
        covariance[7] = 999.0    # linear y
        covariance[14] = 999.0   # linear z

        covariance[21] = 0.05        # angular x
        covariance[28] = 0.05        # angular y
        covariance[35] = 0.05        # angular z

        return covariance


def main(args=None):
    rclpy.init(args=args)

    node = IMUIntegratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
