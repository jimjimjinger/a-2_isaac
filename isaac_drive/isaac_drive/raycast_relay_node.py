"""raycast_relay_node — Isaac Sim 의 raycast IPC 파일을 picking 해 ROS2 토픽으로 발행.

Isaac Sim 5.1 PyPI venv 는 python3.11 — humble rclpy(3.10) 와 ABI 호환 안 되어
런타임 안에서 직접 publish 불가. 우회: run_vehicle_v3.py 가 /tmp/a2_raycast.npz
에 atomic write → 이 노드(system python3.10 + rclpy)가 polling 해 publish.

발행 토픽
---------
/rover/raycast/cloud           sensor_msgs/PointCloud2   — 676 ray hit 전체 (색: obs=red, flat=Mars, miss=gray)
/rover/raycast/obstacle_points sensor_msgs/PointCloud2   — obstacle 셀만
/rover/raycast/built_grid      nav_msgs/OccupancyGrid    — 빈 50×50m 격자에 누적된 obstacle map (Phase 2 핵심)

OccupancyGrid 셀 값
-------------------
-1: 한 번도 관측 안 됨 (백지)
 0: free (정상 ground 만 관측)
50~100: obstacle (관측 횟수 비율). 100 = 거의 매번 obstacle 로 관측.
"""
from __future__ import annotations

import os
import struct
import time
from typing import Optional

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

IPC_PATH_DEFAULT = "/tmp/a2_raycast.npz"


def _make_pointcloud2(header: Header, points_xyz: np.ndarray,
                      colors_rgb: Optional[np.ndarray] = None) -> PointCloud2:
    """numpy (N,3) float32 + optional (N,3) uint8 → PointCloud2.

    RGB 가 주어지면 packed uint32 rgb field 추가 (RViz Color Transformer=RGB8).
    """
    n = int(points_xyz.shape[0])
    pc = PointCloud2()
    pc.header = header
    pc.height = 1
    pc.width = n
    pc.is_bigendian = False
    pc.is_dense = True

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 12
    if colors_rgb is not None:
        fields.append(PointField(name="rgb", offset=12,
                                 datatype=PointField.UINT32, count=1))
        point_step = 16

    pc.fields = fields
    pc.point_step = point_step
    pc.row_step = point_step * n

    if n == 0:
        pc.data = b""
        return pc

    if colors_rgb is None:
        buf = points_xyz.astype(np.float32, copy=False).tobytes()
    else:
        rgb_packed = (
            (colors_rgb[:, 0].astype(np.uint32) << 16)
            | (colors_rgb[:, 1].astype(np.uint32) << 8)
            | (colors_rgb[:, 2].astype(np.uint32))
        )
        out = np.zeros(n, dtype=[
            ("x", np.float32), ("y", np.float32), ("z", np.float32),
            ("rgb", np.uint32),
        ])
        out["x"] = points_xyz[:, 0].astype(np.float32)
        out["y"] = points_xyz[:, 1].astype(np.float32)
        out["z"] = points_xyz[:, 2].astype(np.float32)
        out["rgb"] = rgb_packed
        buf = out.tobytes()

    pc.data = buf
    return pc


class RaycastRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("raycast_relay_node")

        self.declare_parameter("ipc_path", IPC_PATH_DEFAULT)
        self.declare_parameter("poll_hz", 10.0)
        self.declare_parameter("frame_id", "map")
        # EKF rover frame 기준으로 raycast 변환. EKF drift 가 obstacle 위치에
        # 반영돼 /map 원본 과 비교 시 위치 오차 가시화 가능.
        self.declare_parameter("transform_to_ekf_frame", True)
        self.declare_parameter("ekf_pose_topic", "/rover/estimated_pose")

        # Built grid 누적
        self.declare_parameter("grid_size_m", 50.0)         # 50×50 m
        self.declare_parameter("grid_resolution", 0.25)     # 0.25 m/cell → 200×200
        self.declare_parameter("grid_origin_x", -25.0)
        self.declare_parameter("grid_origin_y", -25.0)
        self.declare_parameter("min_obs_ratio", 0.5)        # obstacle 판정 임계 (count_obs / total)

        self.ipc_path = str(self.get_parameter("ipc_path").value)
        poll_hz = float(self.get_parameter("poll_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.transform_to_ekf = bool(
            self.get_parameter("transform_to_ekf_frame").value)
        ekf_pose_topic = str(self.get_parameter("ekf_pose_topic").value)

        size_m = float(self.get_parameter("grid_size_m").value)
        self.res = float(self.get_parameter("grid_resolution").value)
        self.origin_x = float(self.get_parameter("grid_origin_x").value)
        self.origin_y = float(self.get_parameter("grid_origin_y").value)
        self.min_obs_ratio = float(self.get_parameter("min_obs_ratio").value)

        self.gw = int(round(size_m / self.res))  # grid width (cols)
        self.gh = self.gw                         # square grid
        # 누적 카운터
        self.cnt_obs = np.zeros((self.gh, self.gw), dtype=np.uint32)
        self.cnt_free = np.zeros((self.gh, self.gw), dtype=np.uint32)

        self.pub_cloud = self.create_publisher(
            PointCloud2, "/rover/raycast/cloud", 5)
        self.pub_obs = self.create_publisher(
            PointCloud2, "/rover/raycast/obstacle_points", 5)
        self.pub_grid = self.create_publisher(
            OccupancyGrid, "/rover/raycast/built_grid", 1)

        # 최신 EKF rover pose (변환용)
        self._ekf_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._ekf_yaw: float = 0.0
        self._has_ekf: bool = False
        self.create_subscription(
            PoseWithCovarianceStamped, ekf_pose_topic, self._on_ekf, 10)

        # IPC 파일이 갱신됐을 때만 처리하려고 mtime 기억.
        self._last_mtime: float = 0.0
        self.timer = self.create_timer(1.0 / poll_hz, self._tick)

        # static TF: map = world (identity). mvp.launch.py 는 TF broadcast 안
        # 함 → RViz Fixed Frame=map 으로 두려면 직접 알려줘야 한다.
        self._tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = self.frame_id
        tf.transform.rotation.w = 1.0
        self._tf_static.sendTransform(tf)

        self.get_logger().info(
            f"raycast_relay 활성: ipc={self.ipc_path} poll={poll_hz}Hz "
            f"grid={self.gh}x{self.gw}@{self.res:.2f}m frame={self.frame_id} "
            f"(static TF world→{self.frame_id} broadcast) "
            f"transform_to_ekf={self.transform_to_ekf} "
            f"ekf_topic={ekf_pose_topic}")

    def _on_ekf(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self._ekf_xyz = (p.x, p.y, p.z)
        self._ekf_yaw = _quat_to_yaw(msg.pose.pose.orientation)
        self._has_ekf = True

    def _tick(self) -> None:
        if not os.path.isfile(self.ipc_path):
            return
        try:
            st = os.stat(self.ipc_path)
        except FileNotFoundError:
            return
        if st.st_mtime <= self._last_mtime:
            return
        self._last_mtime = st.st_mtime
        try:
            data = np.load(self.ipc_path)
            cell_xyz = data["cell_world_xyz"]      # (N, 3) float32
            cell_is_obs = data["cell_is_obs"]       # (N,) bool
            cell_is_miss = data["cell_is_miss"]     # (N,) bool
            sim_rover_pos = data["rover_pos"]       # (3,) float64
            sim_rover_yaw = float(data["rover_yaw"])
        except Exception as e:
            self.get_logger().warn(f"npz 로드 실패: {e}")
            return

        # Sim GT → rover-local → EKF world 변환.
        # 1) (cell_world - sim_rover) = sim 기준 world-axes offset
        # 2) -sim_yaw 로 회전 = rover body frame offset
        # 3) +ekf_yaw 로 회전 + ekf_rover 이동 = EKF 기준 world 좌표
        # EKF drift 가 그대로 obstacle 위치에 반영되어 /map 과 비교 시 차이 = drift.
        if self.transform_to_ekf and self._has_ekf:
            dx = cell_xyz[:, 0] - float(sim_rover_pos[0])
            dy = cell_xyz[:, 1] - float(sim_rover_pos[1])
            cs, ss = math.cos(-sim_rover_yaw), math.sin(-sim_rover_yaw)
            lx = cs * dx - ss * dy
            ly = ss * dx + cs * dy
            ce, se = math.cos(self._ekf_yaw), math.sin(self._ekf_yaw)
            new_x = self._ekf_xyz[0] + ce * lx - se * ly
            new_y = self._ekf_xyz[1] + se * lx + ce * ly
            cell_xyz = np.stack([new_x.astype(np.float32),
                                 new_y.astype(np.float32),
                                 cell_xyz[:, 2]], axis=-1)

        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id

        n = int(cell_xyz.shape[0])
        # 색: obs=red, flat=white, miss=gray(76,76,76)
        colors = np.zeros((n, 3), dtype=np.uint8)
        colors[:] = (255, 255, 255)
        colors[cell_is_obs] = (255, 25, 25)
        colors[cell_is_miss] = (76, 76, 76)

        # 1) /rover/raycast/cloud — 전체
        self.pub_cloud.publish(_make_pointcloud2(header, cell_xyz, colors))

        # 2) /rover/raycast/obstacle_points — obstacle 만
        if cell_is_obs.any():
            obs_xyz = cell_xyz[cell_is_obs]
            obs_colors = np.tile(np.array([[255, 25, 25]], dtype=np.uint8),
                                 (obs_xyz.shape[0], 1))
            self.pub_obs.publish(_make_pointcloud2(header, obs_xyz, obs_colors))
        else:
            self.pub_obs.publish(_make_pointcloud2(
                header, np.zeros((0, 3), dtype=np.float32)))

        # 3) Built grid 누적 — 미관측(miss/self) 셀은 카운트 안 함.
        valid = ~cell_is_miss
        if valid.any():
            self._accumulate(cell_xyz[valid], cell_is_obs[valid])
        self.pub_grid.publish(self._build_grid_msg(header))

    def _accumulate(self, xyz: np.ndarray, is_obs: np.ndarray) -> None:
        ix = np.floor((xyz[:, 0] - self.origin_x) / self.res).astype(np.int32)
        iy = np.floor((xyz[:, 1] - self.origin_y) / self.res).astype(np.int32)
        in_bounds = (ix >= 0) & (ix < self.gw) & (iy >= 0) & (iy < self.gh)
        ix = ix[in_bounds]
        iy = iy[in_bounds]
        obs = is_obs[in_bounds]
        if ix.size == 0:
            return
        # bincount-style update (per-cell 누적)
        flat = iy * self.gw + ix
        np.add.at(self.cnt_obs.ravel(), flat[obs], 1)
        np.add.at(self.cnt_free.ravel(), flat[~obs], 1)

    def _build_grid_msg(self, header: Header) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = header
        msg.info.map_load_time = header.stamp
        msg.info.resolution = float(self.res)
        msg.info.width = int(self.gw)
        msg.info.height = int(self.gh)
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        total = self.cnt_obs.astype(np.float32) + self.cnt_free.astype(np.float32)
        ratio = np.where(total > 0,
                         self.cnt_obs.astype(np.float32) / np.maximum(total, 1.0),
                         0.0)
        # 매핑: 미관측=-1, obstacle 확정=100, 일부=50~99, free=0
        cells = np.full((self.gh, self.gw), -1, dtype=np.int8)
        observed = total > 0
        # 비율을 0~100 으로 매핑하되 obstacle ratio>=min_obs_ratio 일 때 큰 값.
        scaled = np.clip(np.round(ratio * 100.0), 0, 100).astype(np.int8)
        cells[observed] = scaled[observed]
        # row-major (RViz/nav 표준). occupancy_grid: data[i*width + j].
        msg.data = cells.flatten().tolist()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaycastRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
