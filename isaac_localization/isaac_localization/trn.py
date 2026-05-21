#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


class TRNNode(Node):
    """
    TRN: Terrain Relative Navigation Node

    역할:
    - global_heightmap.npy와 heightmap_metadata.yaml을 로드한다.
    - /rover/wheel_odom을 prior pose로 사용한다.
    - /rover/local_height_patch를 받아 global heightmap과 매칭한다.
    - 가장 유사한 지형 위치를 찾아 /rover/trn_pose로 publish한다.

    현재 구현 가정:
    - local patch는 global heightmap과 같은 resolution을 가진다.
    - local patch는 map frame 축과 정렬되어 있다고 가정한다.
    - yaw까지 직접 추정하지 않고, yaw는 wheel odom prior 값을 사용한다.
    - TRN은 x, y, z 위치 보정값을 제공한다.
    """

    def __init__(self):
        super().__init__("trn_node")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("heightmap_path", "")
        self.declare_parameter("metadata_path", "")

        self.declare_parameter("prior_odom_topic", "/rover/wheel_odom")
        self.declare_parameter("local_patch_topic", "/rover/local_height_patch")
        self.declare_parameter("trn_pose_topic", "/rover/trn_pose")

        self.declare_parameter("frame_id", "map")

        # wheel odom prior 주변 몇 m 안에서 탐색할지
        self.declare_parameter("search_radius_m", 2.0)

        # 탐색 간격. 1이면 모든 grid cell 탐색
        self.declare_parameter("search_step_cells", 1)

        # local patch shape를 layout에서 못 읽을 때 사용할 기본 크기
        self.declare_parameter("default_patch_rows", 21)
        self.declare_parameter("default_patch_cols", 21)

        # matching confidence 계산용 sigma
        self.declare_parameter("match_sigma", 0.20)

        # confidence가 이 값보다 낮으면 publish하지 않음
        self.declare_parameter("min_confidence", 0.20)

        # height offset 제거 여부
        # True이면 local/global patch 모두 평균 높이를 빼고 비교함
        self.declare_parameter("normalize_patch_height", True)

        # covariance 기본값
        self.declare_parameter("base_xy_covariance", 0.05)
        self.declare_parameter("base_z_covariance", 0.10)
        self.declare_parameter("yaw_covariance", 999.0)

        self.heightmap_path = self.get_parameter("heightmap_path").value
        self.metadata_path = self.get_parameter("metadata_path").value

        self.prior_odom_topic = self.get_parameter("prior_odom_topic").value
        self.local_patch_topic = self.get_parameter("local_patch_topic").value
        self.trn_pose_topic = self.get_parameter("trn_pose_topic").value

        self.frame_id = self.get_parameter("frame_id").value

        self.search_radius_m = float(self.get_parameter("search_radius_m").value)
        self.search_step_cells = int(self.get_parameter("search_step_cells").value)

        self.default_patch_rows = int(self.get_parameter("default_patch_rows").value)
        self.default_patch_cols = int(self.get_parameter("default_patch_cols").value)

        self.match_sigma = float(self.get_parameter("match_sigma").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)

        self.normalize_patch_height = bool(
            self.get_parameter("normalize_patch_height").value
        )

        self.base_xy_covariance = float(
            self.get_parameter("base_xy_covariance").value
        )
        self.base_z_covariance = float(
            self.get_parameter("base_z_covariance").value
        )
        self.yaw_covariance = float(
            self.get_parameter("yaw_covariance").value
        )

        if self.search_step_cells < 1:
            self.search_step_cells = 1

        if self.match_sigma <= 0.0:
            self.match_sigma = 0.20

        # -----------------------------
        # Load map
        # -----------------------------
        self.heightmap, self.metadata = self.load_heightmap()

        self.resolution = float(self.metadata["resolution"])
        self.origin_x = float(self.metadata["origin_x"])
        self.origin_y = float(self.metadata["origin_y"])
        self.map_width = int(self.metadata["width"])
        self.map_height = int(self.metadata["height"])

        if self.heightmap.shape != (self.map_height, self.map_width):
            raise ValueError(
                "Heightmap shape does not match metadata. "
                f"heightmap.shape={self.heightmap.shape}, "
                f"metadata height/width=({self.map_height}, {self.map_width})"
            )

        # -----------------------------
        # Internal prior state
        # -----------------------------
        self.prior_x: Optional[float] = None
        self.prior_y: Optional[float] = None
        self.prior_z: Optional[float] = None
        self.prior_yaw: float = 0.0
        self.prior_stamp: Optional[Time] = None

        # -----------------------------
        # ROS Interfaces
        # -----------------------------
        self.prior_sub = self.create_subscription(
            Odometry,
            self.prior_odom_topic,
            self.prior_odom_callback,
            10,
        )

        self.patch_sub = self.create_subscription(
            Float32MultiArray,
            self.local_patch_topic,
            self.local_patch_callback,
            10,
        )

        self.trn_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.trn_pose_topic,
            10,
        )

        self.get_logger().info("TRN Node initialized.")
        self.get_logger().info(f"Heightmap shape: {self.heightmap.shape}")
        self.get_logger().info(f"Resolution     : {self.resolution} m/cell")
        self.get_logger().info(f"Origin         : ({self.origin_x}, {self.origin_y})")
        self.get_logger().info(f"Prior topic    : {self.prior_odom_topic}")
        self.get_logger().info(f"Patch topic    : {self.local_patch_topic}")
        self.get_logger().info(f"TRN output     : {self.trn_pose_topic}")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def prior_odom_callback(self, msg: Odometry) -> None:
        self.prior_x = msg.pose.pose.position.x
        self.prior_y = msg.pose.pose.position.y
        self.prior_z = msg.pose.pose.position.z
        self.prior_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.prior_stamp = Time.from_msg(msg.header.stamp)

    def local_patch_callback(self, msg: Float32MultiArray) -> None:
        if self.prior_x is None or self.prior_y is None:
            self.get_logger().warn("No prior odometry received yet. Skipping TRN update.")
            return

        local_patch = self.parse_local_patch(msg)

        if local_patch is None:
            self.get_logger().warn("Failed to parse local height patch.")
            return

        self.run_trn_update(local_patch)

    # ------------------------------------------------------------------
    # Main TRN logic
    # ------------------------------------------------------------------

    def run_trn_update(self, local_patch: np.ndarray) -> None:
        if self.prior_x is None or self.prior_y is None:
            return

        result = self.match_local_patch(
            local_patch=local_patch,
            prior_x=self.prior_x,
            prior_y=self.prior_y,
        )

        if result is None:
            self.get_logger().warn("TRN matching failed. No valid candidate found.")
            return

        best_x, best_y, best_z, best_rmse, confidence = result

        if confidence < self.min_confidence:
            self.get_logger().warn(
                f"TRN confidence too low: {confidence:.3f}, rmse={best_rmse:.3f}. "
                "Skipping publish."
            )
            return

        stamp = self.get_clock().now()
        if self.prior_stamp is not None:
            stamp = self.prior_stamp

        self.publish_trn_pose(
            stamp=stamp,
            x=best_x,
            y=best_y,
            z=best_z,
            yaw=self.prior_yaw,
            confidence=confidence,
        )

        self.get_logger().info(
            f"TRN matched: x={best_x:.3f}, y={best_y:.3f}, z={best_z:.3f}, "
            f"rmse={best_rmse:.3f}, confidence={confidence:.3f}"
        )

    def match_local_patch(
        self,
        local_patch: np.ndarray,
        prior_x: float,
        prior_y: float,
    ) -> Optional[Tuple[float, float, float, float, float]]:
        """
        prior pose 주변에서 local patch와 가장 유사한 global patch를 찾는다.

        반환:
        (best_x, best_y, best_z, best_rmse, confidence)
        """
        rows, cols = local_patch.shape

        prior_i, prior_j = self.world_to_grid(prior_x, prior_y)

        search_radius_cells = max(1, int(round(self.search_radius_m / self.resolution)))

        half_rows = rows // 2
        half_cols = cols // 2

        best_rmse = float("inf")
        best_i = None
        best_j = None

        local_prepared = self.prepare_patch(local_patch)

        for candidate_j in range(
            prior_j - search_radius_cells,
            prior_j + search_radius_cells + 1,
            self.search_step_cells,
        ):
            for candidate_i in range(
                prior_i - search_radius_cells,
                prior_i + search_radius_cells + 1,
                self.search_step_cells,
            ):
                if not self.is_patch_inside_map(candidate_i, candidate_j, half_rows, half_cols):
                    continue

                global_patch = self.extract_patch_by_grid_center(
                    center_i=candidate_i,
                    center_j=candidate_j,
                    rows=rows,
                    cols=cols,
                )

                if global_patch is None:
                    continue

                global_prepared = self.prepare_patch(global_patch)

                rmse = self.compute_rmse(local_prepared, global_prepared)

                if rmse < best_rmse:
                    best_rmse = rmse
                    best_i = candidate_i
                    best_j = candidate_j

        if best_i is None or best_j is None:
            return None

        best_x, best_y = self.grid_to_world(best_i, best_j)
        best_z = float(self.heightmap[best_j, best_i])

        confidence = self.rmse_to_confidence(best_rmse)

        return best_x, best_y, best_z, best_rmse, confidence

    # ------------------------------------------------------------------
    # Patch handling
    # ------------------------------------------------------------------

    def parse_local_patch(self, msg: Float32MultiArray) -> Optional[np.ndarray]:
        """
        Float32MultiArray에서 local height patch를 2D numpy array로 변환한다.

        우선순위:
        1. msg.layout.dim[0].size, msg.layout.dim[1].size 사용
        2. default_patch_rows/default_patch_cols 사용
        3. data 길이가 정사각형이면 sqrt 기반 shape 사용
        """
        data = np.asarray(msg.data, dtype=np.float32)

        if data.size == 0:
            return None

        rows = None
        cols = None

        if len(msg.layout.dim) >= 2:
            rows = int(msg.layout.dim[0].size)
            cols = int(msg.layout.dim[1].size)

        if rows is None or cols is None or rows <= 0 or cols <= 0:
            rows = self.default_patch_rows
            cols = self.default_patch_cols

        if rows * cols != data.size:
            side = int(round(math.sqrt(data.size)))
            if side * side == data.size:
                rows = side
                cols = side
            else:
                self.get_logger().warn(
                    f"Patch size mismatch. data={data.size}, rows={rows}, cols={cols}"
                )
                return None

        patch = data.reshape((rows, cols))

        if not np.all(np.isfinite(patch)):
            patch = np.nan_to_num(patch, nan=0.0, posinf=0.0, neginf=0.0)

        return patch

    def prepare_patch(self, patch: np.ndarray) -> np.ndarray:
        prepared = patch.astype(np.float32).copy()

        if self.normalize_patch_height:
            prepared -= float(np.mean(prepared))

        return prepared

    @staticmethod
    def compute_rmse(a: np.ndarray, b: np.ndarray) -> float:
        diff = a - b
        return float(np.sqrt(np.mean(diff * diff)))

    def rmse_to_confidence(self, rmse: float) -> float:
        """
        RMSE를 0~1 confidence로 변환한다.
        """
        return float(math.exp(-0.5 * (rmse / self.match_sigma) ** 2))

    # ------------------------------------------------------------------
    # Heightmap access
    # ------------------------------------------------------------------

    def extract_patch_by_world_center(
        self,
        center_x: float,
        center_y: float,
        rows: int,
        cols: int,
    ) -> Optional[np.ndarray]:
        center_i, center_j = self.world_to_grid(center_x, center_y)
        return self.extract_patch_by_grid_center(center_i, center_j, rows, cols)

    def extract_patch_by_grid_center(
        self,
        center_i: int,
        center_j: int,
        rows: int,
        cols: int,
    ) -> Optional[np.ndarray]:
        half_rows = rows // 2
        half_cols = cols // 2

        if not self.is_patch_inside_map(center_i, center_j, half_rows, half_cols):
            return None

        row_start = center_j - half_rows
        row_end = row_start + rows

        col_start = center_i - half_cols
        col_end = col_start + cols

        return self.heightmap[row_start:row_end, col_start:col_end].copy()

    def is_patch_inside_map(
        self,
        center_i: int,
        center_j: int,
        half_rows: int,
        half_cols: int,
    ) -> bool:
        row_start = center_j - half_rows
        row_end = center_j + half_rows + 1

        col_start = center_i - half_cols
        col_end = center_i + half_cols + 1

        if row_start < 0 or col_start < 0:
            return False

        if row_end > self.map_height or col_end > self.map_width:
            return False

        return True

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        i = int(round((x - self.origin_x) / self.resolution))
        j = int(round((y - self.origin_y) / self.resolution))

        i = max(0, min(self.map_width - 1, i))
        j = max(0, min(self.map_height - 1, j))

        return i, j

    def grid_to_world(self, i: int, j: int) -> Tuple[float, float]:
        x = self.origin_x + i * self.resolution
        y = self.origin_y + j * self.resolution
        return x, y

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def publish_trn_pose(
        self,
        stamp: Time,
        x: float,
        y: float,
        z: float,
        yaw: float,
        confidence: float,
    ) -> None:
        msg = PoseWithCovarianceStamped()

        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z

        msg.pose.pose.orientation = self.quaternion_from_yaw(yaw)

        msg.pose.covariance = self.make_covariance(confidence)

        self.trn_pose_pub.publish(msg)

    def make_covariance(self, confidence: float):
        """
        confidence가 높을수록 covariance를 작게 둔다.
        """
        confidence = max(0.05, min(1.0, confidence))

        xy_cov = self.base_xy_covariance / confidence
        z_cov = self.base_z_covariance / confidence

        covariance = [0.0] * 36

        covariance[0] = xy_cov
        covariance[7] = xy_cov
        covariance[14] = z_cov

        # TRN 현재 버전은 yaw를 직접 추정하지 않으므로 큰 covariance
        covariance[21] = 999.0
        covariance[28] = 999.0
        covariance[35] = self.yaw_covariance

        return covariance

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def load_heightmap(self) -> Tuple[np.ndarray, Dict[str, str]]:
        """
        heightmap과 metadata를 로드한다.

        파라미터로 경로가 비어 있으면 기본 경로를 사용한다.
        기본 경로:
        isaac_localization/maps/global_heightmap.npy
        isaac_localization/maps/heightmap_metadata.yaml
        """
        package_dir = Path(__file__).resolve().parent

        if not self.heightmap_path:
            heightmap_path = package_dir / "maps" / "global_heightmap.npy"
        else:
            heightmap_path = Path(self.heightmap_path)

        if not self.metadata_path:
            metadata_path = package_dir / "maps" / "heightmap_metadata.yaml"
        else:
            metadata_path = Path(self.metadata_path)

        if not heightmap_path.exists():
            raise FileNotFoundError(f"Heightmap file not found: {heightmap_path}")

        if not metadata_path.exists():
            raise FileNotFoundError(f"Metadata file not found: {metadata_path}")

        heightmap = np.load(heightmap_path).astype(np.float32)
        metadata = self.parse_simple_yaml(metadata_path)

        required = ["resolution", "origin_x", "origin_y", "width", "height"]
        for key in required:
            if key not in metadata:
                raise KeyError(f"Missing key '{key}' in {metadata_path}")

        return heightmap, metadata

    @staticmethod
    def parse_simple_yaml(path: Path) -> Dict[str, str]:
        """
        PyYAML 의존성을 피하기 위한 단순 YAML parser.
        key: value 형태만 읽는다.
        """
        metadata: Dict[str, str] = {}

        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()

            if not line:
                continue

            if line.startswith("#"):
                continue

            if ":" not in line:
                continue

            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()

            if value.startswith('"') and value.endswith('"'):
                value = value[1:-1]

            if value.startswith("'") and value.endswith("'"):
                value = value[1:-1]

            metadata[key] = value

        return metadata

    # ------------------------------------------------------------------
    # Quaternion helpers
    # ------------------------------------------------------------------

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(0.5 * yaw)
        q.w = math.cos(0.5 * yaw)
        return q

    @staticmethod
    def yaw_from_quaternion(q: Quaternion) -> float:
        """
        quaternion에서 yaw 추출.
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)

    node = TRNNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
