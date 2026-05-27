#!/usr/bin/env python3

import json
import math
import os
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


def _resolve_a2_root(caller_file: str) -> Path:
    """팀 어느 머신에서도 작동하도록 a2_isaac 패키지 루트 해석.
    우선순위: A2_ISAAC_ROOT env → 같은 colcon workspace src → installed.
    """
    env = os.environ.get("A2_ISAAC_ROOT")
    if env:
        return Path(env)
    caller_path = Path(caller_file).resolve()
    for parent in caller_path.parents:
        if parent.name == "a2_isaac" and (parent / "isaac_sim").exists():
            return parent
        candidate = parent / "src" / "a2_isaac"
        if (candidate / "isaac_sim").exists():
            return candidate
    return caller_path.parents[2]


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
        self.declare_parameter("obstacle_grid_path", "")
        self.declare_parameter("terrain_id", "terrain_00001")
        self.declare_parameter("terrain_root", "")

        self.declare_parameter("prior_odom_topic", "/rover/wheel_odom")
        self.declare_parameter("local_patch_topic", "/rover/local_height_patch")
        self.declare_parameter("local_obstacle_topic", "/rover/local_obstacle_patch")
        self.declare_parameter("trn_pose_topic", "/rover/trn_pose")

        self.declare_parameter("frame_id", "map")

        # wheel odom prior 주변 몇 m 안에서 탐색할지
        self.declare_parameter("search_radius_m", 2.0)

        # 탐색 간격. 1이면 모든 grid cell 탐색
        self.declare_parameter("search_step_cells", 4)

        # local patch shape를 layout에서 못 읽을 때 사용할 기본 크기
        self.declare_parameter("default_patch_rows", 121)
        self.declare_parameter("default_patch_cols", 121)

        # local_height_patch_node와 같은 기준. Patch는 로버 중심이 아니라
        # 로버 전방 관측 영역 중심을 기준으로 만들어진다.
        self.declare_parameter("patch_center_x", 3.0)
        self.declare_parameter("patch_center_y", 0.0)

        # matching confidence 계산용 sigma
        self.declare_parameter("match_sigma", 0.20)
        self.declare_parameter("height_weight", 1.0)
        self.declare_parameter("slope_weight", 0.35)
        self.declare_parameter("obstacle_weight", 0.50)
        self.declare_parameter("obstacle_dilation_m", 1.25)
        self.declare_parameter("obstacle_distance_sigma_m", 0.75)
        self.declare_parameter("obstacle_distance_max_m", 2.0)
        self.declare_parameter("yaw_search_max_deg", 12.0)
        self.declare_parameter("yaw_search_step_deg", 4.0)
        self.declare_parameter("slope_sigma_deg", 12.0)
        self.declare_parameter("local_obstacle_slope_deg", 25.0)
        self.declare_parameter("local_obstacle_height_delta_m", 0.18)
        self.declare_parameter("min_feature_overlap_cells", 100)
        self.declare_parameter("prior_weight", 0.0)
        self.declare_parameter("prior_sigma_m", 1.0)
        # 비슷한 후보가 여러 곳이면 TRN을 덜 신뢰한다. 같은 봉우리 주변의
        # 이웃 셀은 제외하고, 일정 거리 이상 떨어진 second-best와 비교한다.
        self.declare_parameter("distinct_candidate_separation_m", 0.75)
        self.declare_parameter("ambiguity_margin_m", 0.04)

        # confidence가 이 값보다 낮으면 publish하지 않음
        self.declare_parameter("min_confidence", 0.20)
        self.declare_parameter("min_publish_local_obstacle_ratio", 0.08)
        self.declare_parameter("max_publish_obstacle_score", 0.55)
        self.declare_parameter("min_publish_ambiguity_margin", 0.08)

        # height offset 제거 여부
        # True이면 local/global patch 모두 평균 높이를 빼고 비교함
        self.declare_parameter("normalize_patch_height", True)

        # covariance 기본값
        self.declare_parameter("base_xy_covariance", 0.05)
        self.declare_parameter("base_z_covariance", 0.10)
        self.declare_parameter("yaw_covariance", 999.0)

        self.heightmap_path = self.get_parameter("heightmap_path").value
        self.metadata_path = self.get_parameter("metadata_path").value
        self.obstacle_grid_path = self.get_parameter("obstacle_grid_path").value
        self.terrain_id = self.get_parameter("terrain_id").value
        self.terrain_root = self.get_parameter("terrain_root").value

        self.prior_odom_topic = self.get_parameter("prior_odom_topic").value
        self.local_patch_topic = self.get_parameter("local_patch_topic").value
        self.local_obstacle_topic = self.get_parameter("local_obstacle_topic").value
        self.trn_pose_topic = self.get_parameter("trn_pose_topic").value

        self.frame_id = self.get_parameter("frame_id").value

        self.search_radius_m = float(self.get_parameter("search_radius_m").value)
        self.search_step_cells = int(self.get_parameter("search_step_cells").value)

        self.default_patch_rows = int(self.get_parameter("default_patch_rows").value)
        self.default_patch_cols = int(self.get_parameter("default_patch_cols").value)
        self.patch_center_x = float(self.get_parameter("patch_center_x").value)
        self.patch_center_y = float(self.get_parameter("patch_center_y").value)

        self.match_sigma = float(self.get_parameter("match_sigma").value)
        self.height_weight = float(self.get_parameter("height_weight").value)
        self.slope_weight = float(self.get_parameter("slope_weight").value)
        self.obstacle_weight = float(self.get_parameter("obstacle_weight").value)
        self.obstacle_dilation_m = float(
            self.get_parameter("obstacle_dilation_m").value
        )
        self.obstacle_distance_sigma_m = float(
            self.get_parameter("obstacle_distance_sigma_m").value
        )
        self.obstacle_distance_max_m = float(
            self.get_parameter("obstacle_distance_max_m").value
        )
        self.yaw_search_max_deg = float(
            self.get_parameter("yaw_search_max_deg").value
        )
        self.yaw_search_step_deg = float(
            self.get_parameter("yaw_search_step_deg").value
        )
        self.slope_sigma_deg = float(self.get_parameter("slope_sigma_deg").value)
        self.local_obstacle_slope_deg = float(
            self.get_parameter("local_obstacle_slope_deg").value
        )
        self.local_obstacle_height_delta_m = float(
            self.get_parameter("local_obstacle_height_delta_m").value
        )
        self.min_feature_overlap_cells = int(
            self.get_parameter("min_feature_overlap_cells").value
        )
        self.prior_weight = float(self.get_parameter("prior_weight").value)
        self.prior_sigma_m = float(self.get_parameter("prior_sigma_m").value)
        self.distinct_candidate_separation_m = float(
            self.get_parameter("distinct_candidate_separation_m").value
        )
        self.ambiguity_margin_m = float(
            self.get_parameter("ambiguity_margin_m").value
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.min_publish_local_obstacle_ratio = float(
            self.get_parameter("min_publish_local_obstacle_ratio").value
        )
        self.max_publish_obstacle_score = float(
            self.get_parameter("max_publish_obstacle_score").value
        )
        self.min_publish_ambiguity_margin = float(
            self.get_parameter("min_publish_ambiguity_margin").value
        )

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
        if self.slope_sigma_deg <= 0.0:
            self.slope_sigma_deg = 12.0
        if self.obstacle_dilation_m < 0.0:
            self.obstacle_dilation_m = 0.0
        if self.obstacle_distance_sigma_m <= 0.0:
            self.obstacle_distance_sigma_m = 0.75
        if self.obstacle_distance_max_m <= 0.0:
            self.obstacle_distance_max_m = 2.0
        if self.yaw_search_max_deg < 0.0:
            self.yaw_search_max_deg = 0.0
        if self.yaw_search_step_deg <= 0.0:
            self.yaw_search_step_deg = max(1.0, self.yaw_search_max_deg)
        if self.min_feature_overlap_cells < 1:
            self.min_feature_overlap_cells = 1
        if self.prior_weight < 0.0:
            self.prior_weight = 0.0
        if self.prior_sigma_m <= 0.0:
            self.prior_sigma_m = 1.0
        if self.distinct_candidate_separation_m < 0.0:
            self.distinct_candidate_separation_m = 0.0
        if self.ambiguity_margin_m <= 0.0:
            self.ambiguity_margin_m = 0.04
        if self.min_publish_local_obstacle_ratio < 0.0:
            self.min_publish_local_obstacle_ratio = 0.0
        if self.max_publish_obstacle_score < 0.0:
            self.max_publish_obstacle_score = 0.0
        if self.min_publish_ambiguity_margin < 0.0:
            self.min_publish_ambiguity_margin = 0.0

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
        self.global_slope = self.compute_slope_deg(self.heightmap, self.resolution)
        self.obstacle_grid = self.load_obstacle_grid()
        if self.obstacle_grid.shape != self.heightmap.shape:
            raise ValueError(
                "Obstacle grid shape does not match heightmap. "
                f"obstacle_grid.shape={self.obstacle_grid.shape}, "
                f"heightmap.shape={self.heightmap.shape}"
            )
        obstacle_dilation_cells = int(round(self.obstacle_dilation_m / self.resolution))
        self.obstacle_grid_match = self.dilate_binary_mask(
            self.obstacle_grid,
            obstacle_dilation_cells,
        )
        self.obstacle_distance_map = self.compute_obstacle_distance_map(
            self.obstacle_grid,
            self.resolution,
        )
        self.yaw_offsets = self.build_yaw_offsets(
            self.yaw_search_max_deg,
            self.yaw_search_step_deg,
        )

        # -----------------------------
        # Internal prior state
        # -----------------------------
        self.prior_x: Optional[float] = None
        self.prior_y: Optional[float] = None
        self.prior_z: Optional[float] = None
        self.prior_yaw: float = 0.0
        self.prior_stamp: Optional[Time] = None
        self.latest_local_obstacle_patch: Optional[np.ndarray] = None
        self.warned_missing_obstacle_patch = False

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

        self.obstacle_sub = self.create_subscription(
            Float32MultiArray,
            self.local_obstacle_topic,
            self.local_obstacle_callback,
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
        self.get_logger().info(f"Terrain ID     : {self.terrain_id}")
        self.get_logger().info(f"Prior topic    : {self.prior_odom_topic}")
        self.get_logger().info(f"Patch topic    : {self.local_patch_topic}")
        self.get_logger().info(f"Obstacle topic : {self.local_obstacle_topic}")
        self.get_logger().info(f"TRN output     : {self.trn_pose_topic}")
        self.get_logger().info(
            f"Patch center   : base x={self.patch_center_x:.3f}, "
            f"y={self.patch_center_y:.3f}"
        )
        self.get_logger().info(
            "Feature weights : "
            f"height={self.height_weight:.2f}, "
            f"slope={self.slope_weight:.2f}, "
            f"obstacle={self.obstacle_weight:.2f}"
        )
        self.get_logger().info(
            f"Obstacle matching dilation: {self.obstacle_dilation_m:.2f} m"
        )
        self.get_logger().info(
            "Obstacle distance score: "
            f"sigma={self.obstacle_distance_sigma_m:.2f} m, "
            f"max={self.obstacle_distance_max_m:.2f} m"
        )
        self.get_logger().info(
            "Yaw search       : "
            f"{[round(math.degrees(v), 1) for v in self.yaw_offsets]} deg"
        )

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

        local_obstacle_patch = self.latest_local_obstacle_patch
        if local_obstacle_patch is not None and local_obstacle_patch.shape != local_patch.shape:
            self.get_logger().warn(
                "Local obstacle patch shape mismatch. "
                f"height={local_patch.shape}, obstacle={local_obstacle_patch.shape}. "
                "Falling back to height-derived obstacle mask."
            )
            local_obstacle_patch = None

        if local_obstacle_patch is None and not self.warned_missing_obstacle_patch:
            self.get_logger().warn(
                "No local obstacle patch received yet. "
                "Falling back to height-derived obstacle mask."
            )
            self.warned_missing_obstacle_patch = True

        self.run_trn_update(local_patch, local_obstacle_patch)

    def local_obstacle_callback(self, msg: Float32MultiArray) -> None:
        local_obstacle_patch = self.parse_local_patch(msg)
        if local_obstacle_patch is None:
            self.get_logger().warn("Failed to parse local obstacle patch.")
            return
        local_obstacle_patch = local_obstacle_patch.astype(np.float32)
        finite = np.isfinite(local_obstacle_patch)
        local_obstacle_patch[finite] = np.where(
            local_obstacle_patch[finite] > 0.5,
            1.0,
            0.0,
        )
        self.latest_local_obstacle_patch = local_obstacle_patch

    # ------------------------------------------------------------------
    # Main TRN logic
    # ------------------------------------------------------------------

    def run_trn_update(
        self,
        local_patch: np.ndarray,
        local_obstacle_patch: Optional[np.ndarray] = None,
    ) -> None:
        if self.prior_x is None or self.prior_y is None:
            return

        result = self.match_local_patch(
            local_patch=local_patch,
            local_obstacle_patch=local_obstacle_patch,
            prior_x=self.prior_x,
            prior_y=self.prior_y,
        )

        if result is None:
            self.get_logger().warn("TRN matching failed. No valid candidate found.")
            return

        (
            best_x,
            best_y,
            best_z,
            best_rmse,
            confidence,
            second_rmse,
            ambiguity_margin,
            height_rmse,
            slope_rmse,
            obstacle_mismatch,
            overlap_count,
            prior_dist,
            local_obstacle_ratio,
            global_obstacle_ratio,
            best_yaw_delta,
        ) = result

        reject_reasons = []
        if confidence < self.min_confidence:
            reject_reasons.append(
                f"confidence {confidence:.3f} < {self.min_confidence:.3f}"
            )
        if local_obstacle_ratio < self.min_publish_local_obstacle_ratio:
            reject_reasons.append(
                "local_obs "
                f"{local_obstacle_ratio:.2f} < {self.min_publish_local_obstacle_ratio:.2f}"
            )
        if obstacle_mismatch > self.max_publish_obstacle_score:
            reject_reasons.append(
                f"obs_score {obstacle_mismatch:.2f} > {self.max_publish_obstacle_score:.2f}"
            )
        if math.isfinite(ambiguity_margin) and ambiguity_margin < self.min_publish_ambiguity_margin:
            reject_reasons.append(
                "margin "
                f"{ambiguity_margin:.3f} < {self.min_publish_ambiguity_margin:.3f}"
            )

        if reject_reasons:
            self.get_logger().warn(
                f"TRN quality rejected: {', '.join(reject_reasons)}. "
                f"candidate=({best_x:.3f}, {best_y:.3f}, {best_z:.3f}), "
                f"score={best_rmse:.3f}, second={second_rmse:.3f}, "
                f"margin={ambiguity_margin:.3f}, "
                f"h={height_rmse:.3f}, slope={slope_rmse:.2f}, "
                f"obs={obstacle_mismatch:.2f}, prior={prior_dist:.2f}m, "
                f"yaw_delta={math.degrees(best_yaw_delta):.1f}deg, "
                f"overlap={overlap_count}, "
                f"local_obs={local_obstacle_ratio:.2f}, "
                f"map_obs={global_obstacle_ratio:.2f}. "
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
            yaw=self.prior_yaw + best_yaw_delta,
            confidence=confidence,
        )

        self.get_logger().info(
            f"TRN matched: x={best_x:.3f}, y={best_y:.3f}, z={best_z:.3f}, "
            f"score={best_rmse:.3f}, second={second_rmse:.3f}, "
            f"margin={ambiguity_margin:.3f}, confidence={confidence:.3f}, "
            f"h={height_rmse:.3f}, slope={slope_rmse:.2f}, "
            f"obs={obstacle_mismatch:.2f}, prior={prior_dist:.2f}m, "
            f"yaw_delta={math.degrees(best_yaw_delta):.1f}deg, "
            f"overlap={overlap_count}, "
            f"local_obs={local_obstacle_ratio:.2f}, "
            f"map_obs={global_obstacle_ratio:.2f}"
        )

    def match_local_patch(
        self,
        local_patch: np.ndarray,
        local_obstacle_patch: Optional[np.ndarray],
        prior_x: float,
        prior_y: float,
    ) -> Optional[Tuple[float, float, float, float, float, float, float, float, float, float, int, float, float, float, float]]:
        """
        prior pose 주변에서 local patch와 가장 유사한 global patch를 찾는다.

        반환:
        (best_x, best_y, best_z, best_score, confidence, second_score,
        ambiguity_margin, height_rmse, slope_rmse, obstacle_mismatch, overlap_count,
        prior_dist, local_obstacle_ratio, global_obstacle_ratio, best_yaw_delta)
        """
        rows, cols = local_patch.shape

        prior_i, prior_j = self.world_to_grid(prior_x, prior_y)

        search_radius_cells = max(1, int(round(self.search_radius_m / self.resolution)))

        half_rows = rows // 2
        half_cols = cols // 2

        candidates = []
        yaw_variants = self.prepare_yaw_variants(local_patch, local_obstacle_patch)

        for yaw_delta, local_features, local_obstacle_ratio in yaw_variants:
            candidate_yaw = self.prior_yaw + yaw_delta

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
                    candidate_x, candidate_y = self.grid_to_world(
                        candidate_i,
                        candidate_j,
                    )
                    patch_center_x, patch_center_y = self.patch_center_world(
                        rover_x=candidate_x,
                        rover_y=candidate_y,
                        yaw=candidate_yaw,
                    )
                    patch_center_i, patch_center_j = self.world_to_grid(
                        patch_center_x,
                        patch_center_y,
                    )

                    global_patch = self.extract_patch_by_grid_center(
                        center_i=patch_center_i,
                        center_j=patch_center_j,
                        rows=rows,
                        cols=cols,
                    )

                    if global_patch is None:
                        continue

                    global_slope_patch = self.extract_array_patch_by_grid_center(
                        self.global_slope,
                        center_i=patch_center_i,
                        center_j=patch_center_j,
                        rows=rows,
                        cols=cols,
                    )
                    global_obstacle_patch = self.extract_array_patch_by_grid_center(
                        self.obstacle_grid,
                        center_i=patch_center_i,
                        center_j=patch_center_j,
                        rows=rows,
                        cols=cols,
                    )
                    global_obstacle_match_patch = self.extract_array_patch_by_grid_center(
                        self.obstacle_grid_match,
                        center_i=patch_center_i,
                        center_j=patch_center_j,
                        rows=rows,
                        cols=cols,
                    )
                    global_obstacle_distance_patch = self.extract_array_patch_by_grid_center(
                        self.obstacle_distance_map,
                        center_i=patch_center_i,
                        center_j=patch_center_j,
                        rows=rows,
                        cols=cols,
                    )

                    if (
                        global_slope_patch is None
                        or global_obstacle_patch is None
                        or global_obstacle_match_patch is None
                        or global_obstacle_distance_patch is None
                    ):
                        continue

                    (
                        score,
                        height_rmse,
                        slope_rmse,
                        obstacle_mismatch,
                        overlap_count,
                        global_obstacle_ratio,
                    ) = self.compute_feature_score(
                        local_features,
                        global_patch,
                        global_slope_patch,
                        global_obstacle_patch,
                        global_obstacle_match_patch,
                        global_obstacle_distance_patch,
                    )

                    if math.isfinite(score):
                        prior_dist = math.hypot(candidate_x - prior_x, candidate_y - prior_y)
                        if self.prior_weight > 0.0:
                            score += self.prior_weight * (prior_dist / self.prior_sigma_m)
                        candidates.append(
                            (
                                score,
                                candidate_i,
                                candidate_j,
                                height_rmse,
                                slope_rmse,
                                obstacle_mismatch,
                                overlap_count,
                                prior_dist,
                                global_obstacle_ratio,
                                local_obstacle_ratio,
                                yaw_delta,
                            )
                        )

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        (
            best_score,
            best_i,
            best_j,
            best_height_rmse,
            best_slope_rmse,
            best_obstacle_mismatch,
            best_overlap_count,
            best_prior_dist,
            best_global_obstacle_ratio,
            best_local_obstacle_ratio,
            best_yaw_delta,
        ) = candidates[0]

        best_x, best_y = self.grid_to_world(best_i, best_j)
        # candidate 가 heightmap 경계 밖을 가리키는 경우 clip — 그렇지 않으면
        # IndexError 로 trn_node 가 죽어 절대 위치 보정이 완전히 정지한다
        # (2026-05-26 시연 회귀). best_j 가 axis 0, best_i 가 axis 1.
        hm_h, hm_w = self.heightmap.shape
        best_j_clip = max(0, min(int(best_j), hm_h - 1))
        best_i_clip = max(0, min(int(best_i), hm_w - 1))
        best_z = float(self.heightmap[best_j_clip, best_i_clip])

        separation_cells = max(
            1,
            int(round(self.distinct_candidate_separation_m / self.resolution)),
        )
        second_rmse = float("inf")
        for item in candidates[1:]:
            rmse, candidate_i, candidate_j = item[:3]
            if math.hypot(candidate_i - best_i, candidate_j - best_j) >= separation_cells:
                second_rmse = rmse
                break

        ambiguity_margin = (
            second_rmse - best_score if math.isfinite(second_rmse) else float("inf")
        )
        confidence = self.rmse_to_confidence(best_score)
        if math.isfinite(ambiguity_margin):
            ambiguity_confidence = max(
                0.0,
                min(1.0, ambiguity_margin / self.ambiguity_margin_m),
            )
            confidence *= ambiguity_confidence

        return (
            best_x,
            best_y,
            best_z,
            best_score,
            confidence,
            second_rmse,
            ambiguity_margin,
            best_height_rmse,
            best_slope_rmse,
            best_obstacle_mismatch,
            best_overlap_count,
            best_prior_dist,
            best_local_obstacle_ratio,
            best_global_obstacle_ratio,
            best_yaw_delta,
        )

    def patch_center_world(
        self,
        rover_x: float,
        rover_y: float,
        yaw: float,
    ) -> Tuple[float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            rover_x
            + cos_yaw * self.patch_center_x
            - sin_yaw * self.patch_center_y,
            rover_y
            + sin_yaw * self.patch_center_x
            + cos_yaw * self.patch_center_y,
        )

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

        patch[~np.isfinite(patch)] = np.nan

        return patch

    def prepare_patch(self, patch: np.ndarray) -> np.ndarray:
        prepared = patch.astype(np.float32).copy()

        if self.normalize_patch_height:
            finite = np.isfinite(prepared)
            if np.any(finite):
                prepared[finite] -= float(np.mean(prepared[finite]))

        return prepared

    def prepare_local_features(
        self,
        patch: np.ndarray,
        obstacle_patch: Optional[np.ndarray] = None,
    ) -> Dict[str, np.ndarray]:
        slope = self.compute_slope_deg(patch, self.resolution)
        if obstacle_patch is None:
            obstacle = self.compute_local_obstacle_mask(patch, slope)
        else:
            obstacle = obstacle_patch.astype(np.float32).copy()
            obstacle[~np.isfinite(obstacle)] = np.nan
            finite = np.isfinite(obstacle)
            obstacle[finite] = np.where(obstacle[finite] > 0.5, 1.0, 0.0)
        return {
            "raw_height": patch.astype(np.float32).copy(),
            "height": self.prepare_patch(patch),
            "slope": slope,
            "obstacle": obstacle,
        }

    def prepare_yaw_variants(
        self,
        patch: np.ndarray,
        obstacle_patch: Optional[np.ndarray] = None,
    ):
        variants = []
        for yaw_delta in self.yaw_offsets:
            if abs(yaw_delta) < 1.0e-9:
                rotated_patch = patch.astype(np.float32).copy()
                rotated_obstacle = (
                    None if obstacle_patch is None
                    else obstacle_patch.astype(np.float32).copy()
                )
            else:
                rotated_patch = self.rotate_patch_nearest(patch, yaw_delta)
                rotated_obstacle = (
                    None if obstacle_patch is None
                    else self.rotate_patch_nearest(obstacle_patch, yaw_delta)
                )

            features = self.prepare_local_features(rotated_patch, rotated_obstacle)
            local_obstacle_ratio = self.compute_mask_ratio(features["obstacle"])
            variants.append((yaw_delta, features, local_obstacle_ratio))
        return variants

    def compute_feature_score(
        self,
        local_features: Dict[str, np.ndarray],
        global_height_patch: np.ndarray,
        global_slope_patch: np.ndarray,
        global_obstacle_patch: np.ndarray,
        global_obstacle_match_patch: np.ndarray,
        global_obstacle_distance_patch: np.ndarray,
    ) -> Tuple[float, float, float, float, int, float]:
        local_obstacle = local_features["obstacle"]
        global_obstacle = global_obstacle_patch
        global_obstacle_match = global_obstacle_match_patch
        elevation_valid = (
            np.isfinite(local_obstacle)
            & np.isfinite(global_obstacle_match)
            & (local_obstacle <= 0.5)
            & (global_obstacle_match <= 0.5)
        )

        local_height = local_features["raw_height"].copy()
        global_height = global_height_patch.astype(np.float32).copy()
        local_height[~elevation_valid] = np.nan
        global_height[~elevation_valid] = np.nan

        height_rmse, height_overlap = self.compute_rmse_and_count(
            self.prepare_patch(local_height),
            self.prepare_patch(global_height),
        )
        if height_overlap < self.min_feature_overlap_cells:
            return (
                float("inf"),
                height_rmse,
                float("inf"),
                1.0,
                height_overlap,
                self.compute_mask_ratio(global_obstacle_patch),
            )

        local_slope = local_features["slope"].copy()
        global_slope = global_slope_patch.astype(np.float32).copy()
        local_slope[~elevation_valid] = np.nan
        global_slope[~elevation_valid] = np.nan

        slope_rmse, _ = self.compute_rmse_and_count(
            local_slope,
            global_slope,
        )
        obstacle_mismatch = self.compute_obstacle_distance_score(
            local_obstacle,
            global_obstacle_distance_patch,
        )
        global_obstacle_ratio = self.compute_mask_ratio(global_obstacle_match_patch)

        slope_score = 0.0
        if math.isfinite(slope_rmse):
            slope_score = slope_rmse / self.slope_sigma_deg

        obstacle_score = 0.0
        if math.isfinite(obstacle_mismatch):
            obstacle_score = obstacle_mismatch

        score = (
            self.height_weight * height_rmse
            + self.slope_weight * slope_score
            + self.obstacle_weight * obstacle_score
        )
        return (
            float(score),
            float(height_rmse),
            float(slope_rmse),
            float(obstacle_mismatch),
            int(height_overlap),
            float(global_obstacle_ratio),
        )

    @staticmethod
    def compute_rmse(a: np.ndarray, b: np.ndarray) -> float:
        valid = np.isfinite(a) & np.isfinite(b)
        if not np.any(valid):
            return float("inf")
        diff = a[valid] - b[valid]
        return float(np.sqrt(np.mean(diff * diff)))

    @staticmethod
    def compute_rmse_and_count(a: np.ndarray, b: np.ndarray) -> Tuple[float, int]:
        valid = np.isfinite(a) & np.isfinite(b)
        count = int(np.count_nonzero(valid))
        if count == 0:
            return float("inf"), 0
        diff = a[valid] - b[valid]
        return float(np.sqrt(np.mean(diff * diff))), count

    @staticmethod
    def compute_slope_deg(patch: np.ndarray, resolution: float) -> np.ndarray:
        arr = patch.astype(np.float32)
        slope = np.full_like(arr, np.nan, dtype=np.float32)
        if arr.size == 0:
            return slope
        dz_dy, dz_dx = np.gradient(arr, resolution, resolution)
        raw = np.degrees(np.arctan(np.sqrt(dz_dx * dz_dx + dz_dy * dz_dy)))
        finite = np.isfinite(arr) & np.isfinite(raw)
        slope[finite] = raw[finite].astype(np.float32)
        return slope

    def compute_local_obstacle_mask(
        self,
        height_patch: np.ndarray,
        slope_patch: np.ndarray,
    ) -> np.ndarray:
        obstacle = np.full_like(height_patch, np.nan, dtype=np.float32)
        finite = np.isfinite(height_patch)
        if not np.any(finite):
            return obstacle

        base_height = float(np.nanpercentile(height_patch[finite], 35.0))
        high = height_patch > (base_height + self.local_obstacle_height_delta_m)
        steep = slope_patch > self.local_obstacle_slope_deg
        obstacle[finite] = np.where(high[finite] | steep[finite], 1.0, 0.0)
        return obstacle

    @staticmethod
    def compute_obstacle_mismatch(local_obstacle: np.ndarray, global_obstacle: np.ndarray) -> float:
        valid = np.isfinite(local_obstacle) & np.isfinite(global_obstacle)
        if not np.any(valid):
            return float("inf")
        local = local_obstacle[valid] > 0.5
        global_mask = global_obstacle[valid] > 0.5

        local_count = int(np.count_nonzero(local))
        if local_count == 0:
            return 0.0

        matched_local = int(np.count_nonzero(local & global_mask))
        return float(1.0 - matched_local / local_count)

    def compute_obstacle_distance_score(
        self,
        local_obstacle: np.ndarray,
        global_obstacle_distance: np.ndarray,
    ) -> float:
        valid = np.isfinite(local_obstacle) & np.isfinite(global_obstacle_distance)
        if not np.any(valid):
            return float("inf")

        local = valid & (local_obstacle > 0.5)
        local_count = int(np.count_nonzero(local))
        if local_count == 0:
            return 0.0

        distances = np.minimum(
            global_obstacle_distance[local],
            self.obstacle_distance_max_m,
        )
        return float(np.mean(distances) / self.obstacle_distance_sigma_m)

    @staticmethod
    def compute_mask_ratio(mask: np.ndarray) -> float:
        valid = np.isfinite(mask)
        if not np.any(valid):
            return 0.0
        return float(np.count_nonzero(mask[valid] > 0.5) / np.count_nonzero(valid))

    @staticmethod
    def dilate_binary_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
        source = mask > 0.5
        if radius_cells <= 0 or not np.any(source):
            return source.astype(np.float32)

        padded = np.pad(source, radius_cells, mode="constant", constant_values=False)
        dilated = np.zeros_like(source, dtype=bool)
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                y0 = radius_cells + dy
                x0 = radius_cells + dx
                dilated |= padded[
                    y0:y0 + source.shape[0],
                    x0:x0 + source.shape[1],
                ]
        return dilated.astype(np.float32)

    @staticmethod
    def compute_obstacle_distance_map(
        obstacle_grid: np.ndarray,
        resolution: float,
    ) -> np.ndarray:
        obstacle = obstacle_grid > 0.5
        if not np.any(obstacle):
            return np.full_like(obstacle_grid, np.inf, dtype=np.float32)

        try:
            from scipy import ndimage  # type: ignore

            distance = ndimage.distance_transform_edt(~obstacle) * resolution
            return distance.astype(np.float32)
        except Exception:
            pass

        distance = np.where(obstacle, 0.0, np.inf).astype(np.float32)
        diagonal = math.sqrt(2.0)
        height, width = distance.shape

        for y in range(height):
            for x in range(width):
                best = distance[y, x]
                if y > 0:
                    best = min(best, distance[y - 1, x] + 1.0)
                    if x > 0:
                        best = min(best, distance[y - 1, x - 1] + diagonal)
                    if x + 1 < width:
                        best = min(best, distance[y - 1, x + 1] + diagonal)
                if x > 0:
                    best = min(best, distance[y, x - 1] + 1.0)
                distance[y, x] = best

        for y in range(height - 1, -1, -1):
            for x in range(width - 1, -1, -1):
                best = distance[y, x]
                if y + 1 < height:
                    best = min(best, distance[y + 1, x] + 1.0)
                    if x > 0:
                        best = min(best, distance[y + 1, x - 1] + diagonal)
                    if x + 1 < width:
                        best = min(best, distance[y + 1, x + 1] + diagonal)
                if x + 1 < width:
                    best = min(best, distance[y, x + 1] + 1.0)
                distance[y, x] = best

        return distance * float(resolution)

    @staticmethod
    def build_yaw_offsets(max_deg: float, step_deg: float):
        if max_deg <= 0.0:
            return [0.0]

        step_deg = max(step_deg, 1.0e-6)
        count = max(1, int(math.floor(max_deg / step_deg)))
        offsets = {0.0}
        for index in range(1, count + 1):
            value = index * step_deg
            if value <= max_deg + 1.0e-6:
                offsets.add(value)
                offsets.add(-value)
        offsets.add(max_deg)
        offsets.add(-max_deg)

        return [math.radians(value) for value in sorted(offsets, key=lambda v: (abs(v), v))]

    @staticmethod
    def rotate_patch_nearest(patch: np.ndarray, yaw_delta: float) -> np.ndarray:
        source = patch.astype(np.float32)
        rows, cols = source.shape
        output = np.full((rows, cols), np.nan, dtype=np.float32)
        if rows == 0 or cols == 0:
            return output

        cos_yaw = math.cos(yaw_delta)
        sin_yaw = math.sin(yaw_delta)
        center_y = (rows - 1) * 0.5
        center_x = (cols - 1) * 0.5

        out_y, out_x = np.indices((rows, cols), dtype=np.float32)
        x = out_x - center_x
        y = out_y - center_y

        src_x = cos_yaw * x + sin_yaw * y + center_x
        src_y = -sin_yaw * x + cos_yaw * y + center_y
        src_i = np.rint(src_x).astype(np.int32)
        src_j = np.rint(src_y).astype(np.int32)

        valid = (
            (src_i >= 0)
            & (src_i < cols)
            & (src_j >= 0)
            & (src_j < rows)
        )
        output[valid] = source[src_j[valid], src_i[valid]]
        return output

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
        return self.extract_array_patch_by_grid_center(
            self.heightmap,
            center_i=center_i,
            center_j=center_j,
            rows=rows,
            cols=cols,
        )

    def extract_array_patch_by_grid_center(
        self,
        source: np.ndarray,
        center_i: int,
        center_j: int,
        rows: int,
        cols: int,
    ) -> Optional[np.ndarray]:
        half_rows = rows // 2
        half_cols = cols // 2

        row_start = center_j - half_rows
        row_end = row_start + rows

        col_start = center_i - half_cols
        col_end = col_start + cols

        src_row_start = max(0, row_start)
        src_row_end = min(source.shape[0], row_end)
        src_col_start = max(0, col_start)
        src_col_end = min(source.shape[1], col_end)

        if src_row_start >= src_row_end or src_col_start >= src_col_end:
            return None

        patch = np.full((rows, cols), np.nan, dtype=np.float32)

        dst_row_start = src_row_start - row_start
        dst_row_end = dst_row_start + (src_row_end - src_row_start)
        dst_col_start = src_col_start - col_start
        dst_col_end = dst_col_start + (src_col_end - src_col_start)

        patch[dst_row_start:dst_row_end, dst_col_start:dst_col_end] = source[
            src_row_start:src_row_end,
            src_col_start:src_col_end,
        ]
        return patch

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
        isaac_sim/assets/generated_terrains/terrain_00001/heightmap.npy
        isaac_sim/assets/generated_terrains/terrain_00001/meta.json
        """
        default_terrain_dir = self.resolve_default_terrain_dir()

        if not self.heightmap_path:
            heightmap_path = default_terrain_dir / "heightmap.npy"
        else:
            heightmap_path = Path(self.heightmap_path)

        if not self.metadata_path:
            metadata_path = default_terrain_dir / "meta.json"
        else:
            metadata_path = Path(self.metadata_path)

        if not heightmap_path.exists():
            raise FileNotFoundError(f"Heightmap file not found: {heightmap_path}")

        if not metadata_path.exists():
            raise FileNotFoundError(f"Metadata file not found: {metadata_path}")

        heightmap = np.load(heightmap_path).astype(np.float32)
        metadata = self.load_metadata(metadata_path, heightmap)

        required = ["resolution", "origin_x", "origin_y", "width", "height"]
        for key in required:
            if key not in metadata:
                raise KeyError(f"Missing key '{key}' in {metadata_path}")

        self.loaded_terrain_dir = metadata_path.parent
        self.loaded_heightmap_path = heightmap_path
        self.loaded_metadata_path = metadata_path

        return heightmap, metadata

    def load_obstacle_grid(self) -> np.ndarray:
        if self.obstacle_grid_path:
            obstacle_path = Path(self.obstacle_grid_path)
        else:
            obstacle_path = getattr(
                self,
                "loaded_terrain_dir",
                self.resolve_default_terrain_dir(),
            ) / "obstacle_grid.npy"

        if not obstacle_path.exists():
            self.get_logger().warn(
                f"Obstacle grid not found: {obstacle_path}. "
                "TRN obstacle feature disabled with an all-free mask."
            )
            return np.zeros_like(self.heightmap, dtype=np.float32)

        grid = np.load(obstacle_path)
        return (grid > 0).astype(np.float32)

    def resolve_default_terrain_dir(self) -> Path:
        if self.terrain_root:
            terrain_root = Path(self.terrain_root)
        else:
            a2_root = _resolve_a2_root(__file__)
            terrain_root = (
                a2_root / "isaac_sim" / "assets" / "generated_terrains"
            )

        terrain_id = str(self.terrain_id)
        if terrain_id == "latest":
            terrain_id = self.resolve_latest_terrain_id(terrain_root)

        return terrain_root / terrain_id

    @staticmethod
    def resolve_latest_terrain_id(terrain_root: Path) -> str:
        index_path = terrain_root / "index.json"
        if index_path.exists():
            index = json.loads(index_path.read_text(encoding="utf-8"))
            terrains = index.get("terrains", [])
            if terrains:
                return str(terrains[-1]["id"])

        terrain_dirs = sorted(
            p.name for p in terrain_root.glob("terrain_*") if p.is_dir()
        )
        if not terrain_dirs:
            raise FileNotFoundError(f"No terrain_* directories found in {terrain_root}")
        return terrain_dirs[-1]

    def load_metadata(self, path: Path, heightmap: np.ndarray) -> Dict[str, str]:
        if path.suffix.lower() == ".json":
            return self.parse_meta_json(path, heightmap)
        return self.parse_simple_yaml(path)

    @staticmethod
    def parse_meta_json(path: Path, heightmap: np.ndarray) -> Dict[str, str]:
        meta = json.loads(path.read_text(encoding="utf-8"))
        origin = meta.get("origin", {})
        height, width = heightmap.shape

        return {
            "resolution": str(meta.get("resolution_m", 0.05)),
            "origin_x": str(origin.get("x", 0.0)),
            "origin_y": str(origin.get("y", 0.0)),
            "width": str(meta.get("width", width)),
            "height": str(meta.get("height", height)),
        }

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
