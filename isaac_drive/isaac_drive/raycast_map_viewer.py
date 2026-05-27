"""raycast_map_viewer — matplotlib 윈도우로 raycast 누적 맵 시각화.

/rover/raycast/built_grid (Phase 2 raycast 누적 "내가 그린 맵") 을 RViz
TopDownOrtho(Angle=-π/2) 와 동일한 방향으로 표시.

토픽 subscribe:
  /rover/raycast/built_grid (nav_msgs/OccupancyGrid)  — 메인 캔버스
  /map (nav_msgs/OccupancyGrid)                       — 원본 비교용 (좌표만)
  /rover/estimated_pose (PoseWithCovarianceStamped)   — rover 위치 (EKF, 파란 삼각형)
  /ground_truth/odom (nav_msgs/Odometry)              — rover 실제 절대좌표 (청록 X)

토픽 publish:
  /rover/raycast/corrected_marker (Marker)            — RViz 노란 화살표
"""
from __future__ import annotations

import math
import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RaycastMapViewer(Node):
    def __init__(self) -> None:
        super().__init__("raycast_map_viewer")

        self.declare_parameter("grid_topic", "/rover/raycast/built_grid")
        self.declare_parameter("orig_grid_topic", "/map")
        self.declare_parameter("pose_topic", "/rover/estimated_pose")
        self.declare_parameter("gt_topic", "/ground_truth/odom")
        self.declare_parameter("trail_max_len", 600)
        self.declare_parameter("view_angle_deg", 90.0)
        self.declare_parameter("nearby_radius_m", 4.0)
        self.declare_parameter("nearby_radius_orig_m", 4.0)
        self.declare_parameter("nearby_top_n", 3)
        self.declare_parameter("nearby_dedup_m", 0.8)
        self.declare_parameter("max_match_dist_m", 2.0)
        self.declare_parameter("max_correction_mag_m", 3.0)

        grid_topic      = str(self.get_parameter("grid_topic").value)
        orig_grid_topic = str(self.get_parameter("orig_grid_topic").value)
        pose_topic      = str(self.get_parameter("pose_topic").value)
        gt_topic        = str(self.get_parameter("gt_topic").value)
        self.trail_max_len       = int(self.get_parameter("trail_max_len").value)
        self.view_angle_deg      = float(self.get_parameter("view_angle_deg").value)
        self.nearby_radius_m     = float(self.get_parameter("nearby_radius_m").value)
        self.nearby_radius_orig_m = float(self.get_parameter("nearby_radius_orig_m").value)
        self.nearby_top_n        = int(self.get_parameter("nearby_top_n").value)
        self.nearby_dedup_m      = float(self.get_parameter("nearby_dedup_m").value)
        self.max_match_dist_m    = float(self.get_parameter("max_match_dist_m").value)
        self.max_correction_mag_m = float(self.get_parameter("max_correction_mag_m").value)

        self.create_subscription(OccupancyGrid, grid_topic, self._on_grid, 5)
        self.create_subscription(OccupancyGrid, orig_grid_topic, self._on_orig, 1)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic,
                                 self._on_pose, 10)
        self.create_subscription(Odometry, gt_topic, self._on_gt_odom, 10)

        self.lock      = threading.Lock()
        self.grid:      Optional[OccupancyGrid] = None
        self.orig_grid: Optional[OccupancyGrid] = None
        self.rover_xy:  Optional[tuple[float, float]] = None
        self.rover_yaw: float = 0.0
        self.gt_xy:     Optional[tuple[float, float]] = None
        self.trail:     list[tuple[float, float]] = []

        self.pub_corrected = self.create_publisher(
            Marker, "/rover/raycast/corrected_marker", 1)

        self.get_logger().info(
            f"raycast_map_viewer 구독 — grid={grid_topic} pose={pose_topic} "
            f"view_angle={self.view_angle_deg:.1f}°")

    def publish_corrected_marker(self, x: float, y: float, yaw: float,
                                 frame_id: str = "map") -> None:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "raycast_corrected"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        length = 0.8
        m.pose.position.x = float(x) - (length / 2.0) * math.cos(yaw)
        m.pose.position.y = float(y) - (length / 2.0) * math.sin(yaw)
        m.pose.position.z = 0.6
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x = length
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 1.0
        m.color.g = 0.82
        m.color.b = 0.29
        m.color.a = 0.9
        self.pub_corrected.publish(m)

    def _on_grid(self, msg: OccupancyGrid) -> None:
        with self.lock:
            self.grid = msg

    def _on_orig(self, msg: OccupancyGrid) -> None:
        with self.lock:
            self.orig_grid = msg

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        y = _quat_to_yaw(msg.pose.pose.orientation)
        self._set_rover(p.x, p.y, y)

    def _on_gt_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self.lock:
            self.gt_xy = (p.x, p.y)

    def _set_rover(self, x: float, y: float, yaw: float) -> None:
        with self.lock:
            self.rover_xy  = (x, y)
            self.rover_yaw = yaw
            self.trail.append((x, y))
            if len(self.trail) > self.trail_max_len:
                self.trail = self.trail[-self.trail_max_len:]


def _grid_to_rgb(g: OccupancyGrid) -> tuple[np.ndarray, tuple]:
    w, h = g.info.width, g.info.height
    res  = g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    arr  = np.array(g.data, dtype=np.int16).reshape(h, w)

    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[arr == -1] = (13, 17, 23)
    free_mask = (arr >= 0) & (arr < 50)
    rgb[free_mask] = (40, 50, 65)
    obs_mask = arr >= 50
    if obs_mask.any():
        t = ((arr[obs_mask].astype(np.float32) - 50.0) / 50.0).clip(0.0, 1.0)
        r  = (180 + t * 75).astype(np.uint8)
        gch = (40 + t * 30).astype(np.uint8)
        b  = (50 + t * 30).astype(np.uint8)
        rgb[obs_mask] = np.stack([r, gch, b], axis=-1)
    extent = (ox, ox + w * res, oy, oy + h * res)
    return rgb, extent


def _nearby_obstacles(g: OccupancyGrid, center_xy: tuple,
                      radius_m: float, top_n: int,
                      dedup_m: float, obs_thresh: int = 50) -> list:
    if g is None or center_xy is None:
        return []
    w, h = g.info.width, g.info.height
    res  = g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    arr  = np.array(g.data, dtype=np.int16).reshape(h, w)
    obs_mask = arr >= obs_thresh
    if not obs_mask.any():
        return []
    iy, ix = np.where(obs_mask)
    xs = ox + (ix.astype(np.float32) + 0.5) * res
    ys = oy + (iy.astype(np.float32) + 0.5) * res
    cx, cy = center_xy
    dist = np.hypot(xs - cx, ys - cy)
    within = dist <= radius_m
    if not within.any():
        return []
    xs, ys, dist = xs[within], ys[within], dist[within]
    order = np.argsort(dist)
    out: list = []
    for i in order:
        x, y, d = float(xs[i]), float(ys[i]), float(dist[i])
        if any(math.hypot(x - ox2, y - oy2) < dedup_m for ox2, oy2, _ in out):
            continue
        out.append((x, y, d))
        if len(out) >= top_n:
            break
    return out


def _pattern_match_correction(orig_list: list, ray_list: list,
                              match_thresh: float,
                              max_correction_mag: float,
                              min_matches: int = 2) -> tuple:
    """장애물 패턴(군집) 매칭 기반 위치 보정.

    알고리즘:
      1. 모든 (ray_i, map_j) 쌍을 앵커 후보 shift 로 시도
      2. 각 shift 에서 나머지 raycast 장애물들이 몇 개나 지도 장애물과
         match_thresh 이내로 일치하는지 채점(score)
      3. score 가 최대인 shift 를 선택, 일치 쌍들의 offset 을 평균해 반환
    """
    if not orig_list or not ray_list:
        return None, None, 0, "no obstacles"

    best_score = 0
    best_dx = 0.0
    best_dy = 0.0

    for (rx, ry, _) in ray_list:
        for (mx, my, _) in orig_list:
            dx_c = mx - rx
            dy_c = my - ry
            if math.hypot(dx_c, dy_c) > max_correction_mag:
                continue

            # 이 shift 에서 각 raycast 장애물의 최근접 지도 장애물 탐색
            match_dxs: list = []
            match_dys: list = []
            used: set = set()
            for (rx2, ry2, _) in ray_list:
                sx, sy = rx2 + dx_c, ry2 + dy_c
                best_d = match_thresh
                best_k = -1
                for k, (mx2, my2, _) in enumerate(orig_list):
                    if k in used:
                        continue
                    d = math.hypot(sx - mx2, sy - my2)
                    if d < best_d:
                        best_d = d
                        best_k = k
                if best_k >= 0:
                    used.add(best_k)
                    mx2, my2, _ = orig_list[best_k]
                    match_dxs.append(mx2 - rx2)
                    match_dys.append(my2 - ry2)

            score = len(match_dxs)
            if score > best_score:
                best_score = score
                best_dx = sum(match_dxs) / score
                best_dy = sum(match_dys) / score

    if best_score < min_matches:
        return None, None, best_score, f"패턴 매칭 실패 (score={best_score}/{min_matches})"

    mag = math.hypot(best_dx, best_dy)
    if mag > max_correction_mag:
        return None, None, best_score, f"|Δ|={mag:.2f}>{max_correction_mag:.1f}"

    return best_dx, best_dy, best_score, "ok"


def _rotated_xlim_ylim(extent: tuple, angle_deg: float) -> tuple:
    rad = math.radians(angle_deg)
    c, s = math.cos(rad), math.sin(rad)
    xs0, xs1, ys0, ys1 = extent
    corners = [(xs0, ys0), (xs1, ys0), (xs1, ys1), (xs0, ys1)]
    rotated = [(c*x - s*y, s*x + c*y) for x, y in corners]
    xs = [p[0] for p in rotated]
    ys = [p[1] for p in rotated]
    return (min(xs), max(xs)), (min(ys), max(ys))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaycastMapViewer()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.transforms import Affine2D

    fig, ax = plt.subplots(figsize=(9, 9), facecolor="#0d1117")
    fig.canvas.manager.set_window_title("Raycast Map")

    angle_deg = node.view_angle_deg
    rot = Affine2D().rotate_deg(angle_deg) + ax.transData

    ax.set_facecolor("#0d1117")
    ax.set_title("RAYCAST MAP — /rover/raycast/built_grid",
                 color="#9fdcff", fontsize=12, loc="left", pad=10)
    ax.set_aspect("equal")
    ax.tick_params(colors="#5a6677")
    for spine in ax.spines.values():
        spine.set_color("#2a3340")
    ax.set_xlabel("X (m)", color="#5a6677")
    ax.set_ylabel("Y (m)", color="#5a6677")

    state = {"im": None, "rover": None, "trail": None, "coord_text": None,
             "corrected": None,
             "obs_texts": []}

    def _draw_rover(rxy, ryaw, trail):
        if trail:
            tx, ty = zip(*trail)
            if state["trail"] is None:
                (state["trail"],) = ax.plot(
                    tx, ty, color="#ffd24a", linewidth=2.0, alpha=0.85,
                    transform=rot)
            else:
                state["trail"].set_data(tx, ty)
        if rxy is None:
            return
        x, y = rxy
        tri = 0.9
        verts = [
            (x + math.cos(ryaw) * tri, y + math.sin(ryaw) * tri),
            (x + math.cos(ryaw + 2.5) * tri * 0.7,
             y + math.sin(ryaw + 2.5) * tri * 0.7),
            (x + math.cos(ryaw - 2.5) * tri * 0.7,
             y + math.sin(ryaw - 2.5) * tri * 0.7),
        ]
        if state["rover"] is None:
            state["rover"] = Polygon(
                verts, closed=True, facecolor="#4fd1e1",
                edgecolor="#0d1117", linewidth=1.5, transform=rot,
                zorder=5)
            ax.add_patch(state["rover"])
        else:
            state["rover"].set_xy(verts)

    def _render():
        with node.lock:
            g        = node.grid
            orig_g   = node.orig_grid
            rxy      = node.rover_xy
            ryaw     = node.rover_yaw
            gt_xy    = node.gt_xy
            trail    = list(node.trail)
            radius      = node.nearby_radius_m
            radius_orig = node.nearby_radius_orig_m
            top_n    = node.nearby_top_n
            dedup    = node.nearby_dedup_m
            max_match = node.max_match_dist_m
            max_corr  = node.max_correction_mag_m

        if g is not None:
            rgb, ext = _grid_to_rgb(g)
            if state["im"] is None:
                state["im"] = ax.imshow(
                    rgb, extent=ext, origin="lower",
                    interpolation="nearest", transform=rot)
                (xlo, xhi), (ylo, yhi) = _rotated_xlim_ylim(ext, angle_deg)
                ax.set_xlim(xlo, xhi)
                ax.set_ylim(ylo, yhi)
            else:
                state["im"].set_data(rgb)
                state["im"].set_extent(ext)
            _draw_rover(rxy, ryaw, trail)

        # 좌상단 텍스트
        lines = ["═══ ROVER POSITION ═══"]
        if rxy is not None:
            lines.append(f"EKF   x={rxy[0]:+7.2f}  y={rxy[1]:+7.2f}  "
                         f"yaw={math.degrees(ryaw):+6.1f}°")
        else:
            lines.append("EKF   (no /rover/estimated_pose)")

        center = rxy
        lines.append("")
        lines.append(f"═══ /map obstacles (≤{radius_orig:.1f}m) ═══")
        orig_list = _nearby_obstacles(orig_g, center, radius_orig, top_n, dedup)
        if orig_list:
            for i, (x, y, d) in enumerate(orig_list, 1):
                lines.append(f"  {i}: ({x:+6.2f}, {y:+6.2f})  d={d:5.2f}m")
        else:
            lines.append("  (no obstacle in radius)")

        lines.append("")
        lines.append(f"═══ raycast obstacles (≤{radius:.1f}m, matched #) ═══")
        ray_list = _nearby_obstacles(g, center, radius, top_n, dedup)
        if ray_list:
            for (x, y, d) in ray_list:
                best_idx = None
                best_d   = float("inf")
                for o_idx, (ox_, oy_, _) in enumerate(orig_list, 1):
                    md = math.hypot(ox_ - x, oy_ - y)
                    if md < best_d:
                        best_d   = md
                        best_idx = o_idx
                label = str(best_idx) if (
                    best_idx is not None and best_d <= max_match) else "?"
                lines.append(f"  {label}: ({x:+6.2f}, {y:+6.2f})  d={d:5.2f}m")
        else:
            lines.append("  (no obstacle in radius)")

        # ── 맵 캔버스에 로버 절대좌표 X 마커 표시 ──────────────────────
        for sc in state["obs_texts"]:
            try:
                sc.remove()
            except Exception:
                pass
        state["obs_texts"].clear()

        if gt_xy is not None and g is not None:
            gx, gy = gt_xy
            sc = ax.scatter([gx], [gy], s=120, c="#4fd1e1", marker="x",
                            linewidths=2.5, transform=rot, zorder=10)
            state["obs_texts"].append(sc)

        # pose correction — 패턴 매칭 기반
        # 패턴 매칭용 지도 탐색: EKF 오차 범위(max_corr)까지 커버하기 위해
        # 표시용 radius_orig 보다 넓은 반경(radius + max_corr)으로 검색
        correction_xy = None
        n_matched = 0
        reason    = "no data"
        if ray_list and rxy is not None:
            match_search_r = radius + max_corr
            orig_list_match = _nearby_obstacles(
                orig_g, center, match_search_r, top_n * 5, dedup * 0.4)
            cx, cy, n_matched, reason = _pattern_match_correction(
                orig_list_match, ray_list, max_match, max_corr)
            if cx is not None:
                correction_xy = (cx, cy)

        lines.append("")
        lines.append("═══ POSE CORRECTION (pattern match) ═══")
        if correction_xy is not None:
            cx, cy = correction_xy
            cmag   = math.hypot(cx, cy)
            cor_x  = rxy[0] + cx
            cor_y  = rxy[1] + cy
            lines.append(f"Δ     x={cx:+6.2f}  y={cy:+6.2f}  |Δ|={cmag:5.2f}m"
                         f"  (score={n_matched})")
            lines.append(f"→ 추정위치  x={cor_x:+7.2f}  y={cor_y:+7.2f}")
        else:
            lines.append(f"({reason})")

        txt = "\n".join(lines)

        # 노란 삼각형 (corrected rover)
        if correction_xy is not None and rxy is not None:
            cx, cy = correction_xy
            ex = rxy[0] + cx
            ey = rxy[1] + cy
            tri = 0.7
            verts_c = [
                (ex + math.cos(ryaw) * tri, ey + math.sin(ryaw) * tri),
                (ex + math.cos(ryaw + 2.5) * tri * 0.7,
                 ey + math.sin(ryaw + 2.5) * tri * 0.7),
                (ex + math.cos(ryaw - 2.5) * tri * 0.7,
                 ey + math.sin(ryaw - 2.5) * tri * 0.7),
            ]
            if state["corrected"] is None:
                state["corrected"] = Polygon(
                    verts_c, closed=True, facecolor="#ffd24a",
                    edgecolor="#0d1117", linewidth=1.5, transform=rot,
                    zorder=6, alpha=0.9)
                ax.add_patch(state["corrected"])
            else:
                state["corrected"].set_xy(verts_c)
            node.publish_corrected_marker(ex, ey, ryaw)

        if state["coord_text"] is None:
            state["coord_text"] = fig.text(
                0.01, 0.98, txt,
                color="#cfe4ff", fontsize=10, family="monospace",
                verticalalignment="top",
                bbox=dict(facecolor="#0d1117", edgecolor="#2a3340",
                          boxstyle="round,pad=0.4", alpha=0.85))
        else:
            state["coord_text"].set_text(txt)

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=500)
    timer.add_callback(_render)
    timer.start()

    try:
        plt.show()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
