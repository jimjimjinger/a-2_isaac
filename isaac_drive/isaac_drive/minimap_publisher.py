"""minimap_publisher — coverage 미니맵 상태를 표준 ROS2 메시지로 발행.

viewer.py(matplotlib) 는 /tmp 의 npz 파일을 폴링하는 디버그 전용 경로다.
이 모듈은 같은 상태를 표준 메시지로 토픽에 실어, RViz·Foxglove·T4 mission UI
어디서든 구독만으로 미니맵을 그릴 수 있게 한다 — 소비자 쪽 렌더링 코드 0.

  /mission/minimap  nav_msgs/OccupancyGrid          fog+obstacle 합성 격자
  /mission/path     nav_msgs/Path                   현재 A* 경로
  /mission/markers  visualization_msgs/MarkerArray  섹터 격자·anchor 후보·목표

로버 pose 는 /rover/estimated_pose 로 이미 발행되므로 여기서 다시 싣지 않는다
— UI 는 그 토픽을 함께 구독하면 된다.

좌표계: 모든 메시지의 frame_id 는 동일 월드 프레임. coverage 는 obstacle_grid
를 절대 월드좌표로 인덱싱하고 브리지도 절대 pose 를 발행하므로 map=odom=world.
"""
from __future__ import annotations

from array import array

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray

# OccupancyGrid 셀 값 (nav_msgs 규약: -1 unknown, 0 free, 100 occupied)
_UNKNOWN, _FREE, _OCCUPIED = -1, 0, 100

# 섹터 라벨 색 — 현재 구역 / 완료 구역 / 미방문 구역 (viewer.py 와 동일 규약)
_SECTOR_DONE_RATIO = 0.95


class MinimapPublisher:
    """coverage 미니맵 상태를 표준 ROS2 메시지로 발행하는 헬퍼."""

    def __init__(self, node, fog, frame_id: str = "map",
                 publish_every: int = 10) -> None:
        """
        Args:
            node:          소유 rclpy Node (publisher·clock 제공).
            fog:           FogMap — 격자·섹터 기하 제공.
            frame_id:      모든 메시지의 좌표 프레임.
            publish_every: N tick 마다 1회 발행 (큰 OccupancyGrid 대역폭 절약).
        """
        self.node = node
        self.fog = fog
        self.frame_id = frame_id
        self.publish_every = max(1, int(publish_every))

        # Relative topic name (no leading /) → namespace 자동 prefix.
        # 단일: /mission/minimap, 다중: /rover_X/mission/minimap.
        self.grid_pub = node.create_publisher(OccupancyGrid, "mission/minimap", 1)
        self.path_pub = node.create_publisher(Path, "mission/path", 1)
        self.marker_pub = node.create_publisher(MarkerArray, "mission/markers", 1)

        # OccupancyGrid 의 정적 메타(해상도·크기·origin)는 1회만 계산.
        self._grid_info = self._build_grid_info()

    # ── tick 진입점 ──────────────────────────────────────
    def maybe_publish(self, tick: int, mission) -> None:
        """publish_every tick 마다 미니맵 3종 토픽을 발행."""
        if tick % self.publish_every != 0:
            return
        stamp = self.node.get_clock().now().to_msg()
        self.grid_pub.publish(self._grid_msg(stamp))
        self.path_pub.publish(self._path_msg(stamp, mission))
        self.marker_pub.publish(self._marker_msg(stamp, mission))

    # ── /mission/minimap — OccupancyGrid ────────────────
    def _build_grid_info(self) -> MapMetaData:
        info = MapMetaData()
        info.resolution = float(self.fog.cell_size)
        info.width = int(self.fog.cols)
        info.height = int(self.fog.rows)
        # origin = 격자 (0,0) 셀의 월드 좌표 = 맵 좌하단.
        info.origin.position.x = -self.fog.map_w / 2.0
        info.origin.position.y = -self.fog.map_h / 2.0
        info.origin.orientation.w = 1.0
        return info

    def _grid_msg(self, stamp) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.info = self._grid_info
        # fog(0/1) + obstacle_mask(0/1) → 점유 격자.
        #   미밝힘 = unknown(-1), 밝힘 = free(0), 장애물 = occupied(100, 항상).
        cells = np.full(self.fog.fog.shape, _UNKNOWN, dtype=np.int8)
        cells[self.fog.fog == 1] = _FREE
        cells[self.fog.obstacle_mask == 1] = _OCCUPIED
        # OccupancyGrid.data 는 (0,0)=좌하단 row-major — fog 격자 규약과 동일.
        msg.data = array("b", cells.tobytes())
        return msg

    # ── /mission/path — Path ────────────────────────────
    def _path_msg(self, stamp, mission) -> Path:
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        for x, y in (mission.nav.path or []):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    # ── /mission/markers — MarkerArray ──────────────────
    def _marker_msg(self, stamp, mission) -> MarkerArray:
        arr = MarkerArray()
        arr.markers.append(self._sector_grid_marker(stamp))
        arr.markers.append(self._anchors_marker(stamp, mission))
        arr.markers.append(self._target_marker(stamp, mission))
        arr.markers.extend(self._sector_label_markers(stamp, mission))
        return arr

    def _base_marker(self, stamp, ns: str, mid: int, mtype: int) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _sector_grid_marker(self, stamp) -> Marker:
        """N×N 섹터 내부 경계선 (LINE_LIST)."""
        m = self._base_marker(stamp, "sector_grid", 0, Marker.LINE_LIST)
        m.scale.x = 0.08          # 선 두께 (m)
        m.color.r = m.color.g = m.color.b = 0.55
        m.color.a = 0.7
        half_w, half_h = self.fog.map_w / 2.0, self.fog.map_h / 2.0
        for k in range(1, self.fog.grid_n):
            x = -half_w + k * self.fog.sector_w
            m.points.append(Point(x=x, y=-half_h, z=0.0))
            m.points.append(Point(x=x, y=half_h, z=0.0))
            y = -half_h + k * self.fog.sector_h
            m.points.append(Point(x=-half_w, y=y, z=0.0))
            m.points.append(Point(x=half_w, y=y, z=0.0))
        return m

    def _anchors_marker(self, stamp, mission) -> Marker:
        """남은 anchor 후보 (SPHERE_LIST). 비었으면 점 0개 → 표시 없음."""
        m = self._base_marker(stamp, "anchors", 0, Marker.SPHERE_LIST)
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.85, 1.0, 0.6  # cyan
        for x, y in (mission.anchor_queue or []):
            m.points.append(Point(x=float(x), y=float(y), z=0.0))
        return m

    def _target_marker(self, stamp, mission) -> Marker:
        """현재 목표 anchor (SPHERE). 목표 없으면 DELETE 로 지움."""
        m = self._base_marker(stamp, "target", 0, Marker.SPHERE)
        target = mission.nav.current_target
        if target is None:
            m.action = Marker.DELETE
            return m
        m.scale.x = m.scale.y = m.scale.z = 0.9
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.9  # magenta
        m.pose.position.x = float(target[0])
        m.pose.position.y = float(target[1])
        return m

    def _sector_label_markers(self, stamp, mission) -> list:
        """섹터 번호 (TEXT). 현재=노랑 / 완료=초록 / 미방문=흰 (viewer.py 규약)."""
        markers = []
        n = self.fog.grid_n
        half_w, half_h = self.fog.map_w / 2.0, self.fog.map_h / 2.0
        for s in range(n * n):
            row, col = s // n, s % n
            m = self._base_marker(stamp, "sector_label", s,
                                   Marker.TEXT_VIEW_FACING)
            m.pose.position.x = -half_w + (col + 0.5) * self.fog.sector_w
            m.pose.position.y = -half_h + (row + 0.5) * self.fog.sector_h
            m.scale.z = 1.6        # 글자 높이 (m)
            m.text = str(s + 1)
            m.color.a = 0.9
            if s == mission.current_sector:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            elif self.fog.sector_revealed_ratio(s) >= _SECTOR_DONE_RATIO:
                m.color.r, m.color.g, m.color.b = 0.3, 1.0, 0.3
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 1.0
            markers.append(m)
        return markers
