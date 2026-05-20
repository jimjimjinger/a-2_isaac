"""구역별 BCD 분할 + zigzag anchor 생성.

각 sector 영역을 obstacle_grid 기준으로 BCD (Boustrophedon Cellular
Decomposition) 분할하고, 작은 sub-cell 을 병합한 뒤 각 sub-cell 안에서
zigzag anchor 를 만든다. sub-cell 방문 순서는 로버 현재 위치 기준
Nearest Neighbor.

anchor 간격·병합 임계값은 reveal_radius 와 grid 해상도에서 자동 계산되어,
맵 크기가 바뀌어도 SectorPlanner 코드는 그대로 동작한다.
"""
import numpy as np

from .bcd_planner import (
    bcd_decompose,
    merge_small_cells,
    generate_zigzag_anchors,
    order_cells_nearest_neighbor,
)

# anchor 간격 = reveal_radius * 이 계수. reveal disk 가 겹치도록 1.5 < 2.0.
ANCHOR_SPACING_FACTOR = 1.5


class SectorPlanner:
    def __init__(self, fog_map, obstacle_grid, reveal_radius=2.0):
        """
        Args:
            fog_map:       FogMap 인스턴스 (sector 경계 정보).
            obstacle_grid: ObstacleGrid 인스턴스 (BCD 입력 그리드).
            reveal_radius: 센서 reveal 반경 (m). anchor 간격 산출 기준.
        """
        self.fog = fog_map
        self.ogrid = obstacle_grid
        self.reveal_radius = float(reveal_radius)
        self.spacing = float(reveal_radius) * ANCHOR_SPACING_FACTOR
        # anchor 하나도 못 넣을 만큼 작은 sub-cell 병합 임계값 (셀 수)
        cs = obstacle_grid.cell_size
        self.min_cell_cells = (self.spacing ** 2) / (cs ** 2)
        # anchor inset 거리. sub-cell 경계는 이미 robot_radius 만큼
        # inflate 돼 있으므로, reveal_radius 에서 그만큼 빼야 anchor 가
        # 실제 장애물/벽에서 reveal_radius 떨어진 위치가 된다.
        self.clearance = max(0.0, self.reveal_radius
                             - obstacle_grid.robot_radius)

    def generate_anchors(self, sector_idx, start_xy):
        """sector 를 BCD 분할 → zigzag anchor 리스트 (방문 순서대로 flat).

        Args:
            sector_idx: 구역 인덱스.
            start_xy:   (x, y) 로버 현재 위치. sub-cell NN 정렬 기준.

        Returns:
            list[(x, y)] — 방문 순서대로 이어붙인 anchor 의 world 좌표.
        """
        x_min, x_max, y_min, y_max = self.fog.sector_bounds(sector_idx)
        i0, j0 = self.ogrid.world_to_cell(x_min, y_min, clip=True)
        i1, j1 = self.ogrid.world_to_cell(x_max, y_max, clip=True)
        sub = self.ogrid.grid[i0:i1, j0:j1]
        if sub.size == 0:
            return []

        label = bcd_decompose(sub)
        label = merge_small_cells(label, self.min_cell_cells)
        n_cell = int(label.max())

        cs = self.ogrid.cell_size
        # sub-grid (0,0) 셀의 좌하단 world 좌표
        origin_x = j0 * cs - self.ogrid.map_w / 2.0
        origin_y = i0 * cs - self.ogrid.map_h / 2.0

        cells = []   # (cid, center_xy, anchors)
        for cid in range(1, n_cell + 1):
            mask = (label == cid)
            if not mask.any():
                continue
            anchors = generate_zigzag_anchors(mask, (origin_x, origin_y),
                                              cs, self.spacing,
                                              clearance_m=self.clearance)
            if not anchors:
                continue
            ys, xs = np.where(mask)
            cx = (xs.mean() + 0.5) * cs + origin_x
            cy = (ys.mean() + 0.5) * cs + origin_y
            cells.append((cid, (cx, cy), anchors))

        if not cells:
            return []

        order = order_cells_nearest_neighbor(
            [(cid, c) for cid, c, _ in cells], start_xy)

        result = []
        for cid in order:
            entry = next(e for e in cells if e[0] == cid)
            result.extend(entry[2])
        return result


def sector_visit_order(num_sectors=9):
    """방문 순서: 5→2→1→4→7→8→9→6→3 (사용자 지정, 중앙에서 나선형).

    1-indexed:  5  2  1  4  7  8  9  6  3
    0-indexed:  4  1  0  3  6  7  8  5  2
    """
    if num_sectors == 9:
        return [4, 1, 0, 3, 6, 7, 8, 5, 2]
        # return [1]
    # 폴백: 기본 순서
    return list(range(num_sectors))
