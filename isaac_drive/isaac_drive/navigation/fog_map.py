"""안개 그리드 + reveal 로직.

설계:
  - 맵을 작은 격자로 나눠 0/1 상태 관리 (안개/밝힘)
  - 로봇 위치 주위 REVEAL_RADIUS 안의 셀을 밝힘
  - 9구역 (3×3) 으로 분할. 각 구역의 모든 셀이 밝혀지면 그 구역 "완료"

좌표:
  - 월드 (x, y): -map_size/2 ~ +map_size/2
  - 그리드 (i, j): i = y축 방향 (행), j = x축 방향 (열). row-major.
"""
import numpy as np


class FogMap:
    def __init__(self, map_size=(24.0, 24.0), cell_size=0.1, reveal_radius=2.0,
                 grid_n=3, obstacles_meta=None):
        """
        Args:
            map_size:     (W, H) 맵 한 변 길이 (m)
            cell_size:    한 셀 한 변 (m). 작을수록 부드럽고 무거움
            reveal_radius: 로봇 주위 reveal 반경 (m)
            grid_n:       구역 그리드 (NxN). 9구역이면 3
            obstacles_meta: [{x, y, w, h}, ...] 장애물 박스 정보.
                            ratio 계산 시 obstacle 셀은 분모에서 제외.
        """
        self.map_w, self.map_h = float(map_size[0]), float(map_size[1])
        self.cell_size = float(cell_size)
        self.reveal_radius = float(reveal_radius)
        self.grid_n = int(grid_n)

        # 셀 개수
        self.cols = int(round(self.map_w / self.cell_size))   # j (x축)
        self.rows = int(round(self.map_h / self.cell_size))   # i (y축)

        # 안개 그리드: 0 = 안개, 1 = 밝힘
        self.fog = np.zeros((self.rows, self.cols), dtype=np.uint8)

        # 장애물 마스크 (1 = 장애물 내부, ratio 분모에서 제외)
        self.obstacle_mask = np.zeros((self.rows, self.cols), dtype=np.uint8)
        if obstacles_meta:
            self._mark_obstacles(obstacles_meta)

        # 구역 경계 (월드 좌표 기준)
        self.sector_w = self.map_w / self.grid_n
        self.sector_h = self.map_h / self.grid_n

        # 각 셀이 속한 구역 인덱스 (0..grid_n*grid_n-1, 좌하단=0, 행 우선)
        self.sector_of_cell = self._compute_sector_grid()
        # 각 구역의 reachable 셀 수 (장애물 제외)
        self.sector_reachable = np.zeros(self.grid_n ** 2, dtype=np.int32)
        for s in range(self.grid_n ** 2):
            m = (self.sector_of_cell == s) & (self.obstacle_mask == 0)
            self.sector_reachable[s] = int(m.sum())

    # ── 좌표 변환 ────────────────────────────────────────
    def world_to_cell(self, x, y):
        """월드 (x, y) → 셀 (i, j). 범위 벗어나면 None."""
        j = int((x + self.map_w / 2.0) / self.cell_size)
        i = int((y + self.map_h / 2.0) / self.cell_size)
        if i < 0 or i >= self.rows or j < 0 or j >= self.cols:
            return None
        return i, j

    def cell_to_world(self, i, j):
        """셀 중심의 월드 좌표."""
        x = (j + 0.5) * self.cell_size - self.map_w / 2.0
        y = (i + 0.5) * self.cell_size - self.map_h / 2.0
        return x, y

    def world_to_sector(self, x, y):
        """월드 → 구역 인덱스 (0..N²-1). 좌하단=0, 행우선."""
        cx = (x + self.map_w / 2.0) / self.sector_w
        cy = (y + self.map_h / 2.0) / self.sector_h
        col = max(0, min(self.grid_n - 1, int(cx)))
        row = max(0, min(self.grid_n - 1, int(cy)))
        return row * self.grid_n + col

    def sector_bounds(self, sector_idx):
        """구역의 월드 좌표 경계 (x_min, x_max, y_min, y_max)."""
        row = sector_idx // self.grid_n
        col = sector_idx % self.grid_n
        x_min = col * self.sector_w - self.map_w / 2.0
        x_max = x_min + self.sector_w
        y_min = row * self.sector_h - self.map_h / 2.0
        y_max = y_min + self.sector_h
        return x_min, x_max, y_min, y_max

    # ── 안개 갱신 ────────────────────────────────────────
    def reveal_around(self, x, y):
        """월드 (x, y) 주변 reveal_radius 안의 셀을 밝힘."""
        center = self.world_to_cell(x, y)
        if center is None:
            return 0
        ci, cj = center

        r_cells = int(np.ceil(self.reveal_radius / self.cell_size))
        i_lo = max(0, ci - r_cells)
        i_hi = min(self.rows, ci + r_cells + 1)
        j_lo = max(0, cj - r_cells)
        j_hi = min(self.cols, cj + r_cells + 1)

        ii = np.arange(i_lo, i_hi)
        jj = np.arange(j_lo, j_hi)
        I, J = np.meshgrid(ii, jj, indexing="ij")
        # 셀 중심까지의 월드 거리
        cell_xs = (J + 0.5) * self.cell_size - self.map_w / 2.0
        cell_ys = (I + 0.5) * self.cell_size - self.map_h / 2.0
        d2 = (cell_xs - x) ** 2 + (cell_ys - y) ** 2
        mask = d2 <= self.reveal_radius ** 2

        new_cells = mask & (self.fog[i_lo:i_hi, j_lo:j_hi] == 0)
        n_new = int(new_cells.sum())
        self.fog[i_lo:i_hi, j_lo:j_hi][mask] = 1
        return n_new

    # ── 진척률 ──────────────────────────────────────────
    def sector_revealed_ratio(self, sector_idx):
        """장애물 셀은 분모에서 제외 → 도달 가능한 영역 기준 ratio."""
        total = int(self.sector_reachable[sector_idx])
        if total == 0:
            return 1.0
        mask = (self.sector_of_cell == sector_idx) & (self.obstacle_mask == 0)
        revealed = int(self.fog[mask].sum())
        return revealed / total

    def all_sector_ratios(self):
        return [self.sector_revealed_ratio(s) for s in range(self.grid_n ** 2)]

    def potential_new_reveal(self, x, y):
        """월드 (x, y) 에 로버가 가면 새로 밝혀질 셀 수.

        이미 밝혀진 셀 / 장애물 셀 / 반경 밖 셀은 제외.
        anchor 가 skip 할만한지 판정용.
        """
        center = self.world_to_cell(x, y)
        if center is None:
            return 0
        ci, cj = center

        r_cells = int(np.ceil(self.reveal_radius / self.cell_size))
        i_lo = max(0, ci - r_cells)
        i_hi = min(self.rows, ci + r_cells + 1)
        j_lo = max(0, cj - r_cells)
        j_hi = min(self.cols, cj + r_cells + 1)

        ii = np.arange(i_lo, i_hi)
        jj = np.arange(j_lo, j_hi)
        I, J = np.meshgrid(ii, jj, indexing="ij")
        cell_xs = (J + 0.5) * self.cell_size - self.map_w / 2.0
        cell_ys = (I + 0.5) * self.cell_size - self.map_h / 2.0
        d2 = (cell_xs - x) ** 2 + (cell_ys - y) ** 2

        region_fog = self.fog[i_lo:i_hi, j_lo:j_hi]
        region_obs = self.obstacle_mask[i_lo:i_hi, j_lo:j_hi]
        mask = (d2 <= self.reveal_radius ** 2) & (region_fog == 0) & (region_obs == 0)
        return int(mask.sum())

    def overall_ratio(self):
        """전체 reachable 영역 중 밝혀진 비율."""
        reachable = (self.obstacle_mask == 0)
        total = int(reachable.sum())
        if total == 0:
            return 1.0
        return int(self.fog[reachable].sum()) / total

    def set_obstacle_mask(self, mask):
        """외부 장애물 마스크(0/1)를 설정하고 sector_reachable 재계산.

        실제 맵의 obstacle_grid.npy (raw rock 영역) 를 쓸 때 사용.
        ratio 계산 분모에서 이 셀들이 제외된다.
        """
        mask = np.asarray(mask, dtype=np.uint8)
        if mask.shape != (self.rows, self.cols):
            raise ValueError(
                f"mask shape {mask.shape} != ({self.rows}, {self.cols})")
        self.obstacle_mask = mask
        for s in range(self.grid_n ** 2):
            m = (self.sector_of_cell == s) & (self.obstacle_mask == 0)
            self.sector_reachable[s] = int(m.sum())

    # ── 내부 ────────────────────────────────────────────
    def _mark_obstacles(self, obstacles_meta):
        """장애물 박스가 차지하는 셀을 obstacle_mask 에 1로 마킹."""
        for ob in obstacles_meta:
            x = float(ob["x"]); y = float(ob["y"])
            w = float(ob["w"]); h = float(ob["h"])
            x_lo, x_hi = x - w / 2.0, x + w / 2.0
            y_lo, y_hi = y - h / 2.0, y + h / 2.0
            j_lo = max(0, int((x_lo + self.map_w / 2.0) / self.cell_size))
            j_hi = min(self.cols, int((x_hi + self.map_w / 2.0) / self.cell_size) + 1)
            i_lo = max(0, int((y_lo + self.map_h / 2.0) / self.cell_size))
            i_hi = min(self.rows, int((y_hi + self.map_h / 2.0) / self.cell_size) + 1)
            self.obstacle_mask[i_lo:i_hi, j_lo:j_hi] = 1

    def _compute_sector_grid(self):
        """각 셀 → 구역 인덱스 매핑 미리 계산."""
        sector = np.zeros((self.rows, self.cols), dtype=np.int32)
        for i in range(self.rows):
            y = (i + 0.5) * self.cell_size - self.map_h / 2.0
            row = max(0, min(self.grid_n - 1,
                             int((y + self.map_h / 2.0) / self.sector_h)))
            for j in range(self.cols):
                x = (j + 0.5) * self.cell_size - self.map_w / 2.0
                col = max(0, min(self.grid_n - 1,
                                 int((x + self.map_w / 2.0) / self.sector_w)))
                sector[i, j] = row * self.grid_n + col
        return sector
