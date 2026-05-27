"""장애물을 2D 그리드로 변환 (A* 입력).

  - 각 셀: 0 = free, 1 = blocked
  - 장애물 박스를 로봇 반경만큼 inflate 해서 표시 → A* 가 절대 안 박힘
  - FogMap 과 동일한 좌표계 / 해상도
"""
import numpy as np


class ObstacleGrid:
    def __init__(self, map_size=(24.0, 24.0), cell_size=0.1, robot_radius=1.0):
        self.map_w, self.map_h = float(map_size[0]), float(map_size[1])
        self.cell_size = float(cell_size)
        self.robot_radius = float(robot_radius)
        self.cols = int(round(self.map_w / self.cell_size))
        self.rows = int(round(self.map_h / self.cell_size))
        self.grid = np.zeros((self.rows, self.cols), dtype=np.uint8)

    def world_to_cell(self, x, y, clip=False):
        j = int((x + self.map_w / 2.0) / self.cell_size)
        i = int((y + self.map_h / 2.0) / self.cell_size)
        if clip:
            i = max(0, min(self.rows - 1, i))
            j = max(0, min(self.cols - 1, j))
        return i, j

    def cell_to_world(self, i, j):
        x = (j + 0.5) * self.cell_size - self.map_w / 2.0
        y = (i + 0.5) * self.cell_size - self.map_h / 2.0
        return x, y

    def in_bounds(self, i, j):
        return 0 <= i < self.rows and 0 <= j < self.cols

    def is_free(self, i, j):
        return self.in_bounds(i, j) and self.grid[i, j] == 0

    def add_obstacles(self, obstacles_meta):
        """장애물 박스를 inflate 해서 그리드에 표시."""
        r = self.robot_radius
        for ob in obstacles_meta:
            x, y, w, h = ob["x"], ob["y"], ob["w"], ob["h"]
            half_w = w / 2.0 + r
            half_h = h / 2.0 + r
            i_lo, j_lo = self.world_to_cell(x - half_w, y - half_h, clip=True)
            i_hi, j_hi = self.world_to_cell(x + half_w, y + half_h, clip=True)
            self.grid[i_lo:i_hi + 1, j_lo:j_hi + 1] = 1

        # 맵 외곽 1셀도 막음 (벽처럼 처리)
        self.grid[0, :] = 1
        self.grid[-1, :] = 1
        self.grid[:, 0] = 1
        self.grid[:, -1] = 1

        n_blocked = int(self.grid.sum())
        print(f"[obstacle_grid] {self.rows}×{self.cols} 그리드, "
              f"막힌 셀 {n_blocked} ({n_blocked / self.grid.size * 100:.1f}%), "
              f"inflate={r}m")

    def set_grid(self, grid):
        """외부에서 만든 0/1 격자(robot inflate·외곽 포함 완성본)를 직접 설정.

        실제 맵의 obstacle_grid.npy 처럼 이미 완성된 격자를 쓸 때 사용.
        add_obstacles 대신 호출.
        """
        grid = np.asarray(grid, dtype=np.uint8)
        if grid.shape != (self.rows, self.cols):
            raise ValueError(
                f"grid shape {grid.shape} != ({self.rows}, {self.cols})")
        self.grid = grid
        n_blocked = int(self.grid.sum())
        print(f"[obstacle_grid] {self.rows}×{self.cols} 그리드 (외부 로드), "
              f"막힌 셀 {n_blocked} ({n_blocked / self.grid.size * 100:.1f}%)")
