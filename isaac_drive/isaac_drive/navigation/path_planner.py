"""8연결 A* 알고리즘 + 경로 단순화.

grid: 2D numpy array uint8 (0=free, 1=blocked)
start, goal: (i, j) 튜플
반환: 셀 인덱스의 리스트 (start → goal), 못 찾으면 None
"""
import heapq
import math


_NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1),
              ( 0, -1),          ( 0, 1),
              ( 1, -1), ( 1, 0), ( 1, 1)]


def astar(grid, start, goal):
    rows, cols = grid.shape
    si, sj = start
    gi, gj = goal
    if not (0 <= si < rows and 0 <= sj < cols
            and 0 <= gi < rows and 0 <= gj < cols):
        return None
    if grid[si, sj] == 1 or grid[gi, gj] == 1:
        return None
    if start == goal:
        return [start]

    open_set = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}

    while open_set:
        _, cur = heapq.heappop(open_set)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path

        ci, cj = cur
        for di, dj in _NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if ni < 0 or ni >= rows or nj < 0 or nj >= cols:
                continue
            if grid[ni, nj] == 1:
                continue
            # 대각선 이동 시 옆 셀 통해 corner-cut 방지
            if di != 0 and dj != 0:
                if grid[ci + di, cj] == 1 or grid[ci, cj + dj] == 1:
                    continue
            step_cost = math.hypot(di, dj)
            tg = g_score[cur] + step_cost
            n = (ni, nj)
            if n not in g_score or tg < g_score[n]:
                came_from[n] = cur
                g_score[n] = tg
                h = math.hypot(ni - gi, nj - gj)
                heapq.heappush(open_set, (tg + h, n))

    return None


def simplify_path(grid, path, max_skip=20):
    """Line-of-sight 단순화: 가능하면 멀리있는 셀로 점프.

    A* 출력은 셀 단위라 너무 빽빽함. 직선으로 막힘 없이 도달 가능한
    최대 점프 길이로 줄여서 navigator 가 따라가기 좋게 만듦.
    """
    if not path or len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    i = 0
    while i < len(path) - 1:
        # path[i] 에서 직선 line-of-sight 가능한 가장 먼 점 찾기
        j_far = min(i + max_skip, len(path) - 1)
        while j_far > i + 1:
            if _line_of_sight(grid, path[i], path[j_far]):
                break
            j_far -= 1
        simplified.append(path[j_far])
        i = j_far
    return simplified


def _line_of_sight(grid, a, b):
    """Bresenham 으로 a→b 사이에 막힌 셀 있는지."""
    i0, j0 = a
    i1, j1 = b
    di = abs(i1 - i0)
    dj = abs(j1 - j0)
    si = 1 if i0 < i1 else -1
    sj = 1 if j0 < j1 else -1
    err = di - dj
    ci, cj = i0, j0
    while True:
        if grid[ci, cj] == 1:
            return False
        if ci == i1 and cj == j1:
            return True
        e2 = 2 * err
        if e2 > -dj:
            err -= dj; ci += si
        if e2 < di:
            err += di; cj += sj
