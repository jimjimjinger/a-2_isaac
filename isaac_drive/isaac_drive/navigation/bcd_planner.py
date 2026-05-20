"""Boustrophedon Cellular Decomposition (BCD).

수직 sweep line 으로 한 sub-grid 영역을 장애물 기준 sub-cell 로 분할 후,
각 sub-cell 안에서 수평 zigzag anchor 점을 생성한다.

사용:
    cell_label = bcd_decompose(sub_grid)       # 0=block, 1+=cell id
    anchors    = generate_zigzag_anchors(
                     cell_label == cid,
                     origin_xy, cell_size_m, spacing_m,
                 )
"""
import numpy as np


def bcd_decompose(grid):
    """수직 sweep line BCD.

    Args:
        grid: (rows, cols) uint8. 0=free, 1=block.

    Returns:
        cell_label: 같은 shape int32. 0=block, 1+=cell id (왼쪽 컬럼부터).
        이벤트(SPLIT/MERGE)마다 새 cell id 발급.
    """
    rows, cols = grid.shape
    cell_label = np.zeros((rows, cols), dtype=np.int32)
    prev_segs = []          # [(top, bot, cell_id), ...]
    next_id = 1

    for j in range(cols):
        col = grid[:, j]
        curr_segs = _free_segments(col)

        # curr_seg → 겹치는 prev_seg 인덱스 리스트
        curr_overlaps = [[] for _ in curr_segs]
        # prev_seg → 겹치는 curr_seg 인덱스 리스트
        prev_overlaps = [[] for _ in prev_segs]
        for ci, (ct, cb) in enumerate(curr_segs):
            for pi, (pt, pb, _) in enumerate(prev_segs):
                if not (pb < ct or pt > cb):
                    curr_overlaps[ci].append(pi)
                    prev_overlaps[pi].append(ci)

        new_prev = []
        for ci, (ct, cb) in enumerate(curr_segs):
            overlaps = curr_overlaps[ci]
            if len(overlaps) == 0:
                cid = next_id; next_id += 1                    # IN
            elif len(overlaps) == 1:
                pi = overlaps[0]
                if len(prev_overlaps[pi]) >= 2:
                    cid = next_id; next_id += 1                # SPLIT
                else:
                    cid = prev_segs[pi][2]                     # 그대로 이어짐
            else:
                cid = next_id; next_id += 1                    # MERGE

            cell_label[ct:cb + 1, j] = cid
            new_prev.append((ct, cb, cid))

        prev_segs = new_prev

    return cell_label


def merge_small_cells(cell_label, min_area_cells):
    """min_area_cells 미만의 작은 sub-cell 을 인접한 가장 큰 cell 에 흡수.

    BCD 는 장애물이 영역 경계에 걸칠 때 anchor 하나 못 넣을 만큼 잘게
    쪼개진 sub-cell 을 만든다. 그런 조각을 인접 cell 로 합쳐 실용적으로
    만든다. 인접 cell 이 없는 고립 조각은 그대로 남는다.

    Args:
        cell_label:     bcd_decompose 출력 (0=block, 1+=cell id).
        min_area_cells: 이 셀 수 미만이면 작은 cell 로 간주.

    Returns:
        새 cell_label. id 가 1..K 로 재정렬됨.
    """
    label = cell_label.astype(np.int32).copy()
    rows, cols = label.shape

    while True:
        ids, counts = np.unique(label[label > 0], return_counts=True)
        if len(ids) <= 1:
            break
        area = {int(i): int(c) for i, c in zip(ids, counts)}
        small = [i for i in area if area[i] < min_area_cells]
        if not small:
            break

        sid = min(small, key=lambda i: area[i])     # 가장 작은 것부터
        mask = (label == sid)

        # 4-이웃으로 닿는 다른 cell id 수집
        nbr = set()
        ys, xs = np.where(mask)
        for y, x in zip(ys, xs):
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < rows and 0 <= nx < cols:
                    v = int(label[ny, nx])
                    if v > 0 and v != sid:
                        nbr.add(v)
        if not nbr:
            break                                    # 고립 — 더 못 합침
        target = max(nbr, key=lambda i: area.get(i, 0))
        label[mask] = target

    return _relabel(label)


def _relabel(label):
    """cell id 를 1..K 연속 정수로 재정렬."""
    out = np.zeros_like(label)
    old_ids = sorted(int(i) for i in np.unique(label) if i > 0)
    for new_id, old in enumerate(old_ids, start=1):
        out[label == old] = new_id
    return out


def _free_segments(col):
    """1D 컬럼에서 연속된 free(0) 구간 (top, bot) 리스트."""
    segs = []
    n = len(col)
    i = 0
    while i < n:
        if col[i] == 0:
            j = i
            while j + 1 < n and col[j + 1] == 0:
                j += 1
            segs.append((i, j))
            i = j + 1
        else:
            i += 1
    return segs


def _erode(mask, k):
    """mask 를 k 셀 정사각형 SE 로 침식 (배열 경계 밖 = 0 으로 취급).

    경계에서 k 셀 이상 떨어진 셀만 남는다.
    """
    m = mask
    for _ in range(int(k)):
        p = np.zeros((m.shape[0] + 2, m.shape[1] + 2), dtype=bool)
        p[1:-1, 1:-1] = m
        m = (p[1:-1, 1:-1] & p[:-2, 1:-1] & p[2:, 1:-1]
             & p[1:-1, :-2] & p[1:-1, 2:]
             & p[:-2, :-2] & p[:-2, 2:] & p[2:, :-2] & p[2:, 2:])
    return m


def generate_zigzag_anchors(mask, origin_xy, cell_size, spacing_m,
                            clearance_m=0.0):
    """한 sub-cell mask 위에서 수평 zigzag anchor (world 좌표) 생성.

    Args:
        mask:        (rows, cols) bool. 이 sub-cell 의 셀들.
        origin_xy:   (x0, y0). mask[0,0] 의 world 좌표 (cell 좌하단).
        cell_size:   m/cell.
        spacing_m:   anchor 간격 (m).
        clearance_m: anchor 를 mask 경계(벽·장애물)에서 이만큼 안쪽에만
                     배치 (보통 reveal 반경). 그 anchor 의 reveal 원이
                     경계까지 덮으므로 로봇이 경계 끝까지 안 가도 된다.

    Returns:
        list[(x, y)]. zigzag 순서.
    """
    if not mask.any():
        return []

    # 경계에서 clearance_m 안쪽 코어에만 anchor 배치.
    core = mask
    if clearance_m > 0:
        k_erode = int(round(clearance_m / cell_size))
        eroded = _erode(mask, k_erode)
        if eroded.any():
            core = eroded   # 침식하면 다 사라지는 좁은 sub-cell 은 원래대로

    spacing_cells = max(1, int(round(spacing_m / cell_size)))
    rows_with = np.where(core.any(axis=1))[0]
    r_lo, r_hi = int(rows_with[0]), int(rows_with[-1])

    # 행 인덱스 선택: spacing_cells 간격 + 마지막 행 보강
    row_pts = list(range(r_lo, r_hi + 1, spacing_cells))
    if row_pts[-1] != r_hi:
        row_pts.append(r_hi)

    anchors = []
    for k, r in enumerate(row_pts):
        cols_in = np.where(core[r])[0]
        if len(cols_in) == 0:
            continue
        c_lo, c_hi = int(cols_in[0]), int(cols_in[-1])
        col_pts = list(range(c_lo, c_hi + 1, spacing_cells))
        if col_pts[-1] != c_hi:
            col_pts.append(c_hi)
        if k % 2 == 1:
            col_pts = col_pts[::-1]
        for c in col_pts:
            if core[r, c]:
                wx = origin_xy[0] + (c + 0.5) * cell_size
                wy = origin_xy[1] + (r + 0.5) * cell_size
                anchors.append((float(wx), float(wy)))
    return anchors


def order_cells_nearest_neighbor(cell_centers, start_xy):
    """sub-cell 중심들을 시작점에서 가장 가까운 순서로 방문 (greedy NN).

    Args:
        cell_centers: list[(cid, (x, y))].
        start_xy:     (x, y) 시작 위치.

    Returns:
        list[cid] — 방문 순서.
    """
    remaining = list(cell_centers)
    cur = np.array(start_xy)
    order = []
    while remaining:
        ds = [np.hypot(c[1][0] - cur[0], c[1][1] - cur[1]) for c in remaining]
        idx = int(np.argmin(ds))
        chosen = remaining.pop(idx)
        order.append(chosen[0])
        cur = np.array(chosen[1])
    return order
