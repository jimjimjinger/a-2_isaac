# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""전방 직진 + 장애물 회피 컨트롤러.

detect_obstacles() 가 준 장애물 목록으로 (선속도, 각속도) 주행 명령을
만든다.  play_avoid.py 의 자동 모드가 매 프레임 호출한다.

기본 동작은 '직진'이고, 전방에 장애물이 있으면 회피한다 (요청 사양):
  · 전방 장애물이 로봇 기준 오른쪽 → 왼쪽으로 회피
  · 전방 장애물이 로봇 기준 왼쪽   → 오른쪽으로 회피
  · 가까워지는데도 못 벗어나면 회피 각을 점점 키운다
  · 회피하려는 쪽에도 장애물이면 후진 후 반대쪽으로 회피
  · 후방 장애물은 무시

좌표 규약 (detect_obstacles 와 동일):
  obstacle["fwd"] > 0 = 전방,  obstacle["lat"] > 0 = 좌측.
  반환 각속도 ang > 0 = 좌회전.

상태기계: CRUISE(직진) → AVOID(회피) → REVERSE(후진).
"""

from __future__ import annotations


class ObstacleAvoider:
    """장애물 목록 → (선속도, 각속도). 직진하다 전방 장애물을 회피한다.

    매 프레임 compute_action(obstacles) 을 호출한다.  내부 상태(모드·
    회피방향·ramp·후진카운터)를 들고 있으므로, 자동 모드 진입·리셋 시
    reset() 도 함께 불러 준다.
    """

    def __init__(
        self,
        cruise_speed: float = 1.5,
        avoid_speed: float = 1.0,
        reverse_speed: float = 0.8,
        base_ang: float = 0.6,
        max_ang: float = 1.5,
        ramp_step: float = 0.03,
        front_range: float = 2.0,
        corridor: float = 0.7,
        block_lat: float = 0.25,
        reverse_steps: int = 45,
        fwd_eps: float = 0.03,
        clear_steps: int = 25,
        exit_corridor: float = 1.1,
        graze_outer: float = 1.0,
        graze_ang: float = 0.5,
        graze_fwd_back: float = 0.3,
    ) -> None:
        """
        Args:
            cruise_speed:  전방이 깨끗할 때 직진 속도 (m/s).
            avoid_speed:   회피 선회 중 전진 속도 (m/s).
            reverse_speed: 후진 속도 (m/s, 양수로 준다).
            base_ang:      회피 시작 각속도 (rad/s).
            max_ang:       ramp 최대일 때 회피 각속도 (rad/s).
            ramp_step:     프레임당 ramp 증감량 (0~1 스케일).
            front_range:   '전방 장애물'로 칠 최대 전방거리 (m).
            corridor:      이 ±폭(m) 안의 전방 장애물만 진로를 막는 것으로 본다.
            block_lat:     회피 방향 쪽 이 거리(m) 밖 장애물 = 그 쪽도 막힘.
            reverse_steps: 후진을 유지할 프레임 수.
            fwd_eps:       전방거리 증감 판정 둔감폭 (m, 떨림 방지).
            clear_steps:   AVOID → CRUISE 히스테리시스 — 전방이 이 만큼 프레임
                           동안 연속으로 깨끗해야만 CRUISE 로 돌아간다.  그
                           사이엔 회피 방향으로 계속 돌아 완전히 빠져나간다.
            exit_corridor: AVOID 중에는 이 ±폭(m) 안의 전방 장애물이 사라져야
                           '깨끗' 으로 본다.  corridor(=진입 폭) 보다 넓혀
                           AVOID↔CRUISE 챠터링을 막는다 (asymmetric hysteresis).
            graze_outer:   GRAZE(긁힘 방지) 모드 작동 폭. corridor < |lat| <
                           graze_outer 안의 옆구리 장애물에 대해 부드럽게
                           반대쪽으로 비킴 (바퀴 스침 방지).
            graze_ang:     GRAZE 모드 각속도 (rad/s) — 작게.
            graze_fwd_back:GRAZE 대상으로 칠 후방 거리 (m).  차량 바로 옆이나
                           살짝 뒤(바퀴 옆)까지 포함하기 위한 마진.
        """
        self.cruise_speed = cruise_speed
        self.avoid_speed = avoid_speed
        self.reverse_speed = reverse_speed
        self.base_ang = base_ang
        self.max_ang = max_ang
        self.ramp_step = ramp_step
        self.front_range = front_range
        self.corridor = corridor
        self.block_lat = block_lat
        self.reverse_steps = reverse_steps
        self.fwd_eps = fwd_eps
        self.clear_steps = clear_steps
        self.exit_corridor = exit_corridor
        self.graze_outer = graze_outer
        self.graze_ang = graze_ang
        self.graze_fwd_back = graze_fwd_back
        self.reset()

    def reset(self) -> None:
        """내부 상태 초기화 — 자동 모드 진입·env.reset() 과 함께 호출한다."""
        self.mode = "CRUISE"        # CRUISE / GRAZE / AVOID / REVERSE
        self.avoid_dir = 0          # +1 = 좌회전 회피, -1 = 우회전 회피
        self.post_reverse_dir = 0   # 후진 뒤 향할 회피 방향
        self.ramp = 0.0             # 회피 각 강화 정도 (0~1)
        self.reverse_count = 0      # 남은 후진 프레임
        self.prev_min_fwd = None    # 직전 프레임 최근접 전방거리
        self.clear_count = 0        # AVOID 중 연속 '깨끗' 프레임 수

    def compute_action(self, obstacles: list) -> tuple:
        """장애물 목록 → (lin, ang) 주행 명령.

        Args:
            obstacles: detect_obstacles() 반환 리스트. 각 항목 dict 에
                       fwd(전방 m)·lat(좌측 m) 등이 들어 있다.
        Returns:
            (lin, ang): 선속도(m/s, +전진)·각속도(rad/s, +좌회전).
        """
        # --- 후진 중이면 계속 후진. 끝나면 AVOID(강제 방향)로 넘어간다 ---
        if self.mode == "REVERSE":
            self.reverse_count -= 1
            if self.reverse_count > 0:
                return (-self.reverse_speed, 0.0)
            self.mode = "AVOID"                       # 후진 완료
            self.avoid_dir = self.post_reverse_dir    # 반대쪽으로 회피
            self.ramp = 0.0
            self.prev_min_fwd = None
            # 아래 AVOID 로직으로 이어진다.

        # --- 전방 장애물만 추린다 (후방 fwd<=0 은 무시) ---
        front = [o for o in obstacles if 0.0 < o["fwd"] <= self.front_range]
        # 진로를 막는 것 = 좌우 corridor 안의 전방 장애물.
        # AVOID 중엔 더 넓은 exit_corridor 로 봐서 '깨끗' 판정을 엄격하게.
        # → 진입은 좁게(0.7), 탈출은 넓게(1.1) → asymmetric hysteresis.
        active_corridor = self.exit_corridor if self.mode == "AVOID" else self.corridor
        blocking = [o for o in front if abs(o["lat"]) < active_corridor]

        if not blocking:
            if self.mode == "AVOID":
                # 히스테리시스 — clear_steps 만큼 연속 깨끗해야 CRUISE 로 풀어준다.
                # 그 사이엔 회피 방향으로 부드럽게 계속 돌아 확실히 빠져나간다.
                self.clear_count += 1
                if self.clear_count < self.clear_steps:
                    return (self.avoid_speed, self.avoid_dir * self.base_ang)

            # --- GRAZE (긁힘 방지) — 진로는 안 막지만 바퀴 옆구리 가까이 있는
            # 장애물에 대해 부드럽게 반대쪽으로 비킨다. 차량 바로 옆 또는 살짝
            # 뒤(graze_fwd_back) 까지도 포함해서, 스쳐 지나가다 바퀴 쪽으로
            # 걸리는 걸 막는다. obstacles 가 비기 전까지 유지.
            near_side = [
                o for o in obstacles
                if -self.graze_fwd_back < o["fwd"] <= self.front_range
                and self.corridor <= abs(o["lat"]) < self.graze_outer
            ]
            if near_side:
                primary = min(near_side,
                              key=lambda o: o["fwd"] ** 2 + o["lat"] ** 2)
                # 장애물 왼쪽(+lat) → 우회전(ang<0), 오른쪽(-lat) → 좌회전(ang>0)
                graze_dir = 1 if primary["lat"] < 0.0 else -1
                self.mode = "GRAZE"
                self.avoid_dir = graze_dir   # 외부에서 방향 읽을 수 있게 기록
                self.ramp = 0.0
                self.prev_min_fwd = None
                self.clear_count = 0
                return (self.cruise_speed, graze_dir * self.graze_ang)

            # 완전히 깨끗 → CRUISE.
            self.mode = "CRUISE"
            self.ramp = 0.0
            self.prev_min_fwd = None
            self.clear_count = 0
            return (self.cruise_speed, 0.0)

        # 막힘 → 회피 계속. 히스테리시스 카운터 리셋.
        self.clear_count = 0

        primary = min(blocking, key=lambda o: o["fwd"])   # 최근접 위협
        cur_min_fwd = primary["fwd"]

        # --- 회피 방향 결정 — AVOID 진입 시 1번만, 이후 유지(떨림 방지) ---
        if self.mode != "AVOID":
            self.mode = "AVOID"
            # 장애물이 오른쪽(lat<0) → 좌회피(+1) / 왼쪽·정중앙 → 우회피(-1).
            self.avoid_dir = 1 if primary["lat"] < 0.0 else -1
            self.ramp = 0.0
            self.prev_min_fwd = None

        # --- 회피하려는 쪽에도 장애물이면 → 후진 후 반대쪽으로 ---
        if self.avoid_dir > 0:      # 좌회피 중 → 왼쪽 확인
            side_blocked = any(o["lat"] > self.block_lat for o in front)
        else:                      # 우회피 중 → 오른쪽 확인
            side_blocked = any(o["lat"] < -self.block_lat for o in front)
        if side_blocked:
            self.mode = "REVERSE"
            self.reverse_count = self.reverse_steps
            self.post_reverse_dir = -self.avoid_dir
            self.ramp = 0.0
            self.prev_min_fwd = None
            return (-self.reverse_speed, 0.0)

        # --- 회피 각 ramp — 가까워지면 키우고, 멀어지면 줄인다 ---
        if self.prev_min_fwd is not None:
            if cur_min_fwd < self.prev_min_fwd - self.fwd_eps:
                self.ramp = min(1.0, self.ramp + self.ramp_step)   # 못 벗어남
            elif cur_min_fwd > self.prev_min_fwd + self.fwd_eps:
                self.ramp = max(0.0, self.ramp - self.ramp_step)   # 벗어나는 중
        self.prev_min_fwd = cur_min_fwd

        # --- 위치 기반 강화: 장애물이 정중앙에 가까울수록 회피각 ↑ ---
        # |lat|=0(정중앙) → 1, |lat|>=corridor(가장자리) → 0.
        center_boost = max(0.0, 1.0 - abs(primary["lat"]) / self.corridor)

        # 시간 기반(ramp) 과 위치 기반(center_boost) 중 큰 쪽이 강도를 끌어올린다.
        intensity = max(self.ramp, center_boost)

        # intensity 가 클수록 더 천천히·더 크게 꺾는다.
        lin = self.avoid_speed * (1.0 - 0.5 * intensity)
        ang = self.avoid_dir * (self.base_ang
                                + intensity * (self.max_ang - self.base_ang))
        return (lin, ang)
