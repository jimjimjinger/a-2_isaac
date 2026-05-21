"""PoseProvider — ROS2 토픽으로 받은 pose를 알고리즘이 먹을 (x,y,yaw)로 제공.

navigation/ 의 Mission·Navigator 는 rover 객체에서 `.get_pose_2d()` 를 호출한다.
ROS2 노드에는 Isaac Sim 쪽 RoverController 가 없으므로, 노드가 pose 토픽
(I5: /rover/estimated_pose, geometry_msgs/PoseWithCovarianceStamped)을 구독해
`update()` 로 최신값을 먹이고, 알고리즘에는 이 PoseProvider 를 rover 로 넘긴다.

이 모듈은 ROS2·Isaac Sim 에 의존하지 않는 순수 파이썬 — navigation/ 의 다른
모듈처럼 헤드리스 테스트에서도 그대로 쓸 수 있다. 토픽 구독과 쿼터니언→yaw
변환은 노드(coverage_node)가 담당한다.

GT ↔ T5 localization 전환은 노드가 어느 토픽을 구독하느냐(파라미터)만 바꾸면
되고, 이 클래스는 그대로다. RL driving_policy_node 등 다른 pose 소비자도 재사용.
"""
from __future__ import annotations

import math


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → 2D yaw (rad). Z축 회전 성분만 추출."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseProvider:
    """알고리즘(Mission/Navigator)에 (x, y, yaw) pose를 제공하는 어댑터.

    Mission/Navigator 가 기대하는 rover 인터페이스 중 `.get_pose_2d()` 만
    제공한다. 구동(`.drive()`)은 노드가 직접 cmd_vel 로 발행하므로 불필요.
    """

    def __init__(self, initial: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        self._x = float(initial[0])
        self._y = float(initial[1])
        self._yaw = float(initial[2])
        self._covariance: tuple[float, ...] | None = None
        self._received = False

    def update(self, x: float, y: float, yaw: float,
               covariance: tuple[float, ...] | None = None) -> None:
        """노드의 pose 구독 콜백에서 호출 — 최신 pose 캐시."""
        self._x = float(x)
        self._y = float(y)
        self._yaw = float(yaw)
        self._covariance = covariance
        self._received = True

    def get_pose_2d(self) -> tuple[float, float, float]:
        """알고리즘이 매 tick 호출 — 캐시된 (x, y, yaw) 즉시 반환."""
        return self._x, self._y, self._yaw

    @property
    def has_pose(self) -> bool:
        """pose 를 한 번이라도 받았는지. False 면 아직 구동하면 안 된다."""
        return self._received

    @property
    def covariance(self) -> tuple[float, ...] | None:
        """최신 pose 공분산 (I5 제공). 현재 미사용 — 추후 신뢰도 기반 감속 등."""
        return self._covariance
