"""coverage_node — T3 coverage 알고리즘을 ROS2 인터페이스로 감싸는 노드.

navigation/ 의 coverage 알고리즘(SectorPlanner·Navigator·Mission·FogMap)은
그대로 두고(무수정), 입·출력만 팀 인터페이스에 연결한다:

  입력:
    · I1  terrain 자산 (obstacle_grid.npy / meta.json) — 파일, 시작 시 1회 로드
    · I5  /rover/estimated_pose (geometry_msgs/PoseWithCovarianceStamped) — pose
  출력:
    · /cmd_vel               (geometry_msgs/Twist)                  — 실제 바퀴 구동
    · /selected_drive_action (isaac_interfaces/SelectedDriveAction)  — 구동 결정 가시화
    · /mission_state         (isaac_interfaces/MissionState)         — coverage 상태

매 tick: pose 읽기 → fog reveal → mission.update() → 명령/상태 발행.
알고리즘에는 PoseProvider 를 rover 로 주입하므로 navigation/ 코드는 손대지 않는다.
"""
from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from isaac_interfaces.msg import MissionState, SelectedDriveAction
from rclpy.node import Node

from isaac_drive.navigation.coverage_planner import SectorPlanner
from isaac_drive.navigation.mission_fsm import Mission
from isaac_drive.navigation.navigator import Navigator
from isaac_drive.navigation.pose_provider import PoseProvider, quat_to_yaw
from isaac_drive.navigation.terrain_loader import load_terrain

def _find_default_terrain() -> str:
    """terrain_00001 자산 폴더를 후보 위치에서 탐색.

    coverage_node 는 소스 트리(헤드리스 스크립트)·colcon 빌드(ros2 run) 양쪽에서
    실행될 수 있고, terrain 자산은 소스 트리에만 있다(install share 에 없음).
    실행 위치별 후보를 순서대로 시도하고, 없으면 "" → 파라미터로 받아야 한다.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    ws_from_build = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    candidates = [
        os.path.dirname(os.path.dirname(here)),                # 소스 실행 → a2_isaac
        os.path.join(ws_from_build, "src", "a2_isaac"),        # build/install 실행
        os.path.expanduser("~/dev_ws/rover_ws/src/a2_isaac"),  # README 표준 위치
    ]
    for root in candidates:
        path = os.path.join(root, "isaac_sim", "assets",
                            "generated_terrains", "terrain_00001")
        if os.path.isdir(path):
            return path
    return ""


_DEFAULT_TERRAIN = _find_default_terrain()


class CoverageNode(Node):
    """coverage 알고리즘을 ROS2 토픽 입·출력으로 구동하는 노드."""

    def __init__(self) -> None:
        super().__init__("coverage_node")

        # ── 파라미터 ──
        self.declare_parameter("terrain_dir", _DEFAULT_TERRAIN)
        self.declare_parameter("pose_topic", "/rover/estimated_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tick_hz", 30.0)
        self.declare_parameter("cell_size", 0.1)
        self.declare_parameter("robot_radius", 0.7)
        self.declare_parameter("reveal_radius", 2.0)
        self.declare_parameter("grid_n", 3)
        self.declare_parameter("max_lin", 3.0)
        self.declare_parameter("max_ang", 1.5)
        self.declare_parameter("sector_done_ratio", 0.95)

        terrain_dir = str(self.get_parameter("terrain_dir").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        tick_hz = float(self.get_parameter("tick_hz").value)
        reveal_radius = float(self.get_parameter("reveal_radius").value)

        # ── I1: terrain 자산 로드 (파일, numpy) ──
        if not os.path.isdir(terrain_dir):
            raise FileNotFoundError(
                f"terrain_dir 가 없습니다: {terrain_dir} "
                f"— 파라미터 terrain_dir 로 지정하세요.")
        self.meta, ogrid, self.fog = load_terrain(
            terrain_dir,
            cell_size=float(self.get_parameter("cell_size").value),
            robot_radius=float(self.get_parameter("robot_radius").value),
            reveal_radius=reveal_radius,
            grid_n=int(self.get_parameter("grid_n").value),
        )

        # ── coverage 알고리즘 (navigation/ — 무수정) ──
        # PoseProvider 를 rover 로 주입: Mission/Navigator 는 .get_pose_2d() 만 호출.
        self.pose = PoseProvider()
        planner = SectorPlanner(self.fog, ogrid, reveal_radius=reveal_radius)
        navigator = Navigator(
            self.pose,
            max_lin=float(self.get_parameter("max_lin").value),
            max_ang=float(self.get_parameter("max_ang").value),
        )
        self.mission = Mission(
            self.fog, ogrid, planner, navigator, self.pose,
            sector_done_ratio=float(self.get_parameter("sector_done_ratio").value),
        )

        # ── I5 입력: pose 구독 ──
        self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._on_pose, 10)

        # ── 출력 publisher ──
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.action_pub = self.create_publisher(
            SelectedDriveAction, "/selected_drive_action", 10)
        self.state_pub = self.create_publisher(MissionState, "/mission_state", 10)

        # ── tick 타이머 ──
        self._tick_index = 0
        self._prev_state = ""
        self._done_announced = False
        self.create_timer(1.0 / max(tick_hz, 1.0), self._tick)

        self.get_logger().info(
            f"coverage_node ready — terrain={os.path.basename(terrain_dir)}, "
            f"map {self.fog.map_w:.0f}x{self.fog.map_h:.0f}m, "
            f"pose_topic={pose_topic}, tick={tick_hz:.0f}Hz")

    # ── I5 구독 콜백: PoseWithCovarianceStamped → PoseProvider ──
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.pose.update(p.x, p.y, yaw, tuple(msg.pose.covariance))

    # ── tick: coverage 한 스텝 ──
    def _tick(self) -> None:
        if not self.pose.has_pose:
            # 아직 pose 미수신 — 정지 명령만 발행하고 대기.
            self.cmd_pub.publish(Twist())
            return

        x, y, _ = self.pose.get_pose_2d()
        self.fog.reveal_around(x, y)

        if self.mission.is_done():
            lin, ang = 0.0, 0.0
            if not self._done_announced:
                self.get_logger().info(
                    f"coverage 완료 — reveal {self.fog.overall_ratio()*100:.1f}%")
                self._done_announced = True
        else:
            lin, ang = self.mission.update(self._tick_index)

        self._tick_index += 1
        self._publish_cmd(lin, ang)
        self._publish_action(lin, ang)
        self._publish_state()

    def _publish_cmd(self, lin: float, ang: float) -> None:
        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self.cmd_pub.publish(twist)

    def _publish_action(self, lin: float, ang: float) -> None:
        msg = SelectedDriveAction()
        msg.action = self.mission.state
        msg.linear_velocity = float(lin)
        msg.angular_velocity = float(ang)
        target = self.mission.nav.current_target
        if target is not None:
            msg.target_x = float(target[0])
            msg.target_y = float(target[1])
        msg.confidence = 1.0
        msg.reason = f"coverage sweep (sector {self.mission.current_sector + 1})"
        self.action_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = MissionState()
        msg.state = self.mission.state
        msg.previous_state = self._prev_state
        # MissionState 에 coverage 진척률(reveal %) 전용 필드가 없어 active_task 에
        # 병기한다 — 인터페이스 갭, T4 와 협의 필요.
        msg.active_task = f"coverage:{self.fog.overall_ratio()*100:.1f}%"
        self.state_pub.publish(msg)
        self._prev_state = self.mission.state


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
