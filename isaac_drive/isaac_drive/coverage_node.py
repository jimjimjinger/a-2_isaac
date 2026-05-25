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
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from isaac_interfaces.msg import MissionState, SelectedDriveAction
from rclpy.node import Node

from isaac_drive.minimap_publisher import MinimapPublisher
from isaac_drive.navigation.coverage_planner import SectorPlanner
from isaac_drive.navigation.mission_fsm import Mission
from isaac_drive.navigation.navigator import Navigator
from isaac_drive.navigation.pose_provider import PoseProvider, quat_to_yaw
from isaac_drive.navigation.terrain_loader import load_terrain

def _find_default_terrain() -> str:
    """terrain_00004 자산 폴더를 후보 위치에서 탐색.

    terrain_00004 는 v2 생성기 산출물로 obstacle_grid 가 USD 씬과 정합한다.
    terrain_00001 은 옛 v1 잔재라 obstacle_grid 가 씬과 ~180° 어긋나므로 쓰지 않는다.
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
                            "generated_terrains", "terrain_00004")
        if os.path.isdir(path):
            return path
    return ""


_DEFAULT_TERRAIN = _find_default_terrain()


def _find_scripts_dir() -> str:
    """state_writer.py · viewer.py 가 있는 scripts/ 디렉터리 경로.

    coverage_node 는 소스 트리·colcon 설치(ros2 run) 양쪽에서 import 되고,
    scripts/ 는 소스 트리에만 있다(install 에 없음). ros2 run 시 __file__ 은
    build 트리를 가리키므로 realpath 로 심링크를 풀어 소스 경로를 얻은 뒤,
    후보 위치를 순서대로 시도한다. 못 찾으면 "" 반환(미니맵 비활성).
    """
    here = os.path.dirname(os.path.realpath(__file__))   # .../isaac_drive/isaac_drive
    candidates = [
        os.path.join(os.path.dirname(here), "scripts"),
        os.path.expanduser("~/dev_ws/rover_ws/src/a2_isaac/isaac_drive/scripts"),
    ]
    for path in candidates:
        if os.path.isfile(os.path.join(path, "state_writer.py")):
            return path
    return ""


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
        self.declare_parameter("enable_minimap", True)
        self.declare_parameter("viewer_write_every", 3)
        self.declare_parameter("enable_minimap_topics", True)
        self.declare_parameter("minimap_publish_every", 10)
        self.declare_parameter("minimap_frame_id", "map")

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

        # External replan trigger — supervisor 가 EXPLORE 재진입 시 publish.
        # 받으면 현재 DRIVE path 무시하고 즉시 PLAN_PATH 로 강제 전환해
        # 로버의 새 위치 기준으로 다음 anchor 까지 path 재계산.
        from std_msgs.msg import Empty as _Empty
        self.create_subscription(
            _Empty, "/coverage/replan_request", self._on_replan_request, 10)

        # ── 출력 publisher ──
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.action_pub = self.create_publisher(
            SelectedDriveAction, "/selected_drive_action", 10)
        self.state_pub = self.create_publisher(MissionState, "/mission_state", 10)

        # ── 미니맵 viewer (T3 의 StateWriter + viewer.py 재배선) ──
        # coverage 알고리즘이 fog·mission 을 들고 있는 유일한 프로세스라
        # 진척 미니맵도 여기서 띄운다. viewer 는 StateWriter 가 clean env +
        # 시스템 python3 로 띄우므로 ROS2/Isaac 환경과 무관하게 동작한다.
        self.writer = None
        if bool(self.get_parameter("enable_minimap").value):
            self._start_minimap(int(self.get_parameter("viewer_write_every").value))

        # ── 미니맵 ROS2 토픽 발행 (RViz/Foxglove/T4 mission UI 용) ──
        # viewer.py(matplotlib) 와 별개로, 같은 상태를 표준 메시지로 토픽에 싣는다.
        # 소비자는 구독만으로 미니맵을 그릴 수 있다 — 렌더링 코드 불필요.
        self.minimap = None
        if bool(self.get_parameter("enable_minimap_topics").value):
            self.minimap = MinimapPublisher(
                self, self.fog,
                frame_id=str(self.get_parameter("minimap_frame_id").value),
                publish_every=int(
                    self.get_parameter("minimap_publish_every").value),
            )
            self.get_logger().info(
                "미니맵 토픽 발행 — /mission/minimap /mission/path /mission/markers")

        # ── tick 타이머 ──
        self._tick_index = 0
        self._prev_state = ""
        self._done_announced = False
        self.create_timer(1.0 / max(tick_hz, 1.0), self._tick)

        self.get_logger().info(
            f"coverage_node ready — terrain={os.path.basename(terrain_dir)}, "
            f"map {self.fog.map_w:.0f}x{self.fog.map_h:.0f}m, "
            f"pose_topic={pose_topic}, tick={tick_hz:.0f}Hz")

    # ── 미니맵 viewer 시작 (scripts/ 의 StateWriter import) ──
    def _start_minimap(self, write_every: int) -> None:
        scripts_dir = _find_scripts_dir()
        if not scripts_dir:
            self.get_logger().warning(
                "미니맵 비활성화 — scripts/ 디렉터리를 못 찾음")
            return
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
        try:
            from state_writer import StateWriter
        except ImportError as exc:
            self.get_logger().warning(
                f"미니맵 비활성화 — state_writer import 실패: {exc}")
            return
        viewer = os.path.join(scripts_dir, "viewer.py")
        self.writer = StateWriter(
            self.fog, viewer_script_path=viewer, write_every=write_every)
        self.get_logger().info(f"미니맵 viewer 시작 — {viewer}")

    # ── I5 구독 콜백: PoseWithCovarianceStamped → PoseProvider ──
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.pose.update(p.x, p.y, yaw, tuple(msg.pose.covariance))

    def _on_replan_request(self, _msg) -> None:
        # Force a path replan from the rover's current pose. Used after
        # APPROACH/PICK exits so the stale DRIVE path (still pointing at the
        # pre-detour anchor) is discarded.
        self.mission.state = "PLAN_PATH"
        self.get_logger().info("coverage replan requested (force PLAN_PATH)")

    # ── tick: coverage 한 스텝 ──
    def _tick(self) -> None:
        if not self.pose.has_pose:
            # 아직 pose 미수신 — 정지 명령만 발행하고 대기.
            self.cmd_pub.publish(Twist())
            return

        x, y, yaw = self.pose.get_pose_2d()
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

        # 미니맵 viewer 로 fog·mission 상태 기록 (write_every tick 마다).
        if self.writer is not None:
            self.writer.maybe_write(self._tick_index, (x, y, yaw), self.mission)

        # 미니맵 상태를 표준 ROS2 토픽으로도 발행 (publish_every tick 마다).
        if self.minimap is not None:
            self.minimap.maybe_publish(self._tick_index, self.mission)

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

    # ── 종료 정리: 미니맵 viewer 프로세스까지 함께 종료 ──
    def destroy_node(self) -> None:
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        super().destroy_node()


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
