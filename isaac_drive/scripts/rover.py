"""AAU 로버 (RLRoverLab) spawn + 카메라 부착 + Ackermann 드라이브.

사용 예:
    from rover import RoverController
    rover = RoverController(world)   # world = isaacsim.core.api.World
    rover.spawn()
    rover.attach_camera()
    rover.initialize()                # World.reset() 이후 호출
    rover.drive(lin_vel=1.0, ang_vel=0.5)
    cx, cy, yaw = rover.get_pose_2d()
"""
import re
from pathlib import Path

import numpy as np
import omni.kit.commands
import omni.usd
from pxr import UsdGeom, Gf
import omni.replicator.core as rep
from omni.kit.viewport.utility import create_viewport_window
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


# 차량 USD = 통합 로버 v2 (v1 + 후방·하단 밸러스트). 팀 통일 모델.
# repo 루트 기준 상대경로 (parents[2] = a2_isaac).
_REPO_ROOT = Path(__file__).resolve().parents[2]
ROVER_USD_PATH = str(
    _REPO_ROOT / "isaac_sim" / "assets" / "vehicle" / "vehicle_v2.usd"
)
# vehicle_v1.usd 내부 (defaultPrim=/Root): 단일 articulation(DOF 27).
#   · /World/Rover            — reference 앵커 (spawn translate 대상)
#   · .../m0609/base_link     — articulation root (get_world_pose 가 주는 pose)
#   · .../rover/Body          — 주행 섀시 링크
# 주행 모드 드라이브 게인(휠 속도제어·스티어 위치제어·로커 passive)은
# vehicle_v1.usd 에 nominal default 로 박혀 있다 — 런타임 패치 불필요.
ROVER_PRIM_PATH = "/World/Rover"
ARTIC_PRIM_PATH = ROVER_PRIM_PATH + "/Vehicle/m0609/base_link"
ROVER_BODY_PATH = ROVER_PRIM_PATH + "/Vehicle/rover/Body"

# Ackermann 파라미터 (RLRoverLab aau_rover_simple)
WHEELBASE_LENGTH      = 0.849
MIDDLE_WHEEL_DISTANCE = 0.894
FRONT_REAR_DISTANCE   = 0.77
WHEEL_RADIUS          = 0.1
ACK_OFFSET            = -0.0135
MIN_TURNING_RADIUS    = MIDDLE_WHEEL_DISTANCE * 0.8

DRIVE_WHEEL_ORDER = ("FL", "FR", "ML", "MR", "RL", "RR")
STEER_WHEEL_ORDER = ("FL", "FR", "RL", "RR")


def _find_wheel_joint_indices(dof_names):
    POSITION_ALIASES = {
        "FL": ["FL", "FRONT_LEFT", "LF"],
        "FR": ["FR", "FRONT_RIGHT", "RF"],
        "ML": ["ML", "MID_LEFT", "LM", "CL"],
        "MR": ["MR", "MID_RIGHT", "RM", "CR"],
        "RL": ["RL", "REAR_LEFT", "LR", "BL"],
        "RR": ["RR", "REAR_RIGHT", "BR"],
    }
    drive_map, steer_map = {}, {}
    for idx, name in enumerate(dof_names):
        upper = name.upper()
        wheel_pos = None
        for pos, aliases in POSITION_ALIASES.items():
            if any(a in upper for a in aliases):
                wheel_pos = pos
                break
        if wheel_pos is None:
            continue
        if "STEER" in upper:
            steer_map[wheel_pos] = idx
        elif "DRIVE" in upper or "WHEEL" in upper:
            drive_map[wheel_pos] = idx
    return drive_map, steer_map


def _ackermann(lin_vel, ang_vel):
    """RLRoverLab Ackermann + auto Point-turn (numpy 포트)."""
    d_fr, d_mw, wl, offs = (FRONT_REAR_DISTANCE, MIDDLE_WHEEL_DISTANCE,
                            WHEELBASE_LENGTH, ACK_OFFSET)

    if abs(lin_vel) < 1e-6 and abs(ang_vel) < 1e-6:
        return np.zeros(4), np.zeros(6)

    direction = 1.0 if lin_vel >= 0 else -1.0
    turn_direction = 1.0 if ang_vel >= 0 else (-1.0 if ang_vel < 0 else 0.0)
    lin_abs, ang_abs = abs(lin_vel), abs(ang_vel)
    turning_radius = np.inf if ang_abs == 0 else lin_abs / ang_abs

    if turning_radius < MIN_TURNING_RADIUS:
        v = ang_abs
        wheel_vels = np.array([-v, +v, -v, +v, -v, +v]) * turn_direction
        steer_angs = np.array([-np.pi / 4, +np.pi / 4, +np.pi / 4, -np.pi / 4])
    else:
        r_ML = turning_radius - (d_mw / 2) * turn_direction
        r_MR = turning_radius + (d_mw / 2) * turn_direction
        r_FL = turning_radius - (d_fr / 2) * turn_direction
        r_FR = turning_radius + (d_fr / 2) * turn_direction
        r_RL = turning_radius - (d_fr / 2) * turn_direction
        r_RR = turning_radius + (d_fr / 2) * turn_direction
        wheel_vels = np.array([
            r_FL * ang_abs * direction, r_FR * ang_abs * direction,
            r_ML * ang_abs * direction, r_MR * ang_abs * direction,
            r_RL * ang_abs * direction, r_RR * ang_abs * direction,
        ])
        steer_angs = np.array([
            np.arctan2(wl / 2 - offs, r_FL) *  turn_direction,
            np.arctan2(wl / 2 - offs, r_FR) *  turn_direction,
            np.arctan2(wl / 2 + offs, r_RL) * -turn_direction,
            np.arctan2(wl / 2 + offs, r_RR) * -turn_direction,
        ])
    wheel_vels = wheel_vels / (WHEEL_RADIUS * 2.0)
    return steer_angs, wheel_vels


def _quat_wxyz_to_yaw(q):
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y),
                            1.0 - 2.0 * (y * y + z * z)))


class RoverController:
    def __init__(self, world):
        self._world = world
        self._robot = None
        self._drive_indices = None
        self._steer_indices = None
        self._num_dof = 0
        self._body_off = (0.0, 0.0)   # articulation root → 섀시 강체 오프셋
        self._camera_path = None
        self._camera_vp = None
        self._camera_rp = None

    def spawn(self, initial_position=(0.0, 0.0, 0.0)):
        """로버 USD reference 추가 + 초기 위치 (옵션) 적용.

        Args:
            initial_position: (x, y, z) world 좌표. 울퉁불퉁 지형에선
                              z 를 0.5 정도로 올려서 위에서 떨어뜨리는 게 안전.
        """
        add_reference_to_stage(usd_path=ROVER_USD_PATH, prim_path=ROVER_PRIM_PATH)
        for _ in range(10):
            self._world.app.update() if hasattr(self._world, "app") else None

        # 초기 위치 설정 (existing translate op 있으면 덮어쓰기)
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(ROVER_PRIM_PATH)
        xform = UsdGeom.Xformable(prim)
        translate_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*initial_position))
        print(f"[rover] spawn at {tuple(initial_position)}")

        self._robot = SingleArticulation(prim_path=ARTIC_PRIM_PATH, name="rover")
        return self._robot

    def attach_camera(self, translation=(0.3, 0.0, 0.3),
                      rpy_deg=(90.0, 0.0, -90.0),
                      resolution=(640, 480)):
        stage = omni.usd.get_context().get_stage()
        # vehicle_v1 의 Body 에는 이미 온보드 카메라가 있어 이름을 구분한다.
        self._camera_path = f"{ROVER_BODY_PATH}/OverviewCamera"
        omni.kit.commands.execute("CreatePrim",
                                  prim_path=self._camera_path,
                                  prim_type="Camera")
        cam_prim = stage.GetPrimAtPath(self._camera_path)
        cam_xf = UsdGeom.Xformable(cam_prim)
        cam_xf.ClearXformOpOrder()
        cam_xf.AddTranslateOp().Set(Gf.Vec3d(*translation))
        cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(*rpy_deg))

        # 카메라 뷰포트 생성 — floating 창으로 화면 안쪽에 띄운다.
        # 도킹(deferred_dock_in/dock_in)을 직접 조작하면 ImGui 도킹 트리가
        # 전이 상태에서 렌더돼 크래시한다(ImGui::SetScrollY segfault).
        # create_viewport_window 는 도킹을 하지 않으므로 position 만 지정해
        # floating 으로 띄운다. 기본값 (0,0)은 메뉴바 뒤에 가려진다.
        vp = create_viewport_window(
            "Rover Camera View",
            width=resolution[0], height=resolution[1],
            position_x=100, position_y=120,
            camera_path=self._camera_path,
        )
        self._camera_vp = vp

        # 렌더 프로덕트 (필요 시 annotator 부착용)
        self._camera_rp = rep.create.render_product(self._camera_path, resolution)
        print(f"[rover] 카메라 부착 완료: {self._camera_path} "
              f"(viewport visible={vp.visible})")
        return self._camera_path

    def initialize(self):
        """World.reset() 이후 호출. dof 매핑 + 초기화."""
        self._robot.initialize()
        drive_map, steer_map = _find_wheel_joint_indices(self._robot.dof_names)
        missing_d = [w for w in DRIVE_WHEEL_ORDER if w not in drive_map]
        missing_s = [w for w in STEER_WHEEL_ORDER if w not in steer_map]
        if missing_d or missing_s:
            print(f"[rover] dof_names = {self._robot.dof_names}")
            raise RuntimeError(
                f"휠 조인트 매핑 실패. missing drive={missing_d} steer={missing_s}"
            )
        self._drive_indices = np.array([drive_map[w] for w in DRIVE_WHEEL_ORDER])
        self._steer_indices = np.array([steer_map[w] for w in STEER_WHEEL_ORDER])
        self._num_dof = self._robot.num_dof
        self._measure_body_offset()
        print(f"[rover] 초기화 완료. DOF={self._num_dof}")

    def _measure_body_offset(self):
        """articulation root(m0609/base_link) → 섀시(Body) 강체 오프셋 측정.

        get_world_pose() 는 articulation root 인 m0609/base_link pose 를
        준다. coverage 는 로버 섀시(Body) pose 가 필요하므로 둘의 고정
        오프셋을 USD 정적 변환에서 한 번 재 로버 로컬프레임 (ox, oy) 으로
        저장한다 — mount 가 FixedJoint 라 오프셋 불변. yaw 는 동일하므로
        보정 불필요, XY 만 ~0.17m 차이.
        """
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(ARTIC_PRIM_PATH)
        body = stage.GetPrimAtPath(ROVER_BODY_PATH)
        if not root.IsValid() or not body.IsValid():
            self._body_off = (0.0, 0.0)
            print("[rover] ⚠ Body 오프셋 측정 실패 — articulation root pose 그대로 사용")
            return
        cache = UsdGeom.XformCache()
        rm = cache.GetLocalToWorldTransform(root)
        rt = rm.ExtractTranslation()
        bt = cache.GetLocalToWorldTransform(body).ExtractTranslation()
        q = rm.ExtractRotationQuat()
        iq = q.GetImaginary()
        yaw = _quat_wxyz_to_yaw((q.GetReal(), iq[0], iq[1], iq[2]))
        dxw, dyw = rt[0] - bt[0], rt[1] - bt[1]            # world 프레임 오프셋
        ox = float(np.cos(yaw) * dxw + np.sin(yaw) * dyw)  # 로버 로컬프레임으로
        oy = float(-np.sin(yaw) * dxw + np.cos(yaw) * dyw)
        self._body_off = (ox, oy)
        print(f"[rover] Body 오프셋 측정: 로컬 ({ox:+.3f},{oy:+.3f})m "
              f"— get_pose_2d 가 섀시 기준으로 보정")

    def drive(self, lin_vel, ang_vel):
        steer_angs, wheel_vels = _ackermann(lin_vel, ang_vel)
        vel = np.zeros(self._num_dof)
        pos = np.full(self._num_dof, np.nan)
        vel[self._drive_indices] = wheel_vels
        pos[self._steer_indices] = steer_angs
        self._robot.apply_action(
            ArticulationAction(joint_velocities=vel, joint_positions=pos)
        )

    def get_pose_2d(self):
        """로버 섀시(Body)의 (cx, cy, yaw) 반환. World frame.

        get_world_pose() 는 articulation root(m0609/base_link) pose 라
        섀시 중심과 ~0.17m 어긋난다. _measure_body_offset 가 잰 강체
        오프셋을 yaw 로 회전해 빼서 섀시 pose 로 보정한다.
        """
        pos, ori = self._robot.get_world_pose()
        yaw = _quat_wxyz_to_yaw(ori)
        ox, oy = self._body_off
        cx = float(pos[0]) - (np.cos(yaw) * ox - np.sin(yaw) * oy)
        cy = float(pos[1]) - (np.sin(yaw) * ox + np.cos(yaw) * oy)
        return cx, cy, yaw

    @property
    def camera_path(self):
        return self._camera_path

    @property
    def robot(self):
        return self._robot
