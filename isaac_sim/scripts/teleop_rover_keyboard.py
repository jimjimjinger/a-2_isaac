#!/usr/bin/env python3
"""키보드 teleop 으로 로버를 몰며 맵경계 충돌벽을 검증하는 테스트 도구.

mars_exploration_world.usd(지형 + 경계 충돌벽 + rocks + 조명)를 띄우고 로버를
spawn 한 뒤, 키보드로 직접 몰아 50×50 m 아레나 경계의 투명 충돌벽이 로버를
제대로 막는지 눈으로 확인한다. 로버 위치를 주기적으로 출력하므로 벽에 닿아
멈추는지(낙하 안 하는지)를 수치로도 검증할 수 있다.

조작 (Isaac Sim 뷰포트를 클릭해 포커스를 준 상태에서):
    W / ↑   전진           S / ↓   후진
    A / ←   좌회전         D / →   우회전
    Space   정지           Q / Esc 종료
회전은 skid-steer(좌/우 휠 속도차)로 구현 — 정밀 주행이 아닌 충돌 확인용.

이 스크립트는 테스트용이라 isaac_sim/scripts/ 에 둔다 (ROS2 패키지에 설치되지
않는 개발 도구 영역). 팔(M0609)·그리퍼는 충돌 검증과 무관하므로 로버만 spawn.

실행 (Isaac Sim 파이썬으로 — GUI 필요):
    isaac-python isaac_sim/scripts/teleop_rover_keyboard.py
    isaac-python isaac_sim/scripts/teleop_rover_keyboard.py --spawn-x 18 --speed 8
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# 스크립트 위치 기준으로 워크스페이스 루트 유도 (절대경로 하드코딩 금지).
# 이 파일은 <A2_ROOT>/isaac_sim/scripts/teleop_rover_keyboard.py.
_SCRIPT_PATH = Path(__file__).resolve()
A2_ROOT = _SCRIPT_PATH.parents[2]
DEFAULT_WORLD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
ROVER_USD = A2_ROOT / "isaac_sim/assets/rover/Mars_Rover.usd"

# ─── 50 m 아레나 상수 (I1 규약) — 낙하/통과 판정용 ──────────────────────────
ARENA_HALF = 25.0           # 경계는 x/y = ±25 m
BREACH_MARGIN = 0.8         # 중심이 ±25.8 m 넘으면 '벽 통과'로 간주
FALL_Z = -8.0               # z 가 이보다 낮으면 '맵 밖 낙하'로 간주


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="키보드 teleop 로버 — 맵경계 충돌벽 검증")
    ap.add_argument("--world", default=str(DEFAULT_WORLD),
                    help="로드할 master world USD (기본: mars_exploration_world.usd)")
    ap.add_argument("--speed", type=float, default=6.0,
                    help="전/후진 휠 각속도 (rad/s)")
    ap.add_argument("--turn", type=float, default=3.0,
                    help="회전 시 좌/우 휠 속도차 (rad/s)")
    ap.add_argument("--spawn-x", type=float, default=5.0)
    ap.add_argument("--spawn-y", type=float, default=0.0)
    ap.add_argument("--spawn-z", type=float, default=2.0,
                    help="spawn 높이 (절대 z) — 지형 위로 자유낙하시켜 안착")
    return ap.parse_args()


ARGS = _parse_args()

# SimulationApp 은 다른 omniverse import 보다 먼저. teleop 은 키보드 입력을
# 받아야 하므로 GUI 필수 → headless=False.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import carb
import carb.input
import omni.appwindow
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


# ─── 휠/조향 조인트 이름 판별 (build_rover_m0609_scene.py 와 동일 규칙) ──────
_LEFT_CODES = ("FL", "CL", "ML", "RL")
_RIGHT_CODES = ("FR", "CR", "MR", "RR")
_ALL_CODES = _LEFT_CODES + _RIGHT_CODES


def _is_drive_joint(name: str) -> bool:
    u = name.upper()
    return "DRIVE" in u and "STEER" not in u and any(c in u for c in _ALL_CODES)


def _is_steer_joint(name: str) -> bool:
    u = name.upper()
    return "STEER" in u and any(c in u for c in _ALL_CODES)


def _wheel_side(name: str):
    """드라이브 휠이 좌(L)/우(R) 중 어느 쪽인지. 미분류면 None."""
    u = name.upper()
    if any(c in u for c in _LEFT_CODES):
        return "L"
    if any(c in u for c in _RIGHT_CODES):
        return "R"
    return None


# ─── USD 헬퍼 ───────────────────────────────────────────────────────────────
def _find_prim_by_name(root_prim, name: str):
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return prim
    return None


def _find_articulation_root(stage, root_path: str) -> str:
    """로버 subtree 에서 ArticulationRootAPI 를 가진 prim 경로를 찾는다."""
    root = stage.GetPrimAtPath(root_path)
    if root.IsValid():
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(prim.GetPath())
    return root_path


def _configure_drives(rover_prim):
    """휠 = 속도 제어 모드(stiffness 0, damping 큼), 조향 = 0(직진) 위치 고정.
    skid-steer teleop 은 휠 속도차로만 회전하므로 조향축은 직진 잠금."""
    n_drive = n_steer = 0
    for prim in Usd.PrimRange(rover_prim):
        name = prim.GetName()
        drv = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drv:
            continue
        if _is_drive_joint(name):
            drv.GetTargetPositionAttr().Set(0.0)
            drv.GetTargetVelocityAttr().Set(0.0)
            drv.GetStiffnessAttr().Set(0.0)
            drv.GetDampingAttr().Set(1e5)
            drv.GetMaxForceAttr().Set(1e7)
            n_drive += 1
        elif _is_steer_joint(name):
            drv.GetTargetPositionAttr().Set(0.0)
            drv.GetTargetVelocityAttr().Set(0.0)
            drv.GetStiffnessAttr().Set(1e8)
            drv.GetDampingAttr().Set(1e6)
            drv.GetMaxForceAttr().Set(1e7)
            n_steer += 1
    return n_drive, n_steer


# ─── 키보드 입력 ────────────────────────────────────────────────────────────
class Keyboard:
    """눌린 키 집합을 유지한다 (press 에 추가 / release 에 제거 → 길게 누름 지원)."""

    def __init__(self):
        self.pressed: set[str] = set()
        app_window = omni.appwindow.get_default_app_window()
        self._keyboard = app_window.get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._sub = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_event)

    def _on_event(self, event, *_args) -> bool:
        name = event.input.name        # 예: "W", "UP", "SPACE", "ESCAPE"
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self.pressed.add(name)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.pressed.discard(name)
        return True


# ─── 씬 구축 ────────────────────────────────────────────────────────────────
def build_scene(stage, world_usd: Path, spawn_pos):
    print(f"\n[1/3] master world 로드 … {world_usd.name}")
    if not world_usd.exists():
        raise FileNotFoundError(world_usd)
    mars = stage.DefinePrim("/World/Mars", "Xform")
    mars.GetReferences().AddReference(str(world_usd))
    for _ in range(10):
        simulation_app.update()

    # PhysicsScene — master world 엔 없음. 화성 중력 3.72 m/s².
    if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(3.72)
        print("  [PhysX] 화성 중력 3.72 m/s² scene 추가")

    # 지형 메시 collider 보강 — v2 terrain 은 이미 baked, 방어적으로 재확인.
    # (경계벽 4면도 terrain_only.usd 안에 collider 포함되어 함께 로드됨.)
    for path in ("/World/Mars/Terrain/TerrainMesh", "/World/Terrain/TerrainMesh"):
        tm = stage.GetPrimAtPath(path)
        if tm.IsValid():
            if not tm.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(tm)
            if not tm.HasAPI(UsdPhysics.MeshCollisionAPI):
                UsdPhysics.MeshCollisionAPI.Apply(tm)
            print(f"  [PhysX] 지형 메시 collider 확인 → {path}")
            break

    print(f"[2/3] 로버 spawn … {ROVER_USD.name} @ {tuple(round(v, 2) for v in spawn_pos)}")
    if not ROVER_USD.exists():
        raise FileNotFoundError(ROVER_USD)
    stage.DefinePrim("/World/Vehicle", "Xform")
    rover = stage.DefinePrim("/World/Vehicle/rover", "Xform")
    rover.GetReferences().AddReference(str(ROVER_USD))
    xf = UsdGeom.Xformable(rover)
    xf.ClearXformOpOrder()
    for op in ("xformOp:translate", "xformOp:orient", "xformOp:scale",
               "xformOp:rotateXYZ", "xformOp:rotateZYX"):
        if rover.GetAttribute(op).IsValid():
            rover.RemoveProperty(op)
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*spawn_pos))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
    for _ in range(10):
        simulation_app.update()

    n_drive, n_steer = _configure_drives(rover)
    print(f"  [Drive] 휠 {n_drive}개 속도모드 / 조향 {n_steer}개 직진잠금")
    return rover


def main() -> int:
    world_usd = Path(ARGS.world).expanduser()
    spawn_pos = (ARGS.spawn_x, ARGS.spawn_y, ARGS.spawn_z)

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    rover_prim = build_scene(stage, world_usd, spawn_pos)
    rover_path = str(rover_prim.GetPath())

    print("[3/3] 물리 초기화 …")
    world.reset()

    artic_root = _find_articulation_root(stage, rover_path)
    rover_art = SingleArticulation(prim_path=artic_root, name="teleop_rover")
    rover_art.initialize()
    dof_names = list(rover_art.dof_names)

    left_idx, right_idx, other_drive_idx = [], [], []
    for i, name in enumerate(dof_names):
        if not _is_drive_joint(name):
            continue
        side = _wheel_side(name)
        (left_idx if side == "L" else right_idx if side == "R"
         else other_drive_idx).append(i)
    left_idx = np.array(left_idx, dtype=np.int32)
    right_idx = np.array(right_idx, dtype=np.int32)
    other_drive_idx = np.array(other_drive_idx, dtype=np.int32)
    print(f"  [Drive] articulation={artic_root}  DOF={len(dof_names)}  "
          f"휠 좌{len(left_idx)}/우{len(right_idx)}"
          + (f"/미분류{len(other_drive_idx)}" if len(other_drive_idx) else ""))

    if len(left_idx) == 0 and len(right_idx) == 0:
        print("  ⚠️ 드라이브 휠 조인트를 못 찾음 — 조인트 이름 확인 필요:")
        for i, n in enumerate(dof_names):
            print(f"     {i}: {n}")

    body = _find_prim_by_name(rover_prim, "Body") or rover_prim
    keyboard = Keyboard()
    world.play()

    speed, turn = ARGS.speed, ARGS.turn
    # 순간 반전 시 PhysX가 불안정해질 수 있어서 teleop 도 가속 기울기를 제한한다.
    cur_v = 0.0
    cur_w = 0.0
    linear_accel = 4.0   # m/s^2
    angular_accel = 3.0   # rad/s^2
    last_cmd_t = time.monotonic()
    print("\n" + "=" * 64)
    print("  키보드 teleop 준비 완료 — 뷰포트를 클릭해 포커스를 주세요.")
    print("  W/↑ 전진   S/↓ 후진   A/← 좌회전   D/→ 우회전")
    print("  Space 정지   Q/Esc 종료")
    print(f"  경계벽: x/y = ±{ARENA_HALF:.0f} m  →  벽으로 몰아 낙하 여부 확인")
    print("=" * 64 + "\n")

    step = 0
    verdict_printed = False
    while simulation_app.is_running():
        keys = keyboard.pressed
        if "ESCAPE" in keys or "Q" in keys:
            print("[teleop] 종료 요청 — 시뮬레이션 정지")
            break

        # skid-steer 믹싱: v=전후, w=회전 → 좌/우 휠 속도.
        target_v = (speed if keys & {"W", "UP"} else 0.0) \
            - (speed if keys & {"S", "DOWN"} else 0.0)
        target_w = (turn if keys & {"A", "LEFT"} else 0.0) \
            - (turn if keys & {"D", "RIGHT"} else 0.0)
        if "SPACE" in keys:
            target_v = 0.0
            target_w = 0.0

        now = time.monotonic()
        dt = min(max(now - last_cmd_t, 0.0), 0.05)
        last_cmd_t = now
        max_dv = linear_accel * dt
        max_dw = angular_accel * dt
        cur_v += max(-max_dv, min(max_dv, target_v - cur_v))
        cur_w += max(-max_dw, min(max_dw, target_w - cur_w))
        if target_v == 0.0 and abs(cur_v) < 0.05:
            cur_v = 0.0
        if target_w == 0.0 and abs(cur_w) < 0.05:
            cur_w = 0.0

        v_left, v_right = cur_v - cur_w, cur_v + cur_w

        jv = np.zeros(rover_art.num_dof, dtype=np.float32)
        if len(left_idx):
            jv[left_idx] = v_left
        if len(right_idx):
            jv[right_idx] = v_right
        if len(other_drive_idx):
            jv[other_drive_idx] = cur_v      # 미분류 휠은 직진 성분만
        rover_art.apply_action(ArticulationAction(joint_velocities=jv))

        world.step(render=True)
        step += 1

        # 1초(≈60스텝)마다 로버 위치 출력 + 낙하/통과 자동 판정.
        if step % 60 == 0:
            t = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            x, y, z = float(t[0]), float(t[1]), float(t[2])
            tag = ""
            if z < FALL_Z:
                tag = "  ⚠️ 맵 밖 낙하 감지 — 경계벽이 막지 못함!"
            elif abs(x) > ARENA_HALF + BREACH_MARGIN \
                    or abs(y) > ARENA_HALF + BREACH_MARGIN:
                tag = "  ⚠️ 경계 통과 — 벽을 넘어감!"
            elif (abs(x) > ARENA_HALF - 1.5 or abs(y) > ARENA_HALF - 1.5):
                tag = "  ← 경계벽 근처 (충돌 검증 지점)"
            print(f"[pos] x={x:+7.2f}  y={y:+7.2f}  z={z:+6.2f}{tag}")
            if tag.startswith("  ⚠️") and not verdict_printed:
                verdict_printed = True
                print("[teleop] 경계벽 검증 실패 정황 — 위 좌표 확인 필요.")

    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
