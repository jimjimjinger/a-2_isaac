"""vehicle_v3.usd (액션그래프 내장 로버) 를 terrain 에 올려 구동.

v3 는 ROS2 센서 그래프가 USD 에 내장돼 있다 — 이 런처는 그래프를 짜지 않는다.
terrain 로드 + v3 reference + play 만 한다. 팀 누구든 이 패턴(또는 이 스크립트)
으로 v3 를 띄워 자기 노드를 개발하면 된다 — 실물 로봇처럼.

단일 rover (기존 동작):
    <isaac-python> isaac_sim/scripts/run_vehicle_v3.py [--terrain terrain_00004]

다중 rover (한 맵에 N 대):
    <isaac-python> isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004 \
        --rovers rover_1 rover_2
    각 rover 마다 별개 prim path (/World/<NS>) + ROS 토픽 prefix (/<NS>/...)
    적용. spawn 위치는 meta.json 의 spawn_locations[i] 사용.
"""
import argparse
import json
import os
import shutil
import sys
import tempfile

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

_p = argparse.ArgumentParser(description="vehicle_v3 (graph-embedded rover) 런처")
_p.add_argument("--terrain", default="terrain_00004")
_p.add_argument("--headless", action="store_true")
_p.add_argument("--rovers", nargs="*", default=[],
                help="네임스페이스 list (예: rover_1 rover_2). "
                     "비우면 단일 rover (no namespace, /World/Rover).")
_p.add_argument("--spawn-spacing", type=float, default=0.0,
                help="다중 rover 시 사이 간격 (m). > 0 이면 meta.json spawn_locations "
                     "무시하고 spawn[0] 기준 X 방향으로 N 미터씩 떨어진 자리에 배치. "
                     "A* 회피 검증용 (가까운 spawn 으로 충돌 시나리오 강제).")
_a, _ = _p.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": _a.headless})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
app.update()

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_SIM = os.path.dirname(HERE)
WORLD = f"{ISAAC_SIM}/worlds/{_a.terrain}.usd"
V3 = f"{ISAAC_SIM}/assets/vehicle/vehicle_v3.usd"
TERRAIN_DIR = f"{ISAAC_SIM}/assets/generated_terrains/{_a.terrain}"


# ─── 단일 rover (기존 동작 호환) ──────────────────────────────────────
DEFAULT_ROVER_PRIM = "/World/Rover"


# ─── ScriptNode 소스 — rover root 를 hardcode 한 버전 ────────────────
# build_vehicle_v3.py 의 GT_SCRIPT / GRASP_SCRIPT 와 동일 동작이지만 stage
# 전역 traverse 대신 rover root prim 아래만 검색 → 두 rover 가 같은 stage
# 에 있어도 서로 간섭하지 않음. {ROOT}, {JOINT} 는 rover 별로 다르게 박힘.

GT_SCRIPT_NSAWARE_TPL = '''
from pxr import UsdGeom, Usd
import omni.usd
import omni.graph.core as og


def setup(db):
    pass


def _find_rover_root(node_path):
    """ScriptNode prim path 에서 rover root 추출.
    예: /World/Rover_1/ActionGraph/ReadGtPose → /World/Rover_1
    """
    parts = node_path.split("/")
    if len(parts) < 3:
        return None
    return "/" + parts[1] + "/" + parts[2]


def compute(db):
    stage = omni.usd.get_context().get_stage()
    # 자신 prim path 에서 rover root 동적 추출 — script content 가 공유돼도 인스턴스별 다른 path
    my_path = ""
    try:
        my_path = db.node.get_prim_path()
    except Exception:
        try:
            my_path = str(db.node.get_prim())
        except Exception:
            return False
    rover_root = _find_rover_root(my_path)
    if not rover_root:
        return False
    root_prim = stage.GetPrimAtPath(rover_root)
    if not root_prim or not root_prim.IsValid():
        return False
    artic = None
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI("PhysicsArticulationRootAPI"):
            artic = prim
            break
    if artic is None:
        for prim in Usd.PrimRange(root_prim):
            if prim.GetName() == "base_link" and "m0609" in str(prim.GetPath()):
                artic = prim
                break
    if artic is None:
        return False
    cache = UsdGeom.XformCache()
    M = cache.GetLocalToWorldTransform(artic)
    t = M.ExtractTranslation()
    q = M.ExtractRotationQuat()
    db.outputs.position = [float(t[0]), float(t[1]), float(t[2])]
    qi = q.GetImaginary()
    db.outputs.orientation = [float(qi[0]), float(qi[1]), float(qi[2]),
                              float(q.GetReal())]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''


GRASP_SCRIPT_NSAWARE_TPL = '''
from pxr import UsdPhysics, UsdGeom, Gf, Sdf, Usd
import omni.usd
import omni.graph.core as og
import math

_state = {"attached_joint_path": None, "attached_obj_path": None,
          "gripper_link_path": None}

ROVER_ROOT = "__ROVER_ROOT__"
GRASP_JOINT_PATH = "__GRASP_JOINT_PATH__"
GRIPPER_LINK_NAME = "right_inner_finger"
SEARCH_RADIUS = 1.5


def _component(v, i):
    try:
        return float(v[i])
    except Exception:
        return 0.0


def _find_gripper_link(stage):
    root = stage.GetPrimAtPath(ROVER_ROOT)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == GRIPPER_LINK_NAME and "onrobot" in str(prim.GetPath()):
            return str(prim.GetPath())
    return None


def _find_nearest_mineral(stage, tx, ty):
    cache = UsdGeom.XformCache()
    best_path = None
    best_d2 = SEARCH_RADIUS * SEARCH_RADIUS
    for prim in stage.Traverse():
        parent = prim.GetParent()
        if not parent or parent.GetName() != "Minerals":
            continue
        if not prim.IsValid():
            continue
        imageable = UsdGeom.Imageable(prim)
        try:
            if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
        except Exception:
            pass
        M = cache.GetLocalToWorldTransform(prim)
        p = M.ExtractTranslation()
        dx = p[0] - tx
        dy = p[1] - ty
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best_path = str(prim.GetPath())
    return best_path, math.sqrt(best_d2) if best_path else float("inf")


def _attach(stage, link_path, obj_path):
    if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
        stage.RemovePrim(GRASP_JOINT_PATH)
    link_prim = stage.GetPrimAtPath(link_path)
    obj_prim = stage.GetPrimAtPath(obj_path)
    if not link_prim.IsValid() or not obj_prim.IsValid():
        return False
    joint = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(obj_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return True


def _detach(stage):
    if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
        stage.RemovePrim(GRASP_JOINT_PATH)
    return True


def _set_mineral_collision(stage, obj_path, enabled):
    if not obj_path:
        return
    prim = stage.GetPrimAtPath(obj_path)
    if not prim or not prim.IsValid():
        return
    for child in Usd.PrimRange(prim):
        if child.HasAPI(UsdPhysics.CollisionAPI):
            ca = UsdPhysics.CollisionAPI(child)
            enabled_attr = ca.GetCollisionEnabledAttr()
            if not enabled_attr:
                enabled_attr = ca.CreateCollisionEnabledAttr()
            enabled_attr.Set(bool(enabled))


def _hide(stage, obj_path):
    prim = stage.GetPrimAtPath(obj_path)
    if prim and prim.IsValid():
        imageable = UsdGeom.Imageable(prim)
        imageable.MakeInvisible()


def setup(db):
    pass


def compute(db):
    lin = db.inputs.linearVelocity
    ang = db.inputs.angularVelocity
    mode = _component(ang, 0)  # +1 pickup, -1 release
    if mode > 0.5:
        stage = omni.usd.get_context().get_stage()
        gripper_path = _state.get("gripper_link_path") or _find_gripper_link(stage)
        if not gripper_path:
            print("[grasp] gripper link 못 찾음")
            return False
        _state["gripper_link_path"] = gripper_path
        tx = _component(lin, 0)
        ty = _component(lin, 1)
        mineral_path, dist = _find_nearest_mineral(stage, tx, ty)
        if not mineral_path:
            print(f"[grasp] pickup ignored — no mineral near ({tx:.2f},{ty:.2f}) within {SEARCH_RADIUS}m")
            return True
        if _attach(stage, gripper_path, mineral_path):
            _state["attached_joint_path"] = GRASP_JOINT_PATH
            _state["attached_obj_path"] = mineral_path
            _set_mineral_collision(stage, mineral_path, False)
            print(f"[grasp] pickup OK — attached {mineral_path} to {gripper_path} (target dist {dist:.2f}m, snapped, collision off)")
        return True
    elif mode < -0.5:
        stage = omni.usd.get_context().get_stage()
        obj = _state.get("attached_obj_path")
        _detach(stage)
        if obj:
            _hide(stage, obj)
            print(f"[grasp] release + hide {obj}")
        _state["attached_joint_path"] = None
        _state["attached_obj_path"] = None
        return True
    return True
'''


# ─── helpers ─────────────────────────────────────────────────────────

def _ns_to_prim_name(ns: str) -> str:
    """네임스페이스 string 을 prim 이름으로 정규화 (예: 'rover_1' → 'Rover_1')."""
    s = ns.strip("/").strip()
    if not s:
        return "Rover"
    # Sdf path component 는 영문/숫자/_ 만 안전
    safe = "".join(c if (c.isalnum() or c == "_") else "_" for c in s)
    return safe[:1].upper() + safe[1:]


def _spawn_for(spots, idx, fallback=(0.0, 0.0, 1.0)):
    if not spots or idx >= len(spots):
        return fallback
    s = spots[idx]
    return (float(s["x"]), float(s["y"]), float(s["z"]) + 0.3)


def _close_spawn_for(spots, idx, spacing):
    """검증용 — spawn_locations[0] 기준 X 방향 spacing m 씩 떨어진 자리.

    spawn_locations[0] 의 z 사용 (해당 지점 terrain 높이) 으로 wrong-z 묻힘 방지.
    """
    base = (0.0, 0.0, 1.0)
    if spots:
        s0 = spots[0]
        base = (float(s0["x"]), float(s0["y"]), float(s0["z"]) + 0.3)
    return (base[0] + idx * spacing, base[1], base[2])


def _load_rover(stage, world, prim_path: str, spawn,
                usd_source: str = V3) -> None:
    """vehicle_v3 USD 를 prim_path 에 reference + 위치 설정.

    usd_source 가 V3 가 아니면 (다중 rover 시 per-rover copy) 별도 파일이라
    USD prototype 공유가 일어나지 않아 OmniGraph 데이터 분리 자연 보장.
    """
    add_reference_to_stage(usd_path=usd_source, prim_path=prim_path)
    prim = stage.GetPrimAtPath(prim_path)
    # 안전망 — 혹시라도 instance 화 됐으면 강제 해제
    try:
        prim.SetInstanceable(False)
    except Exception:
        pass
    xf = UsdGeom.Xformable(prim)
    top = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            top = op
            break
    if top is None:
        top = xf.AddTranslateOp()
    top.Set(Gf.Vec3d(*spawn))
    print(f"[run_v3] reference {os.path.basename(usd_source)} @ {prim_path}  spawn={spawn}")


def _per_rover_usd_copies(namespaces) -> dict:
    """rover 별 vehicle_v3 USD 복사본 경로 dict 반환 {ns: usd_path}.

    같은 파일을 여러 prim path 에 reference 하면 USD/OmniGraph 가 prototype
    공유로 데이터가 섞이는 문제 차단 — 각 rover 가 자기 전용 USD 파일을 봄.
    """
    out_dir = tempfile.mkdtemp(prefix="a2_isaac_rovers_")
    copies = {}
    for ns in namespaces:
        dst = os.path.join(out_dir, f"vehicle_v3_{ns}.usd")
        shutil.copyfile(V3, dst)
        copies[ns] = dst
        print(f"[run_v3] per-rover USD copy: {V3.split('/')[-1]} → {dst}")
    return copies


def _patch_topic_names(stage, rover_root: str, ns: str) -> int:
    """rover_root 아래 모든 OmniGraph 노드의 *:topicName 속성에 /{ns} prefix 추가.

    이미 prefix 가 박혀 있으면 (재실행 등) skip. ns 가 비어 있으면 아무것도 안 함.
    return: 변경한 attribute 개수
    """
    if not ns:
        return 0
    ns_norm = ns.strip("/").strip()
    if not ns_norm:
        return 0
    prefix = f"/{ns_norm}"
    root = stage.GetPrimAtPath(rover_root)
    if not root.IsValid():
        return 0
    patched = 0
    for prim in Usd.PrimRange(root):
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if not (name.endswith(":topicName") or name == "topicName"):
                continue
            cur = attr.Get()
            if not isinstance(cur, str) or not cur:
                continue
            # 이미 prefix 적용돼 있으면 skip
            if cur.startswith(prefix + "/") or cur == prefix:
                continue
            # leading slash 보장
            if not cur.startswith("/"):
                cur = "/" + cur
            new_topic = prefix + cur
            attr.Set(new_topic)
            patched += 1
    return patched


def _patch_script_nodes(stage, rover_root: str, ns: str) -> int:
    """rover_root 아래 GT/GRASP ScriptNode 의 inputs:script 를 rover-scoped 버전으로 교체.

    return: 패치한 ScriptNode 개수
    """
    ns_norm = (ns.strip("/").strip() or "rover")
    joint_path = f"/World/grip_fixed_joint_{ns_norm}"
    root = stage.GetPrimAtPath(rover_root)
    if not root.IsValid():
        return 0
    patched = 0
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "OmniGraphNode":
            continue
        name = prim.GetName()
        script_attr = prim.GetAttribute("inputs:script")
        if not script_attr or not script_attr.IsValid():
            continue
        if name == "ReadGtPose":
            # GT_SCRIPT 는 self-introspection (placeholder 없음). 모든 rover 동일 content,
            # 런타임에 자기 prim path 로 rover root 동적 결정.
            new_src = GT_SCRIPT_NSAWARE_TPL
            script_attr.Set(new_src)
            check = script_attr.Get()
            ok = (isinstance(check, str) and "_find_rover_root" in check)
            print(f"[run_v3]     {script_attr.GetPath()} ← ReadGtPose ({len(new_src)} chars, self-introspect={'OK' if ok else 'FAIL'})")
            patched += 1
        elif name == "GraspScript":
            new_src = (GRASP_SCRIPT_NSAWARE_TPL
                       .replace("__ROVER_ROOT__", rover_root)
                       .replace("__GRASP_JOINT_PATH__", joint_path))
            script_attr.Set(new_src)
            check = script_attr.Get()
            ok = (isinstance(check, str) and rover_root in check and joint_path in check)
            print(f"[run_v3]     {script_attr.GetPath()} ← GraspScript ({len(new_src)} chars, ROVER_ROOT+JOINT={'OK' if ok else 'FAIL'})")
            patched += 1
    return patched


def main() -> None:
    for f in (WORLD, V3):
        if not os.path.isfile(f):
            print(f"[run_v3] ✗ 파일 없음: {f}")
            app.close()
            sys.exit(1)

    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=WORLD, prim_path="/World/MarsScene")
    print(f"[run_v3] 씬 로드: {WORLD}")

    # spawn 좌표 (terrain meta.json)
    spots = []
    meta = os.path.join(TERRAIN_DIR, "meta.json")
    if os.path.isfile(meta):
        with open(meta) as f:
            spots = json.load(f).get("spawn_locations") or []

    stage = omni.usd.get_context().get_stage()

    if not _a.rovers:
        # ── 단일 rover (기존 동작) ──
        spawn = _spawn_for(spots, 0)
        _load_rover(stage, world, DEFAULT_ROVER_PRIM, spawn)
    else:
        # ── 다중 rover ──
        # 핵심: 같은 USD 를 여러 prim path 에 reference 하면 USD/OmniGraph 가
        # prototype 공유 → 데이터 섞임. 각 rover 별 USD 사본을 만들어 reference.
        usd_copies = _per_rover_usd_copies(_a.rovers)
        spacing = float(_a.spawn_spacing)
        if spacing > 0.0:
            print(f"[run_v3] close-spawn 모드: 간격 {spacing:.2f}m (A* 회피 검증)")
        for i, ns in enumerate(_a.rovers):
            prim_path = f"/World/{_ns_to_prim_name(ns)}"
            if spacing > 0.0:
                spawn = _close_spawn_for(spots, i, spacing)
            else:
                spawn = _spawn_for(spots, i, fallback=(i * 3.0, 0.0, 1.0))
            _load_rover(stage, world, prim_path, spawn, usd_source=usd_copies[ns])

            # USD reference 가 stage 에 반영될 시간을 주고 patch
            for _ in range(5):
                app.update()
            n_topics = _patch_topic_names(stage, prim_path, ns)
            n_scripts = _patch_script_nodes(stage, prim_path, ns)
            print(f"[run_v3]   patched {n_topics} topicName attrs, "
                  f"{n_scripts} ScriptNode(s) → namespace /{ns}")

    for _ in range(20):
        app.update()
    world.reset()
    world.play()

    # ─── 진단: 각 rover 의 articulation root 가 실제 어디 있는지 dump ───
    if _a.rovers:
        for _ in range(10):
            app.update()
        print("[run_v3] === 다중 rover 진단 ===")
        for ns in _a.rovers:
            prim_path = f"/World/{_ns_to_prim_name(ns)}"
            root = stage.GetPrimAtPath(prim_path)
            artic_paths = []
            for prim in Usd.PrimRange(root):
                if prim.HasAPI("PhysicsArticulationRootAPI"):
                    artic_paths.append(str(prim.GetPath()))
            cache = UsdGeom.XformCache()
            poses = []
            for ap in artic_paths:
                p = stage.GetPrimAtPath(ap)
                M = cache.GetLocalToWorldTransform(p)
                t = M.ExtractTranslation()
                poses.append((ap, (float(t[0]), float(t[1]), float(t[2]))))
            # 부모 prim_path 의 xform translate 도 확인
            xf = UsdGeom.Xformable(root)
            tops = []
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    v = op.Get()
                    tops.append((float(v[0]), float(v[1]), float(v[2])))
            print(f"[run_v3]   {ns}: parent {prim_path} translate ops={tops}")
            for ap, (x, y, z) in poses:
                print(f"[run_v3]     articulation {ap} world pose=({x:.2f}, {y:.2f}, {z:.2f})")

    if not _a.rovers:
        print("[run_v3] ready — v3 내장 Action Graph 가 센서 토픽 발행 중 "
              "(/imu/data /joint_states_raw /camera/*)")
    else:
        nss = ", ".join(f"/{n}" for n in _a.rovers)
        print(f"[run_v3] ready — {len(_a.rovers)}대 vehicle namespaces: {nss}")

    step = 0
    try:
        while app.is_running():
            world.step(render=True)
            if step % 600 == 0:
                print(f"[run_v3] running... step {step}")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    main()
