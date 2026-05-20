"""Mars exploration world + AAU Mars Rover + Doosan M0609 + OnRobot RG2 (단일 articulation).

기존 rover_pick_place 패턴([[rover-pick-place-setup]])을 a2_isaac 워크스페이스로
이식. RobotAssembler 로 두 articulation(rover + M0609) 을 결합해 PhysX 가
일관된 단일 articulation 으로 시뮬하도록 함.

T1(김현중) 의 화성 환경(mars_exploration_world.usd) 위에 T2 가 책임지는
manipulator 자산(rover+M0609+RG2)을 spawn.

사용:
    isaac-python scripts/build_rover_m0609_scene.py
    isaac-python scripts/build_rover_m0609_scene.py --auto-play
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile

# stdout line-buffered → print 가 즉시 flush 되도록 (tee 캡처용)
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# URDF importer 가 임시 mesh USD 를 cwd 에 쓰려고 함 → cwd 가 read-only 면 실패
# (예: /opt/ove/base_stack 같은 곳에서 실행 시 권한 에러). 임시 쓰기 가능
# 디렉토리로 chdir 한 뒤 SimulationApp 시작.
_TMP_CWD = tempfile.mkdtemp(prefix="isaac_urdf_")
os.chdir(_TMP_CWD)
print(f"[init] chdir → {_TMP_CWD}  (URDF temp mesh staging)", flush=True)

# SimulationApp 은 다른 omniverse import 보다 먼저 와야 함.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import sys
from pathlib import Path

import numpy as np
import omni.kit.commands
import omni.kit.app
import omni.usd
import carb
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

# 시끄러운 로그 silence
_carb = carb.settings.get_settings()
for ch in (
    "/log/channels/isaacsim.core.simulation_manager.plugin",
    "/log/channels/omni.physx.tensors.plugin",
    "/log/channels/omni.hydra.scene_delegate",
    "/log/channels/omni.usd",
):
    try:
        _carb.set(ch, "Error")
    except Exception:
        pass

# RobotAssembler 확장 활성화
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot_setup.assembler", True
)

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.robot_setup.assembler import RobotAssembler


# ─── 경로 ──────────────────────────────────────────────────────────────────
A2_ROOT = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac")
MARS_WORLD_USD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
# T1 김현중이 main 에 추가한 in-repo rover 자산 (2026-05-20).
ROVER_USD = A2_ROOT / "isaac_sim/assets/rover/Mars_Rover.usd"

# M0609 + RG2 도 in-repo 로 이전 (2026-05-20). m0609 URDF 의 mesh absolute path
# 는 sed 로 상대경로(meshes/...) 패치됨. 자가포함 → 다른 PC 에서도 그대로 작동.
M0609_URDF = A2_ROOT / "isaac_sim/assets/doosan-robot2/urdf/m0609_isaac_sim.urdf"
RG2_URDF = A2_ROOT / "isaac_sim/assets/onrobot_rg2/urdf/onrobot_rg2.urdf"

# URDF 안의 link 이름 (dual_cam_pick_place config 와 동일)
M0609_EE_LINK = "link_6"
RG2_BASE_LINK = "angle_bracket"

# ─── Spawn 위치 ─────────────────────────────────────────────────────────────
# 이전 (0,-6) 는 hill 옆이라 rover 기울어짐. 다른 방향으로 이동.
# terrain 은 procedural 이라 정확한 평지 좌표 모름 → 사용자가 본 결과 따라
# 아래 SPAWN_X / SPAWN_Y 만 바꿔서 빠르게 iterate.
SPAWN_X = 5.0    # 동쪽 5m
SPAWN_Y = 0.0
SPAWN_Z = 1.0    # 자유낙하 거리 확보용
ROVER_SPAWN_POS = (SPAWN_X, SPAWN_Y, SPAWN_Z)
ROVER_SPAWN_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)

# M0609 mount offsets from rover origin (사용자가 GUI 에서 시각 확인 후 정한 값,
# 2026-05-20). rover 위에 딱 붙는 자세: world Z=1.21232 → offset Z=0.21232.
M0609_MOUNT_OFFSET_X = 0.15274
M0609_MOUNT_OFFSET_Y = 0.0
M0609_MOUNT_OFFSET_Z = 0.21232
M0609_BASE_POS = (
    SPAWN_X + M0609_MOUNT_OFFSET_X,
    SPAWN_Y + M0609_MOUNT_OFFSET_Y,
    SPAWN_Z + M0609_MOUNT_OFFSET_Z,
)


# ─── URDF / Assembler 헬퍼 ───────────────────────────────────────────────────
def _import_urdf(urdf_path: str, fix_base: bool) -> str:
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(urdf_path)
    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = True
    import_config.import_inertia_tensor = True
    import_config.fix_base = fix_base
    import_config.distance_scale = 1.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 1e10
    import_config.default_position_drive_damping = 1e5
    _, artic_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    if artic_path is None:
        raise RuntimeError(f"URDF import 실패: {urdf_path}")
    robot_root = artic_path.rsplit("/", 1)[0] or artic_path
    print(f"  [URDF] {urdf_path}\n         articulation={artic_path}  root={robot_root}")
    return robot_root


def _find_prim_path_by_name(root_path: str, link_name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == link_name:
            return str(prim.GetPath())
    return None


def _assemble(robot_base, robot_base_mount, robot_attach, robot_attach_mount,
              namespace, variant):
    """RobotAssembler 로 두 robot 을 fixed joint 로 결합.
    mount alignment 기준: attach 의 mount 가 base 의 mount 위치로 정렬됨.
    ⚠️ begin_assembly 가 EditTarget 을 sublayer 로 바꾸는데 finish_assemble 가
    원복하지 않음. 호출 후 root layer 로 명시적 복구해서 다음 DefinePrim 등이
    유효한 spec 으로 가도록.
    """
    stage = omni.usd.get_context().get_stage()
    root_edit_target = Usd.EditTarget(stage.GetRootLayer())
    assembler = RobotAssembler()
    assembler.begin_assembly(
        stage, robot_base, robot_base_mount, robot_attach, robot_attach_mount,
        namespace, variant,
    )
    assembler.assemble()
    assembler.finish_assemble()
    stage.SetEditTarget(root_edit_target)


def _paint_subtree_dark(root_path: str, rgb=(0.08, 0.08, 0.08),
                         roughness: float = 0.6, metallic: float = 0.3):
    """root_path 하위의 시각을 강제로 어두운 색으로 paint.
    (1) 모든 Shader 의 Color3f inputs 를 dark 로 override (입력 이름 무관)
    (2) 기존 material:binding relationship 제거 후 우리 새 dark material 로 재바인딩
    """
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        carb.log_warn(f"[T2-DEBUG] _paint_subtree_dark: root invalid {root_path}")
        return 0

    rgb_vec = Gf.Vec3f(*rgb)

    # 디버그: subtree 의 모든 prim 타입 카운트 (왜 Mesh 가 0개인지 진단)
    type_counts = {}
    all_predicate = Usd.PrimAllPrimsPredicate
    for prim in Usd.PrimRange(root, predicate=all_predicate):
        t = str(prim.GetTypeName()) or "<empty>"
        type_counts[t] = type_counts.get(t, 0) + 1
    top_types = sorted(type_counts.items(), key=lambda x: -x[1])[:15]
    carb.log_warn(f"[T2-DEBUG] _paint_subtree_dark prim type counts: {top_types}")

    # (1) 모든 Shader prim 의 Color3f inputs 를 dark 로 set (이름 무관)
    modified = 0
    for prim in Usd.PrimRange(root, predicate=all_predicate):
        if prim.GetTypeName() != "Shader":
            continue
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if not name.startswith("inputs:"):
                continue
            if attr.GetTypeName() not in (Sdf.ValueTypeNames.Color3f,
                                          Sdf.ValueTypeNames.Float3):
                continue
            # emissive/specular 같은 건 검정 두면 너무 어두워질 수 있어 skip
            lower = name.lower()
            if "emissive" in lower or "specular" in lower or "reflection" in lower:
                continue
            try:
                attr.Set(rgb_vec)
                modified += 1
            except Exception:
                pass
    carb.log_warn(f"[T2-DEBUG] _paint_subtree_dark: modified {modified} Color3f shader inputs")

    # (2) 새 PreviewSurface material 생성
    mat_path = f"{root_path}/Looks/T2DarkBody"
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(rgb_vec)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # (3) Mesh / GeomSubset / Xform-with-material-binding 의 binding 재설정.
    #     URDF in-memory stage 의 prim 타입이 Mesh 가 아닐 수 있어 binding 있는
    #     모든 prim 을 후보로.
    bound = 0
    for prim in Usd.PrimRange(root, predicate=all_predicate):
        p = str(prim.GetPath())
        if "/collisions" in p or "/Looks" in p:
            continue
        # material:binding 이 있거나, Mesh/GeomSubset 인 prim 만 처리
        has_binding = any(
            prim.HasRelationship(rn) for rn in (
                "material:binding", "material:binding:allPurpose",
                "material:binding:preview", "material:binding:full",
            )
        )
        if not has_binding and prim.GetTypeName() not in ("Mesh", "GeomSubset"):
            continue
        # 기존 binding 제거
        for rel_name in ("material:binding", "material:binding:allPurpose",
                         "material:binding:preview", "material:binding:full"):
            if prim.HasRelationship(rel_name):
                prim.RemoveProperty(rel_name)
        # 우리 새 dark material 바인딩
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )
        bound += 1
    carb.log_warn(f"[T2-DEBUG] _paint_subtree_dark: rebound {bound} prims (Mesh/GeomSubset/with-binding)")
    return modified + bound


def _freeze_rover_drives(rover_prim) -> int:
    """rover 의 모든 drive 를 freeze: target=0, stiffness/damping 매우 큼.
    휠/조향이 멋대로 돌지 않게."""
    n = 0
    for prim in Usd.PrimRange(rover_prim):
        for drv_type in ("angular", "linear"):
            drv = UsdPhysics.DriveAPI.Get(prim, drv_type)
            if drv:
                drv.GetTargetPositionAttr().Set(0.0)
                drv.GetTargetVelocityAttr().Set(0.0)
                drv.GetStiffnessAttr().Set(1e8)
                drv.GetDampingAttr().Set(1e6)
                drv.GetMaxForceAttr().Set(1e7)
                n += 1
    return n


# ─── Scene 구축 ─────────────────────────────────────────────────────────────
def build_scene():
    stage = omni.usd.get_context().get_stage()

    # ① mars_exploration_world.usd 를 reference 로 로드 (T1 환경)
    print("\n[1/5] Loading mars_exploration_world.usd …")
    if not MARS_WORLD_USD.exists():
        raise FileNotFoundError(MARS_WORLD_USD)
    mars_prim = stage.DefinePrim("/World/Mars", "Xform")
    mars_prim.GetReferences().AddReference(str(MARS_WORLD_USD))
    for _ in range(10):
        simulation_app.update()

    # PhysicsScene 추가 (mars world 에 없음). Mars gravity 3.72.
    physics_scene_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(physics_scene_path).IsValid():
        scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(3.72)
        print("  [PhysX] Mars gravity 3.72 m/s² scene 추가")

    # Terrain mesh 에 collision 보강 (mars world 원본에 없음).
    # ⚠️ mars world 의 defaultPrim=/World 인데 우리는 /World/Mars 로 reference
    # 하므로 내부 path 는 /World/Mars/Terrain/TerrainMesh (World 중복 없음).
    terrain_mesh_path = "/World/Mars/Terrain/TerrainMesh"
    tm = stage.GetPrimAtPath(terrain_mesh_path)
    if tm.IsValid():
        if not tm.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(tm)
        if not tm.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(tm)
        mca = UsdPhysics.MeshCollisionAPI(tm)
        approx = mca.GetApproximationAttr() or mca.CreateApproximationAttr()
        approx.Set("meshSimplification")
        print(f"  [PhysX] terrain mesh collision 보강 → {terrain_mesh_path}")
    else:
        print(f"  ⚠️ terrain mesh prim not found at {terrain_mesh_path}")
        # 디버그: /World/Mars 자식 출력
        mars_prim = stage.GetPrimAtPath("/World/Mars")
        if mars_prim.IsValid():
            print("  /World/Mars children:")
            for child in mars_prim.GetChildren():
                print(f"    {child.GetPath()}")

    # ② rover_instance 대신 Mars_Rover.usd reference (T2 가 선택한 detailed 모델)
    print("\n[2/5] Adding Mars_Rover.usd …")
    rover_prim_path = "/World/Vehicle/rover"
    stage.DefinePrim("/World/Vehicle", "Xform")
    rover_prim = stage.DefinePrim(rover_prim_path, "Xform")
    rover_prim.GetReferences().AddReference(str(ROVER_USD))
    xform = UsdGeom.Xformable(rover_prim)
    xform.ClearXformOpOrder()
    for op_attr in ("xformOp:translate", "xformOp:orient", "xformOp:scale",
                    "xformOp:rotateXYZ", "xformOp:rotateZYX",
                    "xformOp:rotateXZY", "xformOp:rotateYXZ"):
        if rover_prim.GetAttribute(op_attr).IsValid():
            rover_prim.RemoveProperty(op_attr)
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*ROVER_SPAWN_POS))
    q = ROVER_SPAWN_QUAT_WXYZ
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    )
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
    for _ in range(10):
        simulation_app.update()
    n = _freeze_rover_drives(rover_prim)
    print(f"  [Scene] rover @ {ROVER_SPAWN_POS}  (drives frozen: {n})")

    # ③ M0609 URDF import (fix_base=False → floating-base).
    carb.log_warn(f"[T2-DEBUG] === STEP 3: M0609 URDF import ===")
    carb.log_warn(f"[T2-DEBUG] CONFIG M0609_BASE_POS = {M0609_BASE_POS}")
    carb.log_warn(f"[T2-DEBUG] CONFIG M0609_MOUNT_OFFSET_Z = {M0609_MOUNT_OFFSET_Z}")
    robot_root = _import_urdf(str(M0609_URDF), fix_base=False)

    # DEBUG: import 직후 실제 prim 구조 확인
    m0609_root_prim = stage.GetPrimAtPath(robot_root)
    carb.log_warn(f"[T2-DEBUG] robot_root = {robot_root}  valid={m0609_root_prim.IsValid()}  type={m0609_root_prim.GetTypeName()}")
    carb.log_warn(f"[T2-DEBUG] robot_root APIs: {m0609_root_prim.GetAppliedSchemas()}")
    carb.log_warn(f"[T2-DEBUG] robot_root children: {[c.GetName() for c in m0609_root_prim.GetChildren()][:10]}")
    # 어느 prim 이 ArticulationRootAPI 가지고 있나
    for prim in Usd.PrimRange(m0609_root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            carb.log_warn(f"[T2-DEBUG] ArticulationRoot prim: {prim.GetPath()}")

    # import 직후 base_link world pos
    bl_path = _find_prim_path_by_name(robot_root, "base_link")
    if bl_path:
        bl = stage.GetPrimAtPath(bl_path)
        bl_world = UsdGeom.Xformable(bl).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        carb.log_warn(f"[T2-DEBUG] base_link world BEFORE translate: ({bl_world[0]:+.3f}, {bl_world[1]:+.3f}, {bl_world[2]:+.3f})")

    # URDF importer 가 만든 articulation root prim 을 옮긴다.
    # ⚠️ 핵심: USD xformOp 만 직접 수정하면 USD prim 은 옮겨지지만 PhysX 내부
    # rigid body pose 는 안 바뀜 (GUI 표시와 시각 위치 분리). GUI 의 transform
    # gizmo 가 사용하는 'TransformPrimSRTCommand' 를 통해 set 해야 양쪽 sync.
    omni.kit.commands.execute(
        "TransformPrimSRTCommand",
        path=robot_root,
        new_translation=Gf.Vec3d(*M0609_BASE_POS),
        new_rotation_euler=Gf.Vec3d(0, 0, 0),
        new_scale=Gf.Vec3d(1, 1, 1),
    )
    for _ in range(10):
        simulation_app.update()
    carb.log_warn(f"[T2-DEBUG] M0609 root translate → {M0609_BASE_POS} (via TransformPrimSRTCommand)")

    # translate 직후 실제 world 위치 (정상이면 base_link ≈ M0609_BASE_POS)
    if bl_path:
        bl_world = UsdGeom.Xformable(stage.GetPrimAtPath(bl_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        carb.log_warn(f"[T2-DEBUG] base_link world AFTER translate: ({bl_world[0]:+.3f}, {bl_world[1]:+.3f}, {bl_world[2]:+.3f})  ← 기대값 {M0609_BASE_POS}")
    root_world = UsdGeom.Xformable(m0609_root_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    carb.log_warn(f"[T2-DEBUG] robot_root world AFTER translate: ({root_world[0]:+.3f}, {root_world[1]:+.3f}, {root_world[2]:+.3f})")

    # ④ RG2 URDF import (floating, M0609 ee 에 결합)
    print("\n[4/5] Importing RG2 URDF + assembling to M0609 ee …")
    gripper_root = _import_urdf(str(RG2_URDF), fix_base=False)
    robot_ee = _find_prim_path_by_name(robot_root, M0609_EE_LINK) \
               or f"{robot_root}/{M0609_EE_LINK}"
    gripper_base = _find_prim_path_by_name(gripper_root, RG2_BASE_LINK) \
                   or f"{gripper_root}/{RG2_BASE_LINK}"
    _assemble(robot_root, robot_ee, gripper_root, gripper_base,
              "Gripper", "m0609_rg2")
    print(f"  [Assembly] M0609/{M0609_EE_LINK} ↔ RG2/{RG2_BASE_LINK}")

    # DEBUG: RG2 assemble 후 M0609 위치 변동 확인
    if bl_path:
        bl_world = UsdGeom.Xformable(stage.GetPrimAtPath(bl_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        carb.log_warn(f"[T2-DEBUG] base_link world AFTER RG2 assemble: ({bl_world[0]:+.3f}, {bl_world[1]:+.3f}, {bl_world[2]:+.3f})")

    # ⑤ rover Body ↔ M0609 base_link 결합 (rover 가 M0609 articulation 의 sub-tree 가 됨)
    print("\n[5/5] Assembling rover ↔ M0609 …")
    m0609_base = _find_prim_path_by_name(robot_root, "base_link") \
                 or f"{robot_root}/base_link"
    rover_body = _find_prim_path_by_name(rover_prim_path, "Body") \
                 or f"{rover_prim_path}/Body"
    print(f"  [Assembly] rover/Body ({rover_body}) ↔ M0609/base_link ({m0609_base})")
    # ⚠️ RobotAssembler 는 attach 의 mount 점을 base 의 mount 점 위치로 강제
    # 정렬함 (offset=0). m0609 를 rover Body 위 (0.15274, 0, 0.21232) offset
    # 위치에 두려면, rover 측에 그 offset 위치를 가리키는 Xform "mount point"
    # 를 만들어서 그걸 attach_mount 로 사용.
    # ⚠️ EditTarget 을 root layer 로 명시 — 이전 _assemble 가 sublayer 로 바꿔둔 상태일 수 있음.
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    rover_mount_path = f"{rover_body}/M0609_Mount"
    mount_prim = stage.DefinePrim(rover_mount_path, "Xform")
    if not mount_prim.IsValid():
        raise RuntimeError(f"Failed to create mount prim at {rover_mount_path}")
    UsdGeom.Xformable(mount_prim).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(
            float(M0609_BASE_POS[0] - ROVER_SPAWN_POS[0]),
            float(M0609_BASE_POS[1] - ROVER_SPAWN_POS[1]),
            float(M0609_BASE_POS[2] - ROVER_SPAWN_POS[2]),
        )
    )
    for _ in range(5):
        simulation_app.update()
    # 검증: mount Xform 이 의도된 world 위치에 author 됐는지
    mount_world = UsdGeom.Xformable(mount_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    carb.log_warn(f"[T2-DEBUG] M0609_Mount world pos: ({mount_world[0]:+.3f}, {mount_world[1]:+.3f}, {mount_world[2]:+.3f}) (기대: {M0609_BASE_POS})")
    # 이제 m0609.base_link 가 M0609_Mount 위치로 정렬됨 → rover.Body 위 offset 자세
    _assemble(
        robot_root, m0609_base,
        rover_prim_path, rover_mount_path,
        "RoverMount", "m0609_on_rover",
    )

    # DEBUG: rover assemble 후 M0609 / rover Body 위치 (이게 시뮬 시작 전 최종)
    if bl_path:
        bl_world = UsdGeom.Xformable(stage.GetPrimAtPath(bl_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        carb.log_warn(f"[T2-DEBUG] base_link world AFTER rover assemble: ({bl_world[0]:+.3f}, {bl_world[1]:+.3f}, {bl_world[2]:+.3f})")
    rb_prim = stage.GetPrimAtPath(rover_body)
    if rb_prim.IsValid():
        rb_world = UsdGeom.Xformable(rb_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        carb.log_warn(f"[T2-DEBUG] rover Body world AFTER rover assemble: ({rb_world[0]:+.3f}, {rb_world[1]:+.3f}, {rb_world[2]:+.3f})")

    # rover drive re-freeze (assemble 중에 풀릴 수 있음)
    _freeze_rover_drives(rover_prim)

    # 시각: m0609 (+ RG2) 를 rover 처럼 어두운 색으로 paint
    _paint_subtree_dark(robot_root, rgb=(0.08, 0.08, 0.08))

    for _ in range(10):
        simulation_app.update()

    # 디버그: 주요 link world pose (시뮬 시작 전 최종)
    print("\n[DEBUG] Key world positions (pre-simulation):")
    for path in (rover_body, m0609_base, robot_ee, gripper_base):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            print(f"  {path}: ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")

    return rover_prim, robot_root, rover_body, m0609_base


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--auto-play", action="store_true",
                        help="씬 빌드 후 자동으로 Play")
    args = parser.parse_args()

    world = World(stage_units_in_meters=1.0)
    rover_prim, robot_root, rover_body_path, m0609_base_path = build_scene()

    print("\n[World] Reset …")
    world.reset()

    # reset 후 rover drive 가 풀릴 수 있어 다시 freeze
    _freeze_rover_drives(rover_prim)

    # ⚠️ Articulation.set_world_pose 호출 제거. FixedJoint 가 양쪽 articulation 의
    # 상대 위치를 잡고 있는데 set_world_pose 가 한쪽만 강제 이동하면 joint 가
    # 깨지면서 m0609 가 분리됨. FixedJoint 가 USD authored pose 를 그대로 유지함.
    stage = omni.usd.get_context().get_stage()

    # DEBUG: world.reset 후 위치
    stage = omni.usd.get_context().get_stage()
    carb.log_warn(f"[T2-DEBUG] === AFTER world.reset + set_world_pose ===")
    for label, p in [("rover Body", rover_body_path), ("M0609 base_link", m0609_base_path)]:
        prim = stage.GetPrimAtPath(p)
        if prim.IsValid():
            t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            carb.log_warn(f"[T2-DEBUG] {label}: ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")

    if args.auto_play:
        world.play()
        print("\n[Play] auto-play ON")

    print("\n=== 씬 준비 완료. Isaac Sim GUI 에서 Spacebar 로 Play ===")
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
