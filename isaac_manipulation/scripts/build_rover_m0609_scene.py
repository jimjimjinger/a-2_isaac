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

from pathlib import Path

import numpy as np
import omni.kit.commands
import omni.kit.app
import omni.usd
import omni.graph.core as og
import omni.replicator.core as rep
import carb
import usdrt
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

try:
    _carb.set_bool("/app/omni.graph.scriptnode/opt_in", True)
except Exception:
    _carb.set("/app/omni.graph.scriptnode/opt_in", True)

# 필요한 Isaac/OmniGraph 확장 활성화
_ext_manager = omni.kit.app.get_app().get_extension_manager()
for _ext in (
    "isaacsim.robot_setup.assembler",
    "isaacsim.sensors.physics",
    "isaacsim.ros2.bridge",
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.graph.scriptnode",
    "omni.graph.bundle.action",
    "omni.graph.ui",
    "omni.graph.visualization.nodes",
    "omni.kit.graph.delegate.default",
    "omni.kit.graph.editor.core",
):
    _ext_manager.set_extension_enabled_immediate(_ext, True)

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import move_prim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_setup.assembler import RobotAssembler

os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

# ─── 경로 ──────────────────────────────────────────────────────────────────
A2_ROOT = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac")
MARS_WORLD_USD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
ROVER_USD = A2_ROOT / "isaac_sim/assets/rover_v2/vehicle/vehicle.usd"
M0609_URDF = A2_ROOT / "isaac_sim/assets/doosan-robot2/urdf/m0609_isaac_sim.urdf"
RG2_URDF = A2_ROOT / "isaac_sim/assets/onrobot_rg2/urdf/onrobot_rg2.urdf"
LOCALIZATION_SCENE_USD = A2_ROOT / "isaac_sim/assets/rover_v2/rover_m0609_localization.usd"

# URDF 안의 link 이름 (dual_cam_pick_place config 와 동일)
M0609_EE_LINK = "link_6"
RG2_BASE_LINK = "angle_bracket"

# ─── Spawn 위치 ─────────────────────────────────────────────────────────────
# 이전 (0,-6) 는 hill 옆이라 rover 기울어짐. 다른 방향으로 이동.
# terrain 은 procedural 이라 정확한 평지 좌표 모름 → 사용자가 본 결과 따라
# 아래 SPAWN_X / SPAWN_Y 만 바꿔서 빠르게 iterate.
SPAWN_X = 5.0    # 동쪽 5m
SPAWN_Y = 0.0
SPAWN_Z = 0.5    # terrain 위 시작 높이
ROVER_SPAWN_POS = (SPAWN_X, SPAWN_Y, SPAWN_Z)
ROVER_SPAWN_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)

# 로버 단독 주행 테스트용. Script Editor 없이 이 파일 실행만으로 전진시킨다.
AUTO_DRIVE_SPEED = 5.0

# Rover command/state graph topics and Ackermann geometry values.
ROVER_ACKERMANN_GRAPH_PATH = "/ActionGraph/RoverAckermannDrive"
ROVER_CMD_VEL_TOPIC = "/cmd_vel"
ROVER_STATE_GRAPH_PATH = "/ActionGraph/RoverStatePublishers"
JOINT_STATES_RAW_TOPIC = "/joint_states_raw"
ACKERMANN_WHEELBASE_LENGTH = 0.849
ACKERMANN_MIDDLE_WHEEL_DISTANCE = 0.894
ACKERMANN_REAR_FRONT_WHEEL_DISTANCE = 0.77
ACKERMANN_WHEEL_RADIUS = 0.1
ACKERMANN_OFFSET = -0.0135
ACKERMANN_STEERING_GAIN = 1.1
ACKERMANN_WHEEL_VELOCITY_GAIN = 1.5

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

# Rear basket visual. Robot arm side is front, so basket is placed at -X of Body.
BASKET_LOCAL_X = -0.38
BASKET_LOCAL_Y = 0.0
BASKET_LOCAL_Z = 0.02
BASKET_LENGTH = 0.22
BASKET_WIDTH = 0.46
BASKET_HEIGHT = 0.18
BASKET_WALL = 0.035
BASKET_BOTTOM = 0.035

# IMU sensor attached to rover body. Action Graph should read this prim.
IMU_SENSOR_NAME = "Imu_Sensor"
IMU_LOCAL_X = 0.0
IMU_LOCAL_Y = 0.0
IMU_LOCAL_Z = 0.12
IMU_SENSOR_PERIOD = -1.0

# Rover camera. One USD Camera prim publishes both RGB and depth.
CAMERA_NAME = "Camera"
CAMERA_LOCAL_X = 0.35
CAMERA_LOCAL_Y = 0.0
CAMERA_LOCAL_Z = 0.30
CAMERA_RPY_DEG = (90.0, 0.0, -90.0)
CAMERA_RESOLUTION = (640, 480)
CAMERA_FRAME_ID = "rover_camera"

# Wrist D455 cameras are already included in the integrated vehicle USD.
WRIST_COLOR_CAMERA_NAME = "Camera_OmniVision_OV9782_Color"
WRIST_DEPTH_CAMERA_NAME = "Camera_Pseudo_Depth"
WRIST_CAMERA_RESOLUTION = (640, 480)
WRIST_CAMERA_FRAME_ID = "wrist_camera"


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


def _find_articulation_root_path(root_path: str) -> str:
    """Find the first articulation root under a loaded vehicle USD."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"vehicle root prim not found: {root_path}")

    for prim in Usd.PrimRange(root_prim):
        try:
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(prim.GetPath())
        except Exception:
            pass
        schemas = {str(schema) for schema in prim.GetAppliedSchemas()}
        if "PhysicsArticulationRootAPI" in schemas:
            return str(prim.GetPath())

    carb.log_warn(f"articulation root not found under {root_path}; using vehicle root")
    return root_path


def _move_prim_under(root_path: str, parent_path: str) -> str:
    """Move an imported root prim under parent_path before assembly."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(root_path)
    if not prim.IsValid():
        raise RuntimeError(f"prim not found: {root_path}")

    name = prim.GetName()
    dst_path = f"{parent_path}/{name}"
    if root_path == dst_path:
        return root_path
    if stage.GetPrimAtPath(dst_path).IsValid():
        raise RuntimeError(f"destination already exists: {dst_path}")

    move_prim(root_path, dst_path)
    for _ in range(5):
        simulation_app.update()
    print(f"  [Scene] moved prim: {root_path} -> {dst_path}")
    return dst_path


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
        carb.log_warn(f"_paint_subtree_dark: root invalid {root_path}")
        return 0

    rgb_vec = Gf.Vec3f(*rgb)
    all_predicate = Usd.PrimAllPrimsPredicate

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
    return modified + bound


def _make_preview_material(stage, mat_path: str, rgb,
                           roughness: float = 0.55,
                           metallic: float = 0.35):
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*rgb)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def _define_local_box(stage, path: str, translation, scale, material=None):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*translation)
    )
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    return cube.GetPrim()


def _attach_rear_basket(rover_body: str) -> str:
    """로버 Body 뒤쪽에 visual basket 을 부착한다.

    Body 의 child 로 만들기 때문에 로버와 딱 붙어서 같이 움직인다.
    """
    stage = omni.usd.get_context().get_stage()
    body_prim = stage.GetPrimAtPath(rover_body)
    if not body_prim.IsValid():
        raise RuntimeError(f"rover Body prim not found: {rover_body}")

    basket_path = f"{rover_body}/RearBasket"
    basket = stage.DefinePrim(basket_path, "Xform")
    xf = UsdGeom.Xformable(basket)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(BASKET_LOCAL_X, BASKET_LOCAL_Y, BASKET_LOCAL_Z)
    )

    mat = _make_preview_material(
        stage,
        f"{basket_path}/Looks/BasketDarkMetal",
        rgb=(0.09, 0.075, 0.06),
        roughness=0.5,
        metallic=0.45,
    )
    rim_mat = _make_preview_material(
        stage,
        f"{basket_path}/Looks/BasketRimMetal",
        rgb=(0.45, 0.42, 0.36),
        roughness=0.35,
        metallic=0.75,
    )

    lx = BASKET_LENGTH
    wy = BASKET_WIDTH
    hz = BASKET_HEIGHT
    t = BASKET_WALL
    b = BASKET_BOTTOM

    # Bottom and low panels. Top is open, rear wall is slightly lower like a tray.
    _define_local_box(stage, f"{basket_path}/Bottom",
                      (0.0, 0.0, b * 0.5),
                      (lx, wy, b), mat)
    _define_local_box(stage, f"{basket_path}/LeftWall",
                      (0.0, wy * 0.5 - t * 0.5, hz * 0.5),
                      (lx, t, hz), mat)
    _define_local_box(stage, f"{basket_path}/RightWall",
                      (0.0, -wy * 0.5 + t * 0.5, hz * 0.5),
                      (lx, t, hz), mat)
    _define_local_box(stage, f"{basket_path}/BackWall",
                      (-lx * 0.5 + t * 0.5, 0.0, hz * 0.45),
                      (t, wy, hz * 0.9), mat)
    _define_local_box(stage, f"{basket_path}/FrontLip",
                      (lx * 0.5 - t * 0.5, 0.0, hz * 0.28),
                      (t, wy, hz * 0.56), mat)

    # Thin bright rims make it read more like a basket than a plain box.
    rim_z = hz + t * 0.5
    _define_local_box(stage, f"{basket_path}/LeftTopRim",
                      (0.0, wy * 0.5, rim_z),
                      (lx + t, t, t), rim_mat)
    _define_local_box(stage, f"{basket_path}/RightTopRim",
                      (0.0, -wy * 0.5, rim_z),
                      (lx + t, t, t), rim_mat)
    _define_local_box(stage, f"{basket_path}/BackTopRim",
                      (-lx * 0.5, 0.0, rim_z),
                      (t, wy + t, t), rim_mat)
    _define_local_box(stage, f"{basket_path}/FrontTopRim",
                      (lx * 0.5, 0.0, hz * 0.58),
                      (t, wy + t, t), rim_mat)

    print(f"  [Scene] rear basket attached: {basket_path}")
    return basket_path


def _attach_imu_sensor(rover_body: str) -> str:
    """Create an Isaac IMU sensor under rover Body."""
    stage = omni.usd.get_context().get_stage()
    body_prim = stage.GetPrimAtPath(rover_body)
    if not body_prim.IsValid():
        raise RuntimeError(f"rover Body prim not found: {rover_body}")

    imu_path = f"{rover_body}/{IMU_SENSOR_NAME}"
    if stage.GetPrimAtPath(imu_path).IsValid():
        print(f"  [Scene] IMU sensor already exists: {imu_path}")
        return imu_path

    ret = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=f"/{IMU_SENSOR_NAME}",
        parent=rover_body,
        sensor_period=IMU_SENSOR_PERIOD,
        translation=Gf.Vec3d(IMU_LOCAL_X, IMU_LOCAL_Y, IMU_LOCAL_Z),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        linear_acceleration_filter_size=1,
        angular_velocity_filter_size=1,
        orientation_filter_size=1,
    )

    sensor_prim = ret[1] if isinstance(ret, tuple) else ret
    if sensor_prim is None or not stage.GetPrimAtPath(imu_path).IsValid():
        raise RuntimeError(f"Failed to create IMU sensor at {imu_path}")

    print(f"  [Scene] IMU sensor attached: {imu_path}")
    return imu_path


def _attach_rover_camera(rover_body: str):
    """Create/reuse the rover camera and return (camera_path, render_product_path)."""
    stage = omni.usd.get_context().get_stage()
    body_prim = stage.GetPrimAtPath(rover_body)
    if not body_prim.IsValid():
        raise RuntimeError(f"rover Body prim not found: {rover_body}")

    camera_path = f"{rover_body}/{CAMERA_NAME}"
    if not stage.GetPrimAtPath(camera_path).IsValid():
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=camera_path,
            prim_type="Camera",
        )
        cam_prim = stage.GetPrimAtPath(camera_path)
        cam_xf = UsdGeom.Xformable(cam_prim)
        cam_xf.ClearXformOpOrder()
        cam_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(CAMERA_LOCAL_X, CAMERA_LOCAL_Y, CAMERA_LOCAL_Z)
        )
        cam_xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*CAMERA_RPY_DEG)
        )
        print(f"  [Scene] camera attached: {camera_path}")
    else:
        print(f"  [Scene] camera already exists: {camera_path}")

    UsdGeom.Imageable(stage.GetPrimAtPath(camera_path)).MakeInvisible()
    render_product = rep.create.render_product(camera_path, CAMERA_RESOLUTION)
    render_product_path = render_product.path
    print(f"  [Scene] camera render product: {render_product_path}")
    return camera_path, render_product_path


def _create_wrist_camera_render_products(rover_prim_path: str):
    """Create render products for the D455 wrist RGB and pseudo-depth cameras."""
    stage = omni.usd.get_context().get_stage()
    color_camera_path = _find_prim_path_by_name(rover_prim_path, WRIST_COLOR_CAMERA_NAME)
    depth_camera_path = _find_prim_path_by_name(rover_prim_path, WRIST_DEPTH_CAMERA_NAME)

    if not color_camera_path:
        raise RuntimeError(
            f"wrist color camera prim not found: {WRIST_COLOR_CAMERA_NAME}"
        )
    if not depth_camera_path:
        raise RuntimeError(
            f"wrist depth camera prim not found: {WRIST_DEPTH_CAMERA_NAME}"
        )
    if not stage.GetPrimAtPath(color_camera_path).IsValid():
        raise RuntimeError(f"invalid wrist color camera path: {color_camera_path}")
    if not stage.GetPrimAtPath(depth_camera_path).IsValid():
        raise RuntimeError(f"invalid wrist depth camera path: {depth_camera_path}")

    color_render_product = rep.create.render_product(
        color_camera_path,
        WRIST_CAMERA_RESOLUTION,
    )
    depth_render_product = rep.create.render_product(
        depth_camera_path,
        WRIST_CAMERA_RESOLUTION,
    )

    print(f"  [Scene] wrist color camera: {color_camera_path}")
    print(f"  [Scene] wrist depth camera: {depth_camera_path}")
    print(f"  [Scene] wrist color render product: {color_render_product.path}")
    print(f"  [Scene] wrist depth render product: {depth_render_product.path}")
    return color_render_product.path, depth_render_product.path


def _create_localization_ros2_graph(
    imu_path: str,
    camera_render_product_path: str = "",
    wrist_color_render_product_path: str = "",
    wrist_depth_render_product_path: str = "",
) -> str:
    """Create ROS2 publishers for localization sensors and rover camera."""
    stage = omni.usd.get_context().get_stage()
    graph_path = "/ActionGraph/LocalizationSensors"

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(Sdf.Path(graph_path))

    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
        ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
    ]
    connect = [
        ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
        ("ReadIMU.outputs:execOut", "PublishImu.inputs:execIn"),
        ("Context.outputs:context", "PublishImu.inputs:context"),
        ("ReadIMU.outputs:sensorTime", "PublishImu.inputs:timeStamp"),
        ("ReadIMU.outputs:angVel", "PublishImu.inputs:angularVelocity"),
        ("ReadIMU.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
        ("ReadIMU.outputs:orientation", "PublishImu.inputs:orientation"),
    ]
    set_values = [
        ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_path)]),
        ("ReadIMU.inputs:readGravity", True),
        ("ReadIMU.inputs:useLatestData", False),
        ("PublishImu.inputs:topicName", "/imu/data"),
        ("PublishImu.inputs:frameId", "sim_imu"),
        ("PublishImu.inputs:publishAngularVelocity", True),
        ("PublishImu.inputs:publishLinearAcceleration", True),
        ("PublishImu.inputs:publishOrientation", True),
    ]

    if camera_render_product_path:
        create_nodes.extend(
            [
                ("CameraRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ]
        )
        connect.extend(
            [
                ("OnPlaybackTick.outputs:tick", "CameraRgb.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraDepth.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraInfo.inputs:execIn"),
                ("Context.outputs:context", "CameraRgb.inputs:context"),
                ("Context.outputs:context", "CameraDepth.inputs:context"),
                ("Context.outputs:context", "CameraInfo.inputs:context"),
            ]
        )
        set_values.extend(
            [
                ("CameraRgb.inputs:renderProductPath", camera_render_product_path),
                ("CameraRgb.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraRgb.inputs:topicName", "/camera/rover/image_raw"),
                ("CameraRgb.inputs:type", "rgb"),
                ("CameraDepth.inputs:renderProductPath", camera_render_product_path),
                ("CameraDepth.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraDepth.inputs:topicName", "/camera/rover/depth"),
                ("CameraDepth.inputs:type", "depth"),
                ("CameraInfo.inputs:renderProductPath", camera_render_product_path),
                ("CameraInfo.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraInfo.inputs:topicName", "/camera/rover/camera_info"),
            ]
        )

    if wrist_color_render_product_path and wrist_depth_render_product_path:
        create_nodes.extend(
            [
                ("WristCameraRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("WristCameraDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("WristCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ]
        )
        connect.extend(
            [
                ("OnPlaybackTick.outputs:tick", "WristCameraRgb.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "WristCameraDepth.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "WristCameraInfo.inputs:execIn"),
                ("Context.outputs:context", "WristCameraRgb.inputs:context"),
                ("Context.outputs:context", "WristCameraDepth.inputs:context"),
                ("Context.outputs:context", "WristCameraInfo.inputs:context"),
            ]
        )
        set_values.extend(
            [
                (
                    "WristCameraRgb.inputs:renderProductPath",
                    wrist_color_render_product_path,
                ),
                ("WristCameraRgb.inputs:frameId", WRIST_CAMERA_FRAME_ID),
                ("WristCameraRgb.inputs:topicName", "/camera/wrist/image_raw"),
                ("WristCameraRgb.inputs:type", "rgb"),
                (
                    "WristCameraDepth.inputs:renderProductPath",
                    wrist_depth_render_product_path,
                ),
                ("WristCameraDepth.inputs:frameId", WRIST_CAMERA_FRAME_ID),
                ("WristCameraDepth.inputs:topicName", "/camera/wrist/depth"),
                ("WristCameraDepth.inputs:type", "depth"),
                (
                    "WristCameraInfo.inputs:renderProductPath",
                    wrist_depth_render_product_path,
                ),
                ("WristCameraInfo.inputs:frameId", WRIST_CAMERA_FRAME_ID),
                ("WristCameraInfo.inputs:topicName", "/camera/wrist/camera_info"),
            ]
        )

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connect,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )

    print(f"  [Scene] ROS2 localization graph created: {graph_path}")
    print(f"          /imu/data IMU prim : {imu_path}")
    if camera_render_product_path:
        print("          /camera/rover/image_raw")
        print("          /camera/rover/depth")
        print("          /camera/rover/camera_info")
    if wrist_color_render_product_path and wrist_depth_render_product_path:
        print("          /camera/wrist/image_raw")
        print("          /camera/wrist/depth")
        print("          /camera/wrist/camera_info")
    return graph_path


def _freeze_rover_drives(rover_prim) -> int:
    """Freeze only rover wheel/steer drives while leaving arm/gripper drives intact."""
    n = 0
    for prim in Usd.PrimRange(rover_prim):
        name = prim.GetName()
        if not (_is_drive_joint_name(name) or _is_steer_joint_name(name)):
            continue

        drv = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drv:
            continue

        drv.GetTargetPositionAttr().Set(0.0)
        drv.GetTargetVelocityAttr().Set(0.0)
        drv.GetStiffnessAttr().Set(1e8)
        drv.GetDampingAttr().Set(1e6)
        drv.GetMaxForceAttr().Set(1e7)
        n += 1
    return n


def _configure_rover_drives_for_controller(rover_prim, log: bool = True) -> int:
    """Allow OmniGraph/Python articulation controllers to command rover joints.

    _freeze_rover_drives() is useful while assembling the scene, but it also
    makes wheel drives fight velocity commands. For /cmd_vel graph control the
    drive wheels must be velocity-controlled, while steering joints stay
    position-controlled.
    """
    n = 0
    for prim in Usd.PrimRange(rover_prim):
        name = prim.GetName()
        drv = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drv:
            continue

        if _is_drive_joint_name(name):
            drv.GetTargetPositionAttr().Set(0.0)
            drv.GetTargetVelocityAttr().Set(0.0)
            drv.GetStiffnessAttr().Set(0.0)
            drv.GetDampingAttr().Set(1e5)
            drv.GetMaxForceAttr().Set(1e7)
            n += 1
        elif _is_steer_joint_name(name):
            drv.GetTargetPositionAttr().Set(0.0)
            drv.GetTargetVelocityAttr().Set(0.0)
            drv.GetStiffnessAttr().Set(1e8)
            drv.GetDampingAttr().Set(1e6)
            drv.GetMaxForceAttr().Set(1e7)

    if log:
        print(f"  [Drive] rover drives configured for controller ({n} wheel drives)")
    return n


def _remove_mars_rocks(stage) -> int:
    """Referenced Mars world 에서 rocks subtree 를 전부 숨기고 비활성화."""
    removed = 0
    known_paths = (
        "/World/Mars/Rocks",
        "/World/Mars/World/Rocks",
        "/World/Rocks",
    )

    for path in known_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        prim.SetActive(False)
        print(f"  [Scene] rocks disabled: {path}")
        removed += 1

    for prim in list(stage.Traverse()):
        path = str(prim.GetPath())
        name = prim.GetName()
        text = f"{path}/{name}".lower()
        if "rock" not in text:
            continue
        prim.SetActive(False)
        print(f"  [Scene] rock prim disabled: {path}")
        removed += 1

    if removed == 0:
        print("  [Scene] rocks prim not found; skip removing rocks")
    return removed


def _is_drive_joint_name(name: str) -> bool:
    upper = name.upper()
    return (
        "DRIVE" in upper
        and "STEER" not in upper
        and any(pos in upper for pos in ("FL", "FR", "CL", "CR", "ML", "MR", "RL", "RR"))
    )


def _is_steer_joint_name(name: str) -> bool:
    upper = name.upper()
    return (
        "STEER" in upper
        and any(pos in upper for pos in ("FL", "FR", "RL", "RR"))
    )


def _command_rover_forward(rover_prim, speed: float = AUTO_DRIVE_SPEED, log: bool = True) -> int:
    """Drive wheel target velocity 설정.

    새 모델은 Stage 에 FL_Drive, FR_Drive ... 형태로 보이고, 예전 모델은
    FL_Drive_Continuous ... 형태로 보일 수 있다. 이름에 DRIVE 가 들어간
    wheel joint 를 찾아 angular DriveAPI targetVelocity 를 준다.
    """
    n = 0
    for prim in Usd.PrimRange(rover_prim):
        name = prim.GetName()
        drv = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drv:
            continue

        if _is_drive_joint_name(name):
            drv.GetTargetPositionAttr().Set(0.0)
            drv.GetTargetVelocityAttr().Set(float(speed))
            drv.GetStiffnessAttr().Set(0.0)
            drv.GetDampingAttr().Set(1e5)
            drv.GetMaxForceAttr().Set(1e7)
            if log:
                print(f"  [Drive] {prim.GetPath()} targetVelocity={speed}")
            n += 1
        elif _is_steer_joint_name(name):
            drv.GetTargetPositionAttr().Set(0.0)
            drv.GetTargetVelocityAttr().Set(0.0)
            drv.GetStiffnessAttr().Set(1e8)
            drv.GetDampingAttr().Set(1e6)
            drv.GetMaxForceAttr().Set(1e7)

    if log:
        print(f"  [Drive] forward command applied to {n} wheel drives")
    return n


def _find_drive_dof_indices(dof_names):
    indices = []
    for idx, name in enumerate(dof_names):
        if _is_drive_joint_name(name):
            indices.append(idx)
    return np.array(indices, dtype=np.int32)


def _joint_order_index(name: str, order) -> int:
    upper = name.upper()
    for idx, key in enumerate(order):
        if upper.startswith(key) or f"_{key}" in upper or f"{key}_" in upper:
            return idx
    return len(order)


def _ordered_rover_joint_names(rover_prim):
    """Return joint names in the same order as Isaac Lab's Ackermann action."""
    drive_names = []
    steer_names = []

    for prim in Usd.PrimRange(rover_prim):
        name = prim.GetName()
        if not UsdPhysics.DriveAPI.Get(prim, "angular"):
            continue
        if _is_drive_joint_name(name):
            drive_names.append(name)
        elif _is_steer_joint_name(name):
            steer_names.append(name)

    drive_order = ("FL", "FR", "CL", "CR", "ML", "MR", "RL", "RR")
    steer_order = ("FL", "FR", "RL", "RR")
    drive_names = sorted(set(drive_names), key=lambda n: _joint_order_index(n, drive_order))
    steer_names = sorted(set(steer_names), key=lambda n: _joint_order_index(n, steer_order))

    # Lab action emits six wheel speeds: FL, FR, middle-left, middle-right, RL, RR.
    if len(drive_names) > 6:
        carb.log_warn(f"[AckermannGraph] found {len(drive_names)} drive joints; using first 6: {drive_names[:6]}")
        drive_names = drive_names[:6]
    if len(steer_names) > 4:
        carb.log_warn(f"[AckermannGraph] found {len(steer_names)} steer joints; using first 4: {steer_names[:4]}")
        steer_names = steer_names[:4]

    return steer_names, drive_names


def _collect_controlled_joint_names(root_paths, exclude_names=()):
    """Collect named joints with drive APIs under the given prim roots."""
    stage = omni.usd.get_context().get_stage()
    excluded = set(exclude_names)
    names = []
    seen = set()

    for root_path in root_paths:
        root = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            continue
        for prim in Usd.PrimRange(root):
            name = prim.GetName()
            if name in excluded or name in seen:
                continue
            if _is_drive_joint_name(name) or _is_steer_joint_name(name):
                continue
            if not (
                UsdPhysics.DriveAPI.Get(prim, "angular")
                or UsdPhysics.DriveAPI.Get(prim, "linear")
            ):
                continue
            names.append(name)
            seen.add(name)

    return names


def _make_ackermann_script(steer_joint_names, drive_joint_names) -> str:
    """Python body for OmniGraph ScriptNode.

    This mirrors rover_envs.mdp.actions.ackermann_actions.ackermann(), but
    runs inside the USD Action Graph so the saved scene can receive /cmd_vel.
    """
    return f"""
import math
import omni.graph.core as og

STEER_JOINT_NAMES = {list(steer_joint_names)!r}
DRIVE_JOINT_NAMES = {list(drive_joint_names)!r}
WHEELBASE_LENGTH = {ACKERMANN_WHEELBASE_LENGTH!r}
MIDDLE_WHEEL_DISTANCE = {ACKERMANN_MIDDLE_WHEEL_DISTANCE!r}
REAR_FRONT_WHEEL_DISTANCE = {ACKERMANN_REAR_FRONT_WHEEL_DISTANCE!r}
WHEEL_RADIUS = {ACKERMANN_WHEEL_RADIUS!r}
OFFSET = {ACKERMANN_OFFSET!r}
STEERING_GAIN = {ACKERMANN_STEERING_GAIN!r}
WHEEL_VELOCITY_GAIN = {ACKERMANN_WHEEL_VELOCITY_GAIN!r}


def _component(value, index):
    try:
        return float(value[index])
    except Exception:
        try:
            return float(getattr(value, ("x", "y", "z")[index]))
        except Exception:
            return 0.0


def setup(db):
    pass


def compute(db):
    lin_vel = _component(db.inputs.linearVelocity, 0)
    ang_vel = _component(db.inputs.angularVelocity, 2)

    direction = 1.0 if lin_vel >= 0.0 else -1.0
    if ang_vel > 0.0:
        turn_direction = 1.0
    elif ang_vel < 0.0:
        turn_direction = -1.0
    else:
        turn_direction = 0.0

    lin = abs(lin_vel)
    ang = abs(ang_vel)
    turning_radius = float("inf") if ang == 0.0 else lin / ang
    minimum_radius = MIDDLE_WHEEL_DISTANCE * 0.8

    r_ml = turning_radius - (MIDDLE_WHEEL_DISTANCE / 2.0) * turn_direction
    r_mr = turning_radius + (MIDDLE_WHEEL_DISTANCE / 2.0) * turn_direction
    r_fl = math.hypot(r_ml, WHEELBASE_LENGTH / 2.0)
    r_fr = math.hypot(r_mr, WHEELBASE_LENGTH / 2.0)
    r_rl = math.hypot(r_ml, WHEELBASE_LENGTH / 2.0)
    r_rr = math.hypot(r_mr, WHEELBASE_LENGTH / 2.0)

    if turning_radius < minimum_radius:
        vel_fl = -ang * turn_direction
        vel_fr = ang * turn_direction
        vel_rl = -ang * turn_direction
        vel_rr = ang * turn_direction
        vel_ml = -ang * turn_direction
        vel_mr = ang * turn_direction
    else:
        vel_fl = (r_fl * ang if ang > 0.0 else lin) * direction
        vel_fr = (r_fr * ang if ang > 0.0 else lin) * direction
        vel_rl = (r_rl * ang if ang > 0.0 else lin) * direction
        vel_rr = (r_rr * ang if ang > 0.0 else lin) * direction
        vel_ml = (r_ml * ang if ang > 0.0 else lin) * direction
        vel_mr = (r_mr * ang if ang > 0.0 else lin) * direction

    if turning_radius < minimum_radius:
        theta_fl = -math.pi / 4.0
        theta_fr = math.pi / 4.0
        theta_rl = math.pi / 4.0
        theta_rr = -math.pi / 4.0
    else:
        theta_fl = math.atan2((WHEELBASE_LENGTH / 2.0) - OFFSET, r_fl) * turn_direction
        theta_fr = math.atan2((WHEELBASE_LENGTH / 2.0) - OFFSET, r_fr) * turn_direction
        theta_rl = math.atan2((WHEELBASE_LENGTH / 2.0) + OFFSET, r_rl) * -turn_direction
        theta_rr = math.atan2((WHEELBASE_LENGTH / 2.0) + OFFSET, r_rr) * -turn_direction

    db.outputs.steerJointNames = STEER_JOINT_NAMES
    db.outputs.driveJointNames = DRIVE_JOINT_NAMES
    db.outputs.steeringAngles = [
        STEERING_GAIN * theta_fl,
        STEERING_GAIN * theta_fr,
        STEERING_GAIN * theta_rl,
        STEERING_GAIN * theta_rr,
    ][:len(STEER_JOINT_NAMES)]
    db.outputs.wheelVelocities = [
        WHEEL_VELOCITY_GAIN * vel_fl / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_fr / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_ml / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_mr / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_rl / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_rr / (WHEEL_RADIUS * 2.0),
    ][:len(DRIVE_JOINT_NAMES)]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
"""


def _create_rover_ackermann_drive_graph(robot_root: str, rover_prim) -> str:
    """Create a saved Action Graph that drives the rover from ROS2 /cmd_vel."""
    stage = omni.usd.get_context().get_stage()
    graph_path = ROVER_ACKERMANN_GRAPH_PATH

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(Sdf.Path(graph_path))

    steer_joint_names, drive_joint_names = _ordered_rover_joint_names(rover_prim)
    if len(steer_joint_names) != 4:
        carb.log_warn(f"[AckermannGraph] expected 4 steer joints, found {len(steer_joint_names)}: {steer_joint_names}")
    if len(drive_joint_names) != 6:
        carb.log_warn(f"[AckermannGraph] expected 6 drive joints, found {len(drive_joint_names)}: {drive_joint_names}")

    script = _make_ackermann_script(steer_joint_names, drive_joint_names)

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeCmdVel", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("Ackermann", "omni.graph.scriptnode.ScriptNode"),
                ("SteerController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("DriveController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CREATE_ATTRIBUTES: [
                ("Ackermann.inputs:linearVelocity", "any"),
                ("Ackermann.inputs:angularVelocity", "any"),
                ("Ackermann.outputs:steerJointNames", "token[]"),
                ("Ackermann.outputs:driveJointNames", "token[]"),
                ("Ackermann.outputs:steeringAngles", "double[]"),
                ("Ackermann.outputs:wheelVelocities", "double[]"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeCmdVel.inputs:execIn"),
                ("Context.outputs:context", "SubscribeCmdVel.inputs:context"),
                ("SubscribeCmdVel.outputs:execOut", "Ackermann.inputs:execIn"),
                ("SubscribeCmdVel.outputs:linearVelocity", "Ackermann.inputs:linearVelocity"),
                ("SubscribeCmdVel.outputs:angularVelocity", "Ackermann.inputs:angularVelocity"),
                ("Ackermann.outputs:execOut", "SteerController.inputs:execIn"),
                ("Ackermann.outputs:execOut", "DriveController.inputs:execIn"),
                ("Ackermann.outputs:steerJointNames", "SteerController.inputs:jointNames"),
                ("Ackermann.outputs:steeringAngles", "SteerController.inputs:positionCommand"),
                ("Ackermann.outputs:driveJointNames", "DriveController.inputs:jointNames"),
                ("Ackermann.outputs:wheelVelocities", "DriveController.inputs:velocityCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeCmdVel.inputs:topicName", ROVER_CMD_VEL_TOPIC),
                ("Ackermann.inputs:script", script),
                ("SteerController.inputs:robotPath", robot_root),
                ("DriveController.inputs:robotPath", robot_root),
            ],
        },
    )

    print(f"  [Scene] ROS2 Ackermann drive graph created: {graph_path}")
    print(f"          subscribe Twist: {ROVER_CMD_VEL_TOPIC}")
    print(f"          steer joints   : {steer_joint_names}")
    print(f"          drive joints   : {drive_joint_names}")
    return graph_path


def _create_rover_state_publishers_graph(robot_root: str) -> str:
    """Publish raw articulation joint states with Isaac's built-in ROS2 node.

    Filtering into /rover/wheel_states and arm-focused /joint_states should be
    done by a normal ROS2 node outside Isaac Sim. This keeps the Isaac graph on
    official bridge nodes and avoids rclpy inside ScriptNode.
    """
    stage = omni.usd.get_context().get_stage()
    graph_path = ROVER_STATE_GRAPH_PATH

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(Sdf.Path(graph_path))

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointStateRaw", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointStateRaw.inputs:execIn"),
                ("Context.outputs:context", "PublishJointStateRaw.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointStateRaw.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointStateRaw.inputs:topicName", JOINT_STATES_RAW_TOPIC),
                ("PublishJointStateRaw.inputs:targetPrim", [usdrt.Sdf.Path(robot_root)]),
            ],
        },
    )

    print(f"  [Scene] ROS2 rover state publishers graph created: {graph_path}")
    print(f"          {JOINT_STATES_RAW_TOPIC}: raw articulation JointState")
    return graph_path


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
    _remove_mars_rocks(stage)

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

    # ② rover + M0609 + RG2-FT 통합 vehicle.usd reference
    print("\n[2/5] Adding integrated vehicle.usd …")
    if not ROVER_USD.exists():
        raise FileNotFoundError(ROVER_USD)
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

    robot_root = _find_articulation_root_path(rover_prim_path)
    rover_body = _find_prim_path_by_name(rover_prim_path, "Body") \
                 or f"{rover_prim_path}/Body"
    m0609_base = _find_prim_path_by_name(rover_prim_path, "base_link") \
                 or robot_root

    n = _freeze_rover_drives(rover_prim)
    print(f"  [Scene] integrated vehicle @ {ROVER_SPAWN_POS}  (drives frozen: {n})")
    print(f"  [Scene] articulation root: {robot_root}")
    print(f"  [Scene] rover body       : {rover_body}")

    imu_path = _attach_imu_sensor(rover_body)
    _, camera_render_product_path = _attach_rover_camera(rover_body)
    wrist_color_render_product_path, wrist_depth_render_product_path = (
        _create_wrist_camera_render_products(rover_prim_path)
    )
    _create_localization_ros2_graph(
        imu_path,
        camera_render_product_path,
        wrist_color_render_product_path,
        wrist_depth_render_product_path,
    )
    _create_rover_ackermann_drive_graph(robot_root, rover_prim)
    _create_rover_state_publishers_graph(robot_root)

    for _ in range(10):
        simulation_app.update()

    return rover_prim, robot_root, rover_body, m0609_base


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--auto-play", action="store_true",
                        help="씬 빌드 후 자동으로 Play")
    parser.add_argument("--no-drive", action="store_true",
                        help="씬만 띄우고 로버 전진 명령은 주지 않음")
    parser.add_argument("--drive-speed", type=float, default=AUTO_DRIVE_SPEED,
                        help="로버 전진 wheel target velocity")
    parser.add_argument("--save-scene", action="store_true",
                        help=f"구성된 localization scene USD 저장: {LOCALIZATION_SCENE_USD}")
    args = parser.parse_args()

    world = World(stage_units_in_meters=1.0)
    rover_prim, robot_root, rover_body_path, m0609_base_path = build_scene()

    print("\n[World] Reset …")
    world.reset()

    # reset 후 drive 상태가 바뀔 수 있어 다시 설정한다. --no-drive 는 Python
    # 주행만 끄는 옵션이고, /cmd_vel Action Graph 제어는 계속 허용해야 한다.
    _configure_rover_drives_for_controller(rover_prim)

    # ⚠️ Articulation.set_world_pose 호출 제거. FixedJoint 가 양쪽 articulation 의
    # 상대 위치를 잡고 있는데 set_world_pose 가 한쪽만 강제 이동하면 joint 가
    # 깨지면서 m0609 가 분리됨. FixedJoint 가 USD authored pose 를 그대로 유지함.
    stage = omni.usd.get_context().get_stage()

    if args.save_scene:
        LOCALIZATION_SCENE_USD.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(LOCALIZATION_SCENE_USD))
        print(f"\n[Save] scene USD saved: {LOCALIZATION_SCENE_USD}")

    drive_articulation = None
    drive_indices = np.array([], dtype=np.int32)

    if not args.no_drive:
        try:
            drive_articulation = SingleArticulation(
                prim_path=robot_root,
                name="rover_m0609_drive_articulation",
            )
            drive_articulation.initialize()
            drive_indices = _find_drive_dof_indices(drive_articulation.dof_names)
            print("[Drive] articulation root:", robot_root)
            print(f"[Drive] drive DOF count = {len(drive_indices)}")
            for idx in drive_indices:
                print(f"  {idx}: {drive_articulation.dof_names[idx]}")
        except Exception as exc:
            carb.log_warn(f"[Drive] articulation drive init failed; USD drive fallback only: {exc}")
            drive_articulation = None
            drive_indices = np.array([], dtype=np.int32)

    if not args.no_drive:
        print(f"\n[Drive] rover forward speed = {args.drive_speed}")
        _command_rover_forward(rover_prim, args.drive_speed)

    if args.auto_play or not args.no_drive:
        world.play()
        print("\n[Play] auto-play ON")

    print("\n=== 씬 준비 완료. 정지하려면 터미널에서 Ctrl+C ===")
    try:
        while simulation_app.is_running():
            if drive_articulation is not None and len(drive_indices) > 0:
                joint_velocities = np.zeros(drive_articulation.num_dof)
                joint_velocities[drive_indices] = args.drive_speed
                drive_articulation.apply_action(
                    ArticulationAction(joint_velocities=joint_velocities)
                )
            elif not args.no_drive:
                _command_rover_forward(rover_prim, args.drive_speed, log=False)
            world.step(render=True)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
