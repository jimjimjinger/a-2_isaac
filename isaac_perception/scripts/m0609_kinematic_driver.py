"""ManualM0609Driver — all-kinematic 환경에서 M0609 + RG2-FT 를 FK 로 구동.

배경:
  PhysX 는 ArticulationRootAPI 가 적용된 prim 이 kinematic 인 경우 articulation 을
  무효화한다 (`ArticulationRootAPI definition on a kinematic rigid body is not allowed`).
  rover_yolo_demo.py 는 모든 차량 RigidBody 를 kinematic 으로 두고 t_op 으로
  teleport 식 navigation 을 하므로 SingleArticulation 사용 불가.

해결:
  USD 의 joint info (axis, body0/body1LocalPos·Rot) 를 파싱해 joint angle → link world
  xform 을 FK 로 직접 계산하고, 각 link 의 xform 을 USD 에 set. SingleArticulation 의
  부분 인터페이스 (num_dof, dof_names, body_names, get_joint_positions, set_joint_positions,
  get_jacobians, get_articulation_controller) 를 mimic 해 기존 IK 코드를 재사용.

Coordinate convention:
  numpy 4x4 column-major (translation = mat[:3, 3]). Gf.Matrix4d 는 row-major 이므로
  변환 시 transpose. 회전은 Rodrigues 식.
"""
from __future__ import annotations

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def _gf_to_np(m: Gf.Matrix4d) -> np.ndarray:
    """Gf.Matrix4d (row-major) → numpy 4x4 column-major."""
    arr = np.array(m, dtype=np.float64)  # shape (4,4), m[i][j] = arr[i, j]
    return arr.T  # transpose to column-major


def _np_to_gf(arr: np.ndarray) -> Gf.Matrix4d:
    """numpy 4x4 column-major → Gf.Matrix4d (row-major)."""
    row_major = arr.T
    return Gf.Matrix4d(
        float(row_major[0, 0]), float(row_major[0, 1]), float(row_major[0, 2]), float(row_major[0, 3]),
        float(row_major[1, 0]), float(row_major[1, 1]), float(row_major[1, 2]), float(row_major[1, 3]),
        float(row_major[2, 0]), float(row_major[2, 1]), float(row_major[2, 2]), float(row_major[2, 3]),
        float(row_major[3, 0]), float(row_major[3, 1]), float(row_major[3, 2]), float(row_major[3, 3]),
    )


def _pose_to_mat(pos, quat) -> np.ndarray:
    """pos (Gf.Vec3f/Vec3d), quat (Gf.Quatf/Quatd, w+xyz) → numpy 4x4 column-major."""
    if pos is None:
        pos = (0.0, 0.0, 0.0)
    if quat is None:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    else:
        qw = quat.GetReal()
        qi = quat.GetImaginary()
        qx, qy, qz = float(qi[0]), float(qi[1]), float(qi[2])
    # rotation matrix from quaternion (column-major)
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [float(pos[0]), float(pos[1]), float(pos[2])]
    return M


def _rot_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues — 4x4 rotation about unit axis by angle (rad), column-major."""
    a = axis / (np.linalg.norm(axis) + 1e-12)
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = a
    R = np.array([
        [c + x*x*(1-c),   x*y*(1-c) - z*s, x*z*(1-c) + y*s],
        [y*x*(1-c) + z*s, c + y*y*(1-c),   y*z*(1-c) - x*s],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)  ],
    ], dtype=np.float64)
    M = np.eye(4)
    M[:3, :3] = R
    return M


class _StubController:
    """SingleArticulation 의 articulation_controller.apply_action() 인터페이스 mimic."""
    def __init__(self, parent):
        self._parent = parent

    def apply_action(self, action):
        joint_positions = getattr(action, "joint_positions", None)
        joint_indices = getattr(action, "joint_indices", None)
        if joint_positions is None or joint_indices is None:
            return
        cur = self._parent._joint_angles.copy()
        positions = np.asarray(joint_positions, dtype=np.float64).flatten()
        indices = np.asarray(joint_indices, dtype=np.int64).flatten()
        for k, j in enumerate(indices):
            if 0 <= j < len(cur):
                cur[j] = float(positions[k])
        self._parent.set_joint_positions(cur)


class ManualM0609Driver:
    NUM_ARM_DOF = 6
    NUM_GRIP_DOF = 2  # finger_joint, right_inner_knuckle_joint
    NUM_DOF = NUM_ARM_DOF + NUM_GRIP_DOF

    AXIS_MAP = {"X": np.array([1.0, 0.0, 0.0]),
                "Y": np.array([0.0, 1.0, 0.0]),
                "Z": np.array([0.0, 0.0, 1.0])}

    def __init__(self, stage, m0609_path: str):
        """m0609_path: e.g. '/World/Vehicle/Vehicle/m0609' (m0609 의 직접 부모, base_link 의 parent)"""
        self.stage = stage
        self.m0609_path = m0609_path.rstrip("/")
        self.base_link_path = f"{self.m0609_path}/base_link"
        self.base_path = f"{self.m0609_path}/base"
        self.link_paths = [f"{self.m0609_path}/link_{i+1}" for i in range(6)]
        self.ee_path = self.link_paths[5]

        # angle_bracket (gripper base) — sibling subtree under same parent as m0609
        parent_path = self.m0609_path.rsplit("/", 1)[0]
        self.angle_bracket_path = f"{parent_path}/onrobot_rg2ft/angle_bracket"
        self.gripper_body_path  = f"{parent_path}/onrobot_rg2ft/gripper_body"

        # 1) Parse joint info from USD
        self._joint_axes = []
        self._joint_piv0 = []  # 4x4 column-major: pivot in body0 (parent) frame
        self._joint_piv1 = []  # 4x4 column-major: pivot in body1 (child) frame
        for i in range(6):
            self._parse_joint(f"{self.m0609_path}/joints/joint_{i+1}")
        if len(self._joint_axes) != 6:
            raise RuntimeError(f"M0609 joints parse 실패: 6개 중 {len(self._joint_axes)}개만 읽음")

        # 2) Cache base_link → base 의 fixed offset (joint 가 0 인 초기 상태)
        T_bl = self._read_world_mat(self.base_link_path)
        T_b = self._read_world_mat(self.base_path)
        self._T_baselink_to_base = np.linalg.inv(T_bl) @ T_b

        # 3) Cache link_6 → angle_bracket / gripper_body 의 fixed offset
        T_l6 = self._read_world_mat(self.ee_path)
        T_ab = self._read_world_mat(self.angle_bracket_path)
        T_gb = self._read_world_mat(self.gripper_body_path)
        self._T_link6_to_ab = np.linalg.inv(T_l6) @ T_ab
        self._T_link6_to_gb = np.linalg.inv(T_l6) @ T_gb

        # 4) Joint angles state (radians)
        self._joint_angles = np.zeros(self.NUM_DOF, dtype=np.float64)

        # 5) Init 시점엔 FK 를 적용하지 않음 — USD 원본 xform (복합 op 스택) 그대로 유지.
        #    set_joint_positions 가 처음 호출될 때 비로소 _apply_fk() 가 link xform 을 override.
        #    이를 위해 "FK 가 한 번이라도 적용됐는지" 플래그를 둠.
        self._fk_applied_once = False
        print(f"[FK driver] init OK — m0609={self.m0609_path}")
        print(f"           gripper={self.angle_bracket_path}")
        print(f"           (FK 적용 보류 — set_joint_positions 호출 시 활성화)")

    # ───────────────────────── SingleArticulation 인터페이스 mimic ─────────────────────────
    @property
    def num_dof(self) -> int:
        return self.NUM_DOF

    @property
    def dof_names(self):
        return [f"joint_{i+1}" for i in range(6)] + ["finger_joint", "right_inner_knuckle_joint"]

    @property
    def body_names(self):
        # IK 가 ee_body_index 로 Jacobian 첫 axis 를 indexing 하므로 link_6 하나만 노출 → ee_body_index = 0
        return ["link_6"]

    def get_joint_positions(self):
        return self._joint_angles.copy().astype(np.float32)

    def set_joint_positions(self, positions):
        positions = np.asarray(positions, dtype=np.float64).flatten()
        n = min(self.NUM_DOF, len(positions))
        # 변경량이 미미하면 (≤1e-6 rad) FK 건너뜀 — USD 원본 자세 유지
        if self._fk_applied_once or np.any(np.abs(positions[:n] - self._joint_angles[:n]) > 1e-6):
            self._joint_angles[:n] = positions[:n]
            self._apply_fk()
            self._fk_applied_once = True
        else:
            self._joint_angles[:n] = positions[:n]

    def get_jacobians(self):
        """Geometric Jacobian for link_6 wrt 8 DoF — shape (1, 6, 8). Gripper 컬럼은 0."""
        J6x6 = self._compute_geometric_jacobian()  # arm 6 joints → ee 6-vector
        full = np.zeros((1, 6, self.NUM_DOF), dtype=np.float64)
        full[0, :, :6] = J6x6
        return full

    def compute_link6_pose_at_angles(self, arm_angles_rad):
        """Non-destructive — USD 변경 없이 주어진 arm 각도에서 link_6 의 world (pos, quat)
        를 FK 로 계산해 반환. Manipulation 진입 전 HOME quat lock 캡쳐용."""
        arm_angles = np.asarray(arm_angles_rad, dtype=np.float64).flatten()[:6]
        T_bl = self._read_world_mat(self.base_link_path)
        T = T_bl @ self._T_baselink_to_base
        for i in range(6):
            R = _rot_about_axis(self._joint_axes[i], arm_angles[i])
            T = T @ self._joint_piv0[i] @ R @ np.linalg.inv(self._joint_piv1[i])
        # T 는 link_6 의 world pose. 위치 + 쿼터니언 추출
        pos = T[:3, 3].copy()
        R = T[:3, :3]
        # rotation matrix → quaternion (w, x, y, z)
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 2.0 * np.sqrt(trace + 1.0)
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return pos, np.array([qw, qx, qy, qz], dtype=np.float64)

    def get_articulation_controller(self):
        return _StubController(self)

    # ───────────────────────────────── 내부 ─────────────────────────────────
    def _parse_joint(self, joint_path: str):
        prim = self.stage.GetPrimAtPath(joint_path)
        if not prim.IsValid():
            raise RuntimeError(f"joint prim not found: {joint_path}")
        joint = UsdPhysics.RevoluteJoint(prim)
        axis_str = joint.GetAxisAttr().Get() if joint.GetAxisAttr() else "X"
        axis = self.AXIS_MAP.get(axis_str, self.AXIS_MAP["X"])
        p0 = joint.GetLocalPos0Attr().Get() if joint.GetLocalPos0Attr() else Gf.Vec3f(0, 0, 0)
        r0 = joint.GetLocalRot0Attr().Get() if joint.GetLocalRot0Attr() else Gf.Quatf(1, 0, 0, 0)
        p1 = joint.GetLocalPos1Attr().Get() if joint.GetLocalPos1Attr() else Gf.Vec3f(0, 0, 0)
        r1 = joint.GetLocalRot1Attr().Get() if joint.GetLocalRot1Attr() else Gf.Quatf(1, 0, 0, 0)
        self._joint_axes.append(axis)
        self._joint_piv0.append(_pose_to_mat(p0, r0))
        self._joint_piv1.append(_pose_to_mat(p1, r1))

    def _read_world_mat(self, prim_path: str) -> np.ndarray:
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"prim not found: {prim_path}")
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return _gf_to_np(m)

    def _set_world_mat(self, prim_path: str, mat: np.ndarray):
        """link 의 USD xform 을 set 해서 world pose = mat 이 되도록.
        부모의 world xform 을 읽어 local = parent_inv @ mat 계산 후, 단일 TransformOp 적용."""
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return
        parent = prim.GetParent()
        T_parent = _gf_to_np(
            UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        )
        local = np.linalg.inv(T_parent) @ mat
        gf_local = _np_to_gf(local)

        xf = UsdGeom.Xformable(prim)
        # 기존 op order 제거 후 단일 transform op 만 사용
        xf.ClearXformOpOrder()
        ops = xf.GetOrderedXformOps()  # 이미 추가된 op (이전 FK 적용 잔재) 가 있을 수 있음
        transform_op = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform and op.GetOpName() == "xformOp:transform:fk":
                transform_op = op
                break
        if transform_op is None:
            transform_op = xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, opSuffix="fk")
        transform_op.Set(gf_local)
        xf.SetXformOpOrder([transform_op])

    def _apply_fk(self):
        # 시작: base_link 의 world pose (USD-managed via kinematic parent /World/Vehicle)
        T_bl = self._read_world_mat(self.base_link_path)
        # base_link → base 의 고정 오프셋
        T = T_bl @ self._T_baselink_to_base
        # base 도 xform set (보통은 base_link 와 같이 움직이지만 명시적으로)
        self._set_world_mat(self.base_path, T)
        # joint_1 ~ joint_6
        for i in range(6):
            piv0 = self._joint_piv0[i]
            piv1 = self._joint_piv1[i]
            R = _rot_about_axis(self._joint_axes[i], self._joint_angles[i])
            T = T @ piv0 @ R @ np.linalg.inv(piv1)
            # T 는 link_{i+1} 의 world pose
            self._set_world_mat(self.link_paths[i], T)
        # link_6 → angle_bracket, gripper_body
        T_ab = T @ self._T_link6_to_ab
        T_gb = T @ self._T_link6_to_gb
        self._set_world_mat(self.angle_bracket_path, T_ab)
        self._set_world_mat(self.gripper_body_path, T_gb)

    def _compute_geometric_jacobian(self) -> np.ndarray:
        """현재 joint angle 에서 link_6 의 6×6 geometric Jacobian (arm joints only)."""
        # FK 를 다시 진행하면서 각 joint 의 world pivot 위치 (p_i) 와 axis (ω_i) 를 기록
        T_bl = self._read_world_mat(self.base_link_path)
        T = T_bl @ self._T_baselink_to_base
        p_joints = []
        omega_joints = []
        for i in range(6):
            piv0 = self._joint_piv0[i]
            piv1 = self._joint_piv1[i]
            T_pivot = T @ piv0
            p_joints.append(T_pivot[:3, 3].copy())
            R_world = T_pivot[:3, :3]
            omega_world = R_world @ self._joint_axes[i]
            omega_joints.append(omega_world.copy())
            R = _rot_about_axis(self._joint_axes[i], self._joint_angles[i])
            T = T @ piv0 @ R @ np.linalg.inv(piv1)
        p_ee = T[:3, 3]
        J = np.zeros((6, 6), dtype=np.float64)
        for i in range(6):
            J[:3, i] = np.cross(omega_joints[i], p_ee - p_joints[i])
            J[3:6, i] = omega_joints[i]
        return J
