"""D455 (rsd455.usd) + 의존 외형 MDL·텍스처를 Nucleus 에서 repo 로 로컬화.

rsd455.usd 는 외형 MDL 3개(Aluminum_Anodized/Cast, Plastic_ABS)를
'../../../Materials/...' 상대경로로 참조하고, 각 MDL 은 다시 옆 디렉토리의
텍스처 PNG 를 참조한다. rsd455.usd 만 옮기면 전부 깨진다.

이 스크립트는:
  1. rsd455.usd 를 isaac_sim/assets/d455/ 로 복사
  2. 외형 MDL 3개를 isaac_sim/assets/d455/materials/ 로 복사
  3. 각 MDL 의 텍스처 디렉토리를 d455/materials/<name>/ 로 복사
     (MDL 은 텍스처를 옆 디렉토리 상대경로로 참조 — 구조 유지하면 됨)
  4. rsd455.usd 의 MDL sourceAsset 경로를 'materials/<name>.mdl' 로 재작성
한다. OmniPBR/OmniGlass 등 코어 MDL 은 search path 로 해석되므로 제외.

isaac-python 으로 headless 실행:
    <isaac-python> isaac_sim/scripts/localize_d455.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

from pathlib import Path

import omni.client
from pxr import Usd, Sdf

try:
    from isaacsim.storage.native import get_assets_root_path
except ImportError:
    from omni.isaac.core.utils.nucleus import get_assets_root_path

_ISAAC_SIM = Path(__file__).resolve().parents[1]
DST_DIR = _ISAAC_SIM / "assets" / "d455"
RSD455_DST = DST_DIR / "rsd455.usd"
MAT_DIR = DST_DIR / "materials"

RSD455_REL = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"

# rsd455.usd 가 참조하는 외형 MDL (Nucleus 상대경로 → 로컬 파일명)
MDL_SRCS = {
    "/Isaac/Materials/Base/Metals/Aluminum_Anodized.mdl": "Aluminum_Anodized.mdl",
    "/Isaac/Materials/Base/Metals/Aluminum_Cast.mdl": "Aluminum_Cast.mdl",
    "/Isaac/Materials/Base/Plastics/Plastic_ABS.mdl": "Plastic_ABS.mdl",
}

# 각 MDL 옆의 텍스처 디렉토리 (MDL 이 'Name/Name_*.png' 상대로 참조)
TEX_DIRS = {
    "/Isaac/Materials/Base/Metals/Aluminum_Anodized": "Aluminum_Anodized",
    "/Isaac/Materials/Base/Metals/Aluminum_Cast": "Aluminum_Cast",
    "/Isaac/Materials/Base/Plastics/Plastic_ABS": "Plastic_ABS",
}


def _copy(src, dst):
    result = omni.client.copy(src, str(dst), omni.client.CopyBehavior.OVERWRITE)
    ok = Path(dst).is_file()
    print(f"  {'OK ' if ok else 'FAIL'} ({result}) → {dst}", flush=True)
    return ok


def _copy_dir(src_dir, dst_dir):
    """src_dir(Nucleus) 의 파일을 dst_dir 로 복사. 디렉토리 없으면 0 반환."""
    result, entries = omni.client.list(src_dir)
    if result != omni.client.Result.OK:
        print(f"  (텍스처 디렉토리 없음/접근불가: {src_dir} — {result})", flush=True)
        return 0
    Path(dst_dir).mkdir(parents=True, exist_ok=True)
    n = 0
    for e in entries:
        name = e.relative_path
        # .thumbs 등 숨김 디렉토리/항목은 건너뛴다 (썸네일 캐시).
        if not name or name.startswith(".") or name.endswith("/"):
            continue
        if _copy(f"{src_dir}/{name}", Path(dst_dir) / name):
            n += 1
    return n


def main():
    root = get_assets_root_path()
    if not root:
        print("[localize] ✗ assets root 없음 — Nucleus 연결 실패", flush=True)
        simulation_app.close()
        return

    DST_DIR.mkdir(parents=True, exist_ok=True)
    MAT_DIR.mkdir(parents=True, exist_ok=True)

    # 1) rsd455.usd
    print("[localize] rsd455.usd 복사 …", flush=True)
    _copy(root + RSD455_REL, RSD455_DST)

    # 2) 외형 MDL 3개
    print("[localize] 외형 MDL 복사 …", flush=True)
    for src_rel, fname in MDL_SRCS.items():
        _copy(root + src_rel, MAT_DIR / fname)

    # 3) MDL 텍스처 디렉토리
    print("[localize] MDL 텍스처 디렉토리 복사 …", flush=True)
    for src_rel, name in TEX_DIRS.items():
        cnt = _copy_dir(root + src_rel, MAT_DIR / name)
        print(f"  [{name}] 텍스처 {cnt}개", flush=True)

    # 4) rsd455.usd 의 MDL sourceAsset 경로 재작성
    print("[localize] rsd455.usd MDL 경로 재작성 …", flush=True)
    stage = Usd.Stage.Open(str(RSD455_DST))
    local_names = set(MDL_SRCS.values())
    fixed = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Shader":
            continue
        attr = prim.GetAttribute("info:mdl:sourceAsset")
        if not attr:
            continue
        val = attr.Get()
        if not val:
            continue
        fname = val.path.rsplit("/", 1)[-1]
        if fname in local_names:
            attr.Set(Sdf.AssetPath(f"materials/{fname}"))
            fixed += 1
    stage.GetRootLayer().Save()
    print(f"[localize] ✓ MDL sourceAsset {fixed}개 → materials/", flush=True)

    print("[localize] 완료", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
