"""main.py 에서 viewer.py 로 상태를 전달하는 npz writer + viewer subprocess 관리."""
import os
import subprocess
import tempfile
import numpy as np


SYS_PYTHON = "/usr/bin/python3"
DATA_FILENAME = "starcraft_map_state.npz"
VIEWER_LOG = "/tmp/starcraft_map_viewer.log"


class StateWriter:
    def __init__(self, fog_map, viewer_script_path, write_every=3,
                 rover_id: str = "", spawn_viewer: bool = True):
        """
        Args:
            fog_map: FogMap 인스턴스 (obstacle_mask 격자 포함)
            viewer_script_path: viewer.py 경로
            write_every: 매 N step 마다 저장
            rover_id: 다중 rover 시 파일명 구분용 (예: "rover_1"). 비우면 기존 단일 파일.
            spawn_viewer: False 면 파일만 쓰고 viewer subprocess 안 띄움 (다중 rover 시 첫 rover 만 True).
        """
        self.fog_map = fog_map
        self.write_every = int(write_every)
        # 다중 rover — 각자 별도 파일 (viewer 가 모두 overlay 로 읽음)
        if rover_id:
            self.data_path = os.path.join(
                tempfile.gettempdir(),
                DATA_FILENAME.replace(".npz", f"_{rover_id}.npz"))
        else:
            self.data_path = os.path.join(tempfile.gettempdir(), DATA_FILENAME)

        # 이전 캐시 제거
        for p in (self.data_path, self.data_path + ".tmp.npz"):
            if os.path.exists(p):
                try:
                    os.remove(p)
                except OSError:
                    pass

        # 다중 rover 모드 (rover_id 가 설정됨) → viewer 에 glob pattern 전달해서
        # 한 viewer 가 모든 rover 의 state file 을 동시 overlay.
        if rover_id and spawn_viewer:
            self._viewer_arg = os.path.join(
                tempfile.gettempdir(),
                DATA_FILENAME.replace(".npz", "_*.npz"))
        else:
            self._viewer_arg = self.data_path
        self.viewer_proc = (self._spawn_viewer(viewer_script_path)
                            if spawn_viewer else None)

    def _spawn_viewer(self, viewer_script_path):
        if not os.path.exists(viewer_script_path):
            print(f"[viewer] 스크립트 없음: {viewer_script_path}")
            return None
        if not os.path.exists(SYS_PYTHON):
            print(f"[viewer] 시스템 파이썬 없음: {SYS_PYTHON}")
            return None

        clean_env = os.environ.copy()
        for var in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP",
                    "LD_PRELOAD", "LD_LIBRARY_PATH"):
            clean_env.pop(var, None)
        try:
            proc = subprocess.Popen(
                [SYS_PYTHON, viewer_script_path, self._viewer_arg],
                stdout=open(VIEWER_LOG, "w"),
                stderr=subprocess.STDOUT,
                env=clean_env,
            )
            print(f"[viewer] 시작 pid={proc.pid}  arg={self._viewer_arg}  "
                  f"(log={VIEWER_LOG})")
            return proc
        except Exception as e:
            print(f"[viewer] 시작 실패: {e}")
            return None

    def maybe_write(self, step_index, rover_xy_yaw, mission):
        if step_index % self.write_every != 0:
            return
        tmp = self.data_path + ".tmp.npz"

        # mission 이 정한 동선: 현재 A* 경로 + 남은 anchor 후보
        path = (np.array(mission.nav.path, dtype=np.float32)
                if mission.nav.path else np.zeros((0, 2), dtype=np.float32))
        candidates = (np.array(mission.anchor_queue, dtype=np.float32)
                      if mission.anchor_queue
                      else np.zeros((0, 2), dtype=np.float32))

        try:
            np.savez(
                tmp,
                rover=np.array(rover_xy_yaw, dtype=np.float32),
                fog=self.fog_map.fog.astype(np.uint8),
                obstacle_mask=self.fog_map.obstacle_mask.astype(np.uint8),
                map_size=np.array(
                    [self.fog_map.map_w, self.fog_map.map_h], dtype=np.float32
                ),
                cell_size=np.float32(self.fog_map.cell_size),
                reveal_radius=np.float32(self.fog_map.reveal_radius),
                grid_n=np.int32(self.fog_map.grid_n),
                sector_ratios=np.array(
                    self.fog_map.all_sector_ratios(), dtype=np.float32
                ),
                current_sector=np.int32(mission.current_sector),
                path=path,                              # (N,2) 현재 A* 경로
                path_idx=np.int32(mission.nav.idx),     # 추종 중 waypoint 인덱스
                candidates=candidates,                  # (M,2) 남은 anchor 후보
            )
            os.replace(tmp, self.data_path)
        except Exception as e:
            print(f"[viewer] 저장 실패: {e}")

    def close(self):
        if self.viewer_proc is not None:
            try:
                self.viewer_proc.terminate()
                self.viewer_proc.wait(timeout=2.0)
            except Exception:
                pass
            self.viewer_proc = None
