"""레거시 베이스캠프 시각 에셋용 호환 헬퍼."""

from __future__ import annotations

from pathlib import Path

from save_utils import write_placeholder_reference_asset


# 단순한 베이스캠프 참조 에셋을 만드는 레거시 헬퍼.
def build_basecamp_visual_asset(out_path: str | Path) -> Path:
    return write_placeholder_reference_asset(out_path, "Basecamp", (0.82, 0.84, 0.88))
