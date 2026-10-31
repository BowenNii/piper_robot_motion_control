"""前三阶段历史分析入口：只组织独立实验模块。"""
from __future__ import annotations
from pathlib import Path
import json
from ...robot_model.urdf import load_piper
from .static_fk import analyze_static
from .dynamic_fk import analyze_dynamic
from .ik_trajectory import analyze_ik


def analyze_history(output) -> None:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    model = load_piper()
    hashes = {}
    analyze_static(output / 'static_fk', model, hashes)
    analyze_dynamic(output / 'dynamic_fk', model, hashes)
    analyze_ik(output / 'ik_trajectory', model, hashes)
    (output / 'source_hashes.json').write_text(json.dumps(hashes, indent=2) + '\n')
