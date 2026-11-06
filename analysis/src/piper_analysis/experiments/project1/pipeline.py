"""ABC收尾总流程：只组织读取、计算、验证、导出与绘图。"""
from __future__ import annotations
import json
from pathlib import Path
from ...common.paths import REPO
from ...robot_model.urdf import load_piper
from ...data_io.config import load_config
from ...data_io.csv import read_numeric
from ...data_io.project1 import load_record
from .record import analyze_record
from .stream import evaluate_stream
from .export import export_trace, export_tables, export_provenance
from .baseline import compare_baseline


DEFAULT_ROOT = REPO / 'experiments/project1/2026-10-27_movej_repeat'


def analyze(root=DEFAULT_ROOT, output=None, config=None, make_plots: bool = True):
    root = Path(root)
    base = root / 'project1_closeout'
    output = Path(output) if output else base / 'python'
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite: {output}')
    cfg = load_config(config or REPO / 'config/project1_analysis.json')
    inventory = json.loads((base / 'inventory.json').read_text())
    model = load_piper()
    cache, responses, sources = [], [], {}
    output.mkdir(parents=True)
    (output / 'traces').mkdir()
    for number, metadata in enumerate(inventory['records'], 1):
        print(f"Python收尾分析 {number}/{len(inventory['records'])}", flush=True)
        data = load_record(root, metadata)
        result, response = analyze_record(data, model)
        cache.append(result)
        responses.extend(response)
        sources.update(data.source_hashes)
    for result in cache:#逐条处理 stream 实验，为它找到同组唯一的去程 stream 记录作为延迟来源，再进行评价并导出数据。
        if result['record']['manifest']['mode'] != 'stream':
            continue
        sources_for_delay = [candidate for candidate in cache
            if candidate['record']['group'] == result['record']['group']
            and candidate['record']['manifest']['mode'] == 'stream'
            and candidate['record']['manifest']['direction'] == 'outbound']
        if len(sources_for_delay) != 1:
            raise ValueError('Need unique outbound delay fit source')
        evaluate_stream(result, sources_for_delay[0], cfg)
        export_trace(result, output)
    export_tables(cache, responses, output)
    compare_baseline(output, base / 'matlab')
    export_provenance(output, sources, cfg, model)
    if make_plots:#根据开关决定是否绘图
        from ...plotting import draw_closeout
        draw_closeout(cache, responses, inventory['selected_record_numbers'],
                      read_numeric(base / 'decode_comparison.csv'), output)
    return output
