"""安全数值NPZ与绘图元数据，禁止pickle；不调用Matplotlib。"""
from __future__ import annotations
import numpy as np
from pathlib import Path
import json

def _json_safe(value):
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def save_plot_cache(cache, response, selected, quality, output) -> None:
    (output / 'figures').mkdir()
    (output / 'plot_data').mkdir()
    metadata = []
    for result in cache:
        number = result['record']['record']
        arrays = {key: value for key, value in result.items() if isinstance(value, np.ndarray)}
        np.savez_compressed(output / 'plot_data' / f'{number:02d}.npz', **arrays)
        metadata.append({key: result[key] for key in ('record', 'row', 'duration')})
    payload = dict(records=metadata, response=response, selected=selected,
                   quality={key: value.tolist() for key, value in quality.items()})
    (output / 'plot_data/index.json').write_text(
        json.dumps(_json_safe(payload), ensure_ascii=False, allow_nan=False))


def load_plot_cache(output):
    output = Path(output)
    metadata = json.loads((output / 'plot_data/index.json').read_text())
    cache = []
    for result in metadata['records']:
        filename = output / 'plot_data' / f"{result['record']['record']:02d}.npz"
        with np.load(filename, allow_pickle=False) as arrays:
            result.update({key: arrays[key] for key in arrays.files})
        cache.append(result)
    return (cache, metadata['response'], metadata['selected'],
            {key: np.array(value) for key, value in metadata['quality'].items()})
