"""ABC实验路径重定位、CSV适配与来源记录。只读输入，不计算实验指标。"""
from pathlib import Path

import numpy as np

from ..common.records import FeedbackRecord
from ..validation.timing import strict_time
from .csv import columns, read_numeric
from .feedback import feedback
from .provenance import sha256


def load_record(root: Path, metadata: dict) -> FeedbackRecord:
    """按当前仓库重定位历史清单路径，时间统一为相对轨迹启动的秒数。"""
    base = root / 'project1_closeout'
    execution = (
        root / metadata['group'] / metadata['run_name']
        / Path(metadata['execution_dir']).name
    )
    decoded = base / 'decoded' / Path(metadata['decoded_dir']).name / 'feedback_fk.csv'
    inputs = [decoded, execution / 'commands.csv', execution / 'feedback.csv']
    source_hashes = {str(path.relative_to(root)): sha256(path) for path in inputs}
    data, q, poses = feedback(decoded)
    q_columns = [f'q{i}_rad' for i in range(1, 7)]
    commands = read_numeric(
        inputs[1],
        ['publish_return_epoch_s', 'publish_start_elapsed_s', 'target_time_s'] + q_columns,
    )
    received = read_numeric(inputs[2], ['receive_elapsed_s'])
    strict_time(received['receive_elapsed_s'])
    start = metadata['execution_summary']['trajectory_start_epoch_s']
    command_time = commands['publish_return_epoch_s'] - start
    if len(command_time) > 1:
        strict_time(command_time)
    manifest = metadata['manifest']
    q_start = np.array(manifest['q_start_rad'])
    q_goal = np.array(manifest['q_goal_rad'])
    return FeedbackRecord(
        metadata=metadata,
        time_s=data['timestamp_s'] - start,
        q_rad=q,
        poses=poses,
        command_time_s=command_time,
        command_q_rad=columns(commands, q_columns),
        q_start_rad=q_start,
        q_goal_rad=q_goal,
        active_joints=np.flatnonzero(abs(q_goal - q_start) > 1e-6),
        duration_s=manifest['duration_s'],
        frame_span_ms=data['frame_span_s'] * 1000,
        command_lateness_ms=(
            commands['publish_start_elapsed_s'] - commands['target_time_s']
        ) * 1000,
        receive_gap_ms=np.diff(received['receive_elapsed_s']) * 1000,
        source_hashes=source_hashes,
    )
