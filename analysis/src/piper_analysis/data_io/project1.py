"""ABC实验路径重定位、CSV适配与来源记录。只读输入，不计算实验指标。"""
from pathlib import Path

import numpy as np

from ..common.records import FeedbackRecord
from ..validation.timing import strict_time
from .csv import columns, read_numeric
from .feedback import feedback
from .provenance import sha256


def load_record(root: Path, metadata: dict) -> FeedbackRecord:
    """读取一段项目1 ABC实验，整理时间基准、关节与位姿数据及来源摘要。
    Args:
        root: 实验根目录的 Path 对象，不是代码仓库根目录。
        metadata: inventory.json 的 records 列表中的一份完整记录字典。
    Returns:
        FeedbackRecord 对象，可通过 data.字段名读取：
        - metadata：传入的实验记录字典。
        - time_s：形状 (N,)，反馈绝对时间减去轨迹启动绝对时间，单位 s；启动前的数据可以为负数，不做延迟补偿。
        - q_rad：形状 (N, 6)，CAN反馈关节角，单位 rad。
        - poses：形状 (N, 4, 4)，读取 feedback_fk.csv 的 poe 列构造的位姿，即反馈关节角经模型FK回算的位姿，
        位置单位 m；不是外部实测位姿，也不是厂家反馈位姿列。
        - command_time_s、command_q_rad：形状 (K,) 和 (K, 6)，指令发布返回时刻相对轨迹启动的时间（s）及指令关节角（rad）。
        - q_start_rad、q_goal_rad：清单中的六轴起点、目标角，形状 (6,)。
        - active_joints：起终角差绝对值大于 1e-6 rad 的关节下标，从0开始。
        - duration_s：清单中记录的计划时长，单位 s，不是实测到位时间。
        - frame_span_ms：每组反馈六帧的时间跨度，单位 ms。
        - command_lateness_ms：实际发布开始时间减计划目标时间，单位 ms。
        - receive_gap_ms：ROS反馈相邻接收时间间隔，单位 ms；来自独立反馈序列，不保证长度与 CAN 完整帧组序列相同。
        - source_hashes：三个输入文件的相对路径到 SHA256 摘要的字典。
    Raises:
        OSError: 所需的 feedback_fk.csv、commands.csv 或 feedback.csv 不存在、不可读，或文件读取失败。
        KeyError: metadata 缺少必要字段。
        ValueError: 必需数据列缺失、数据为空或数值不合法，时间不严格递增、相对时间与绝对时间不一致，或位姿旋转矩阵不合法等。
    """
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
    received = read_numeric(inputs[2], ['receive_elapsed_s'])#读取 ROS 反馈记录
    strict_time(received['receive_elapsed_s'])#检查反馈接收时间
    start = metadata['execution_summary']['trajectory_start_epoch_s']#取出轨迹开始的绝对时间
    command_time = commands['publish_return_epoch_s'] - start#计算指令的轨迹相对时间
    if len(command_time) > 1:
        strict_time(command_time)
    manifest = metadata['manifest']
    q_start = np.array(manifest['q_start_rad'])
    q_goal = np.array(manifest['q_goal_rad'])
    return FeedbackRecord(
        metadata=metadata,
        time_s=data['timestamp_s'] - start,#实际反馈的采样时间轴
        q_rad=q,
        poses=poses,
        command_time_s=command_time,#指令发布返回的时间轴
        command_q_rad=columns(commands, q_columns),
        q_start_rad=q_start,
        q_goal_rad=q_goal,
        active_joints=np.flatnonzero(abs(q_goal - q_start) > 1e-6),
        duration_s=manifest['duration_s'],#计划运动时长
        frame_span_ms=data['frame_span_s'] * 1000,#一组关节反馈帧的时间跨度
        command_lateness_ms=(
            commands['publish_start_elapsed_s'] - commands['target_time_s']
        ) * 1000,#实际开始发布指令，比计划发布时刻晚了多久。
        receive_gap_ms=np.diff(received['receive_elapsed_s']) * 1000,#相邻反馈的接收间隔
        source_hashes=source_hashes,
    )
