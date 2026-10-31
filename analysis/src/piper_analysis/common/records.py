"""数据层与实验层共用的序列类型；单位写进字段名，数组按(N,...)排列。"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .types import Array


@dataclass(frozen=True)
class FeedbackRecord:
    """一段ABC实验的只读数据载体，不在这里进行指标计算或文件写入。"""

    metadata: dict[str, Any]
    time_s: Array
    q_rad: Array
    poses: Array
    command_time_s: Array
    command_q_rad: Array
    q_start_rad: Array
    q_goal_rad: Array
    active_joints: Array
    duration_s: float
    frame_span_ms: Array
    command_lateness_ms: Array
    receive_gap_ms: Array
    source_hashes: dict[str, str]


@dataclass(frozen=True)
class JointStateSeries:
    """Project2状态序列；measured_torque_Nm为None表示未提供已确认力矩。"""

    source: Path
    time_s: Array
    q_rad: Array
    dq_rad_s: Array
    ddq_rad_s2: Array
    measured_torque_Nm: Array | None = None
