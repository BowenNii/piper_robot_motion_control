"""CAN导出CSV到关节/位姿序列的适配与格式检查。"""
from __future__ import annotations
import numpy as np
from .csv import read_numeric, columns
from ..validation.timing import strict_time

def feedback(path, strict=True):
    """读取 CAN 解码 CSV，校验时间与旋转，返回原始列、关节角和FK位姿。
    Args:
        path: CSV 文件路径（字符串或 Path），例如 feedback_fk.csv。
            必须包含 t_s、timestamp_s、frame_span_s、q1_rad～q6_rad、
            poe_x_m～poe_z_m 以及 poe_r11～poe_r33 数值列。
        strict: 是否要求时间严格递增，默认 True，且至少需要两个样本。
            False 时允许相同时间和单个样本，但仍拒绝时间倒退；
            不代表关闭其他数据检查。
    Returns:
        三元组 (d, q, T)，可写为 data, q, poses = feedback(path)：
        - d：按列名索引的 CSV 数据字典，例如 d['timestamp_s']。
          时间单位 s，关节角单位 rad，位置单位 m；不改变原始时间基准。
        - q：形状 (N, 6) 的反馈关节角数组，单位 rad。
        - T：形状 (N, 4, 4) 的齐次位姿矩阵序列，由 poe 列重建；
          表示反馈关节角经模型 FK 回算的位姿，不是外部实测位姿，
          也不是 CSV 中厂家反馈 XYZ/RPY 列。位置单位 m。
        N 为 CSV 数据行数；本函数不重新计算 FK，不滤波、重采样、
        延迟补偿或筛除超过 2ms 的帧组。
    Raises:
        OSError: 文件不存在、不可读或发生其他文件读取错误。
        ValueError: 表头非法、必需列缺失、数据为空、必需数值非有限，
            时间顺序不符合 strict 设置，或相对/绝对时间增量差超过
            1e-6 s、六帧跨度为负、旋转矩阵不正交或行列式不接近 +1。
    """
    names = ['t_s', 'timestamp_s', 'frame_span_s']+[f'q{i}_rad' for i in range(1, 7)]
    names += [f'poe_{a}_m' for a in 'xyz']+[f'poe_r{i}{j}' for i in range(1, 4) for j in range(1, 4)]
    d = read_numeric(path, names)
    if strict:
        strict_time(d['timestamp_s'])
        strict_time(d['t_s'])
    elif np.any(np.diff(d['timestamp_s']) < 0) or np.any(np.diff(d['t_s']) < 0):
        raise ValueError("Time moved backward")
    if np.max(np.abs((d['t_s']-d['t_s'][0])-(d['timestamp_s']-d['timestamp_s'][0]))) > 1e-6:
        raise ValueError("Relative/epoch time inconsistent")
    if np.any(d['frame_span_s'] < 0):
        raise ValueError("Negative CAN span")
    q = columns(d, [f'q{i}_rad' for i in range(1, 7)])
    T = np.repeat(np.eye(4)[None], len(q), axis=0)
    T[:, :3, :3] = columns(d, [f'poe_r{i}{j}' for i in range(1, 4) for j in range(1, 4)]).reshape(-1, 3, 3)
    T[:, :3, 3] = columns(d, [f'poe_{a}_m' for a in 'xyz'])
    if not np.allclose(T[:, :3, :3].transpose(0, 2, 1) @ T[:, :3, :3], np.eye(3), atol=1e-7, rtol=0):
        raise ValueError("Invalid rotation matrices")
    if not np.allclose(np.linalg.det(T[:, :3, :3]), 1, atol=1e-7, rtol=0):
        raise ValueError("Reflection instead of SO(3)")
    return d, q, T
