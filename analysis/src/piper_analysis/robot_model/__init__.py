"""标称机器人模型；导入不读取URDF、不操作硬件。"""
from __future__ import annotations
from .types import RobotModel
from .urdf import load_piper
from .payload import with_payload

