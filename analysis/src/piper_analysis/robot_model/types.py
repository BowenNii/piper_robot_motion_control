"""串联模型数据结构；Mlist按(link,4,4)，Glist按(link,6,6)。"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class RobotModel:
    S: np.ndarray       # (6,n), base frame home screws
    Mlist: np.ndarray   # (n+1,4,4), home parent->child, then tool transform
    Glist: np.ndarray   # (n,6,6), link-frame spatial inertias
    gravity: np.ndarray
    q_min: np.ndarray
    q_max: np.ndarray
    velocity_max: np.ndarray
    effort_max: np.ndarray
    source: str = "synthetic"

    @property
    def n(self) -> int:
        return self.S.shape[1]

    @property
    def home(self) -> np.ndarray:
        T = np.eye(4)
        for M in self.Mlist:
            T = T @ M
        return T
