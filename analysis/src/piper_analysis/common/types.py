"""数组类型别名：SI单位，序列按(N,...)排列。"""
from __future__ import annotations
import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]

