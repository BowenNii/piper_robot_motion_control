"""旧入口兼容；新入口experiments.project1.pipeline。"""
from __future__ import annotations
from .experiments.project1.pipeline import analyze, DEFAULT_ROOT
from .experiments.project1.baseline import compare_baseline
from .experiments.project1.stream import errors
