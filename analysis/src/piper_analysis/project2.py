"""旧入口兼容；新入口experiments.project2.pipeline。"""
from __future__ import annotations
from .experiments.project2.pipeline import analyze_csv
from .dynamics.batch import torque_prediction
from .identification.regressor import stack_regressors
