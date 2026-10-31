#!/usr/bin/env python3
"""例：python analysis/scripts/view_project1.py 9，工具栏放大/平移，底部显示坐标。"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from piper_analysis.experiments.project1.pipeline import DEFAULT_ROOT
from piper_analysis.plotting import view

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('number', type=int, choices=range(1, 11))
    p.add_argument('--output', type=Path, default=DEFAULT_ROOT/'project1_closeout/python')
    a = p.parse_args()
    view(a.output, a.number)
