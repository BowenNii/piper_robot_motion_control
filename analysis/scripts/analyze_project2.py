#!/usr/bin/env python3
"""分析已整理的Project2关节状态CSV；不读取CAN、不发送命令。"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from piper_analysis.experiments.project2.pipeline import analyze_csv

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--measured-torque-verified', action='store_true')
    a = p.parse_args()
    print(analyze_csv(a.source, a.output, a.measured_torque_verified))
