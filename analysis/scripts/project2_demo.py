#!/usr/bin/env python3
"""运行Project2离线合成演示；不连接机械臂。"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from piper_analysis.common.paths import REPO
from piper_analysis.experiments.project2.demo import run_demo


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=REPO / 'experiments/project2/nominal_demo_python')
    args = parser.parse_args()
    print(f'离线演示完成：{run_demo(args.output)}，不是项目2真机验收。')


if __name__ == '__main__':
    main()
