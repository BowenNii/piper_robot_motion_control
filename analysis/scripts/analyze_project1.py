#!/usr/bin/env python3
"""项目1统一离线入口；默认迁移全部四阶段，绝不控制真机。"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from piper_analysis.experiments.project1.pipeline import analyze, DEFAULT_ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--closeout-only', action='store_true')
    args = parser.parse_args()
    output = analyze(args.root, args.output, args.config)
    if not args.closeout_only:
        from piper_analysis.experiments.project1.history import analyze_history
        analyze_history(output/'history')
    print(f'完成，原始数据及MATLAB结果未修改：{output}')


if __name__ == '__main__':
    main()
