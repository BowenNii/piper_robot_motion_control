#!/usr/bin/env python3
"""Project1收尾的离线数据入口：重新按CAN周期解码，不连接ROS/CAN。

原始日志/命令/反馈不改动；新目录project1_closeout不允许覆盖。
旧结果仅用于逐项对照，不作为新分析的输入数值。
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

REPO=Path(__file__).resolve().parents[2]
GROUPS=('j1_0p5','j4_0p5','j5_0p5','j6_0p5','multi_abc')

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def load_baseline(path):
    """只读已保存的历史对照列；无文件则返回空，不把新解码冒充旧结果。"""
    if not path.is_file():return {}
    result={}
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            key=(row['group'],row['run_name'])
            if key in result:raise ValueError('历史对照包含重复实验条件')
            result[key]=dict(fk_samples=float(row['old_samples']),
                span_over_2ms=float(row['old_over_2ms']),max_span_s=float(row['old_max_span_ms'])/1000)
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=REPO/'experiments/project1/2026-10-27_movej_repeat')
    parser.add_argument('--output-dir',type=Path,help='可选新输出目录，不覆盖正式结果')
    parser.add_argument('--baseline-comparison',type=Path,help='历史decode_comparison.csv；默认读取正式结果内保存的表')
    args=parser.parse_args();root=args.root.resolve();out=(args.output_dir or root/'project1_closeout').resolve()
    if out.exists():raise ValueError('project1_closeout已存在，不覆盖')
    baseline=load_baseline(args.baseline_comparison or root/'project1_closeout/decode_comparison.csv')
    out.mkdir();(out/'decoded').mkdir();records=[];quality=[]
    sources=[]
    for group in GROUPS:
        sources.extend((group,p.parent) for p in sorted((root/group).glob('*/execution_*/summary.json')))
    if len(sources)!=34:raise ValueError(f'预期34段，实际{len(sources)}；请核对实验范围')
    for k,(group,e) in enumerate(sources,1):
        print(f'[{k}/34] {group}/{e.parent.name}',flush=True)
        meta=json.loads((e/'summary.json').read_text());manifest=json.loads((e.parent/'manifest.json').read_text())
        assert meta['success']
        paths=[e/n for n in ('raw_can.log','commands.csv','feedback.csv','status.csv','summary.json')]
        assert all(p.is_file() and p.stat().st_size for p in paths)
        hashes={str(p.relative_to(root)):digest(p) for p in paths}
        dest=out/'decoded'/f'{k:02d}_{group}_{e.parent.name}'
        subprocess.run([str(REPO/'build/debug/project1_feedback_export'),str(e/'raw_can.log'),str(dest)],check=True)
        for p in paths:assert digest(p)==hashes[str(p.relative_to(root))]
        new=json.loads((dest/'decode_summary.json').read_text())
        old_path=e/'decoded/decode_summary.json'
        old=json.loads(old_path.read_text()) if old_path.is_file() else baseline.get((group,e.parent.name),dict(
            fk_samples=math.nan,span_over_2ms=math.nan,max_span_s=math.nan))
        assert new['ordered_cycles'] and new['invalid_lines']==0
        quality.append(dict(record=k,group=group,run_name=e.parent.name,
            old_samples=old['fk_samples'],old_over_2ms=old['span_over_2ms'],old_max_span_ms=old['max_span_s']*1000,
            new_samples=new['fk_samples'],new_over_2ms=new['span_over_2ms'],
            new_over_2ms_fraction=new['span_over_2ms']/new['fk_samples'],new_max_span_ms=new['max_span_s']*1000,
            unaligned_frames=new['unaligned_frames'],cycle_resets=new['cycle_resets'],pending_frames=new['pending_frames']))
        records.append(dict(record=k,group=group,execution_dir=str(e),decoded_dir=str(dest),
            run_name=e.parent.name,manifest=manifest,execution_summary=meta,decoder=new,raw_hashes=hashes))
    selected=[]
    for group in GROUPS:
        for mode in ('once','repeat','stream'):
            for direction in ('outbound','return'):
                match=[r for r in records if r['group']==group and r['manifest']['mode']==mode
                       and r['manifest']['direction']==direction]
                selected.append(max(match,key=lambda r:r['run_name'])['record'])
    payload=dict(records=records,selected_record_numbers=selected,
        assembler_policy='Ordered observed bursts 0x2A2..0x2A7; discard incomplete prefix/sequence, report every discard; 50ms ceiling, 2ms quality flag.',
        scope='Feedback-derived link6 poses, not external measurement. No motor current/velocity diagnosis.')
    (out/'inventory.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2))
    with (out/'decode_comparison.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(quality[0]));w.writeheader();w.writerows(quality)
    print('完成：',out)

if __name__=='__main__':main()
