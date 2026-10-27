#!/usr/bin/env python3
"""离线比较PiPER原始CAN与ROS反馈；不导入ROS、不连接CAN、不控制机器人。"""
import argparse
from bisect import bisect_right
from collections import Counter
import csv
import json
import math
from pathlib import Path
import re
import statistics

def read_table(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))

def quantile_middle(x):return statistics.median(x) if x else None

def characterize(series,start,end,direction=1):
    v=[(t,q) for t,q in series if start<=t<=end]
    delta=[v[i][1]-v[i-1][1] for i in range(1,len(v))]
    reverse=[(v[i][0],v[i][1]-v[i-1][1]) for i in range(1,len(v))
             if -direction*(v[i][1]-v[i-1][1])>.001]
    first=next(((t,q) for t,q in series if t>=0 and abs(q)>.001),None)
    return dict(samples=len(v),distinct_values=len(set(q for t,q in v)),
        max_step_rad=max(map(abs,delta),default=0),reverse_steps_over_001rad=len(reverse),
        reverse_examples=reverse[:8],first_change_over_001rad=first,
        median_interval_ms=quantile_middle([(v[i][0]-v[i-1][0])*1000 for i in range(1,len(v))]))

def analyze(folder,execution_name):
    execution=folder/execution_name
    summary=json.loads((execution/'summary.json').read_text())
    J=read_table(execution/'feedback.csv');C=read_table(execution/'commands.csv')
    offset=summary['trajectory_start_receive_elapsed_s']
    origin=statistics.median(float(r['header_stamp_s'])-float(r['receive_elapsed_s']) for r in J)+offset
    groups={i:[] for i in (0x2A5,0x2A6,0x2A7,0x155,0x156,0x157)}
    counts=Counter();unparsed=0;malformed=0
    pattern=re.compile(r'^\(([0-9.]+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)$')
    with (folder/'can_run02.log').open() as f:
        for line in f:
            match=pattern.match(line.strip())
            if not match:unparsed+=1;continue
            stamp,interface,can_id,data=match.groups();can_id=int(can_id,16);counts[hex(can_id)]+=1
            if can_id not in groups:continue
            if interface!='can0' or len(data)!=16:malformed+=1;continue
            payload=bytes.fromhex(data)
            q=[int.from_bytes(payload[k:k+4],'big',signed=True)*math.pi/180000 for k in (0,4)]
            groups[can_id].append((float(stamp),*q))
    assert all(groups[k] for k in (0x2A5,0x2A6,0x2A7)),'CAN记录缺少反馈组'
    assert all(all(b[0]>=a[0] for a,b in zip(v,v[1:])) for v in groups.values()),'CAN时间回退'
    comparison=[];decoded=[]
    for joint in range(1,7):
        group=(joint-1)//2;column=1+(joint-1)%2;sign=-1 if joint==3 else 1
        fb=groups[0x2A5+group];cmd=groups[0x155+group]
        raw=[(t-origin,row[column]) for row in fb for t in [row[0]]]
        sent=[(t-origin,row[column]) for row in cmd for t in [row[0]]]
        ros=[(float(row['receive_elapsed_s'])-offset,float(row[f'q{joint}_rad'])) for row in J]
        comparison.append(dict(joint=joint,raw_startup=characterize(raw,5,10,sign),
            ros_startup=characterize(ros,5,10,sign),command_startup=characterize(sent,5,10,sign),
            raw_motion=characterize(raw,0,60,sign)))
        for t,q in raw:
            if 0<=t<=60:decoded.append([t,joint,q,'feedback'])
        for t,q in sent:
            if 0<=t<=60:decoded.append([t,joint,q,'command'])
    # 用ROS header（来自joint56）定位近邻原始组；不是假定六轴同一硬件采样时刻。
    pair_checks=[]
    for joint in range(1,7):
        v=groups[0x2A5+(joint-1)//2];times=[row[0] for row in v];column=1+(joint-1)%2
        errors=[];ages=[]
        for row in J:
            stamp=float(row['header_stamp_s']);idx=bisect_right(times,stamp)-1
            if idx<0:continue
            age=stamp-times[idx]
            if age>.02:continue
            errors.append(abs(float(row[f'q{joint}_rad'])-v[idx][column]));ages.append(age)
        pair_checks.append(dict(joint=joint,samples=len(errors),same_value_count=sum(e<1e-10 for e in errors),
            max_difference_rad=max(errors,default=0),p50_pair_age_ms=statistics.median(ages)*1000))
    intervals=[float(b['send_elapsed_s'])-float(a['send_elapsed_s']) for a,b in zip(C,C[1:])]
    raw_intervals={}
    for k,v in groups.items():
        values=[(b[0]-a[0])*1000 for a,b in zip(v,v[1:]) if 0<=a[0]-origin and b[0]-origin<=60]
        if values:raw_intervals[hex(k)]=max(values)
    result=dict(execution=execution_name,success=summary['success'],sent_points=summary['sent_points'],
        final_joint_error_rad=summary['final_joint_error_max_rad'],epoch_origin_est_s=origin,
        clock_note='CAN相对时间用ROS头时间与单调接收时间中位差估计，微小传输偏差未消除',
        unparsed_lines=unparsed,malformed_selected_frames=malformed,can_id_counts=dict(counts),
        wire_command_frames_available=all(groups[k] for k in (0x155,0x156,0x157)),
        comparison=comparison,raw_to_ros_pair_checks=pair_checks,
        max_host_send_interval_ms=max(intervals)*1000,max_raw_interval_ms=raw_intervals)
    out=execution/'matlab_analysis'/'can_diagnostics';out.mkdir(parents=True,exist_ok=True)
    (out/'can_comparison.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False))
    with (out/'decoded_joints.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['time_s_est','joint','q_rad','source']);w.writerows(sorted(decoded))
    print(json.dumps(result,indent=2,ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder',type=Path);p.add_argument('--execution',default='execution_joaafelg')
    a=p.parse_args();analyze(a.folder,a.execution)
