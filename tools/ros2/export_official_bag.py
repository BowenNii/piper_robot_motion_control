#!/usr/bin/env python3
"""仅读MCAP并导出官方对照的ROS反馈；不初始化ROS节点，不发布或调用服务。"""
import argparse
import csv
import json
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args()
    folder=args.folder.resolve()
    out=folder/'ros_export'
    out.mkdir(exist_ok=False)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder/'rosbag'/'rosbag_0.mcap'),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    classes={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts={}
    with (out/'feedback.csv').open('x',newline='') as f, (out/'status.csv').open('x',newline='') as s:
        fw=csv.writer(f);sw=csv.writer(s)
        fw.writerow(['receive_epoch_s','header_epoch_s',*[f'q{i}_rad' for i in range(1,7)]])
        sw.writerow(['receive_epoch_s','ctrl_mode','arm_status','mode_feedback','motion_status',
                     'err_status','any_angle_limit','any_communication_error'])
        while reader.has_next():
            topic,data,stamp=reader.read_next()
            counts[topic]=counts.get(topic,0)+1
            if topic not in ('/feedback/joint_states','/feedback/arm_status'):
                continue
            msg=deserialize_message(data,classes[topic])
            if topic.endswith('/joint_states'):
                values=dict(zip(msg.name,msg.position))
                fw.writerow([stamp/1e9,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                             *[values[f'joint{i}'] for i in range(1,7)]])
            else:
                sw.writerow([stamp/1e9,msg.ctrl_mode,msg.arm_status,msg.mode_feedback,msg.motion_status,
                             msg.err_status,int(any(msg.joint_angle_limit)),int(any(msg.communication_status_joint))])
    (out/'summary.json').write_text(json.dumps({'topics_read':counts,
        'note':'Offline MCAP extraction; no command reconstruction; timestamps are bag receive times.'},indent=2))
    print(out)


if __name__=='__main__':
    main()
