"""仅供集成测试的隔离域假驱动；不连接CAN。"""
import json
import os
from pathlib import Path
import sys
import rclpy
from sensor_msgs.msg import JointState
from agx_arm_msgs.msg import AgxArmStatus
from std_srvs.srv import SetBool, Empty

root=Path(sys.argv[1])
Q0=[.0999724595542352,.09972811345895599,-.1015432558810301,
    .08391543043588737,.01153662635568252,-.08513716091228339]
if os.environ.get('FAKE_ZERO')=='1':Q0=[0.]*6
rclpy.init()
node=rclpy.create_node("agx_arm_ctrl_single_node")
for key,value in {"arm_type":"piper","can_port":"can0","fast_mode":False,
                  "speed_percent":5,"auto_enable":False}.items():node.declare_parameter(key,value)
publisher=node.create_publisher(JointState,"/feedback/joint_states",10)
status_pub=node.create_publisher(AgxArmStatus,"/feedback/arm_status",10)
state={"q":list(Q0),"gate":False,"sent":0,"stops":0,"fault_after":None,"fault":False,"stale_after":None}

def save():
    temp=root/"fake_state.tmp"
    temp.write_text(json.dumps(state))
    temp.replace(root/"fake_state.json")

def gate(req,res):
    state["gate"]=req.data
    control=root/"fake_control.json"
    if not req.data and control.exists():
        settings=json.loads(control.read_text());control.unlink()
        state.update(q=list(Q0),sent=0,stops=0,fault=False,fault_after=None,stale_after=None)
        state.update(settings)
    save();res.success=True;res.message="fake gate updated";return res

def stop(req,res):state["stops"]+=1;save();return res

def command(msg):
    if state["gate"]:
        state["q"]=list(msg.position);state["sent"]+=1
        if state["fault_after"] and state["sent"]>=state["fault_after"]:state["fault"]=True

def tick():
    if state["stale_after"] and state["sent"]>=state["stale_after"]:return
    msg=JointState();msg.header.stamp=node.get_clock().now().to_msg()
    msg.name=["gripper",*[f"joint{i}" for i in range(1,7)]]
    msg.position=[0.,*state["q"]];msg.velocity=[0.]*7;msg.effort=[0.]*7
    publisher.publish(msg)
    s=AgxArmStatus();s.ctrl_mode=1;s.mode_feedback=1;s.motion_status=0
    s.arm_status=7 if state["fault"] else 0
    s.joint_angle_limit=[False]*6;s.communication_status_joint=[False]*6
    status_pub.publish(s)

services=[node.create_service(SetBool,"/control_enable",gate),
          node.create_service(Empty,"/emergency_stop",stop)]
sub=node.create_subscription(JointState,"/control/move_j",command,10)
timer=node.create_timer(1./float(os.environ.get("FAKE_FEEDBACK_HZ","200")),tick)
save()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
    save();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
