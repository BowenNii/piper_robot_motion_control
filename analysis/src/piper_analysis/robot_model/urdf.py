"""URDF适配：只读取base_link到link6，默认不含夹爪。"""
from __future__ import annotations
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ..common.paths import REPO
from ..common.validation import finite
from ..lie_group.transforms import transform
from ..dynamics.spatial_inertia import spatial_inertia
from .types import RobotModel

def _tag_name(tag: str) -> str:
    return tag.rsplit('}', 1)[-1] if '}' in tag else tag


def load_piper(urdf: str | Path | None = None) -> RobotModel:
    """读取 PiPER URDF，构建用于运动学和动力学计算的六轴模型。
    Args:
        urdf: URDF 文件路径（字符串或 Path）；省略或传入 None 时，
            使用仓库中的 models/urdf/piper_description.urdf。
    Returns:
        RobotModel 对象，通过 model.字段名读取参数：
        - S：形状 (6, 6)，每列为基座系零位空间螺旋轴，顺序为 [角; 线]。
        - Mlist：形状 (7, 4, 4)，六段零位相对变换及末段单位变换。
        - Glist：形状 (6, 6, 6)，按 [连杆, 行, 列] 保存的空间惯量。
        - gravity：重力向量 [0, 0, -9.81]，单位 m/s²。
        - q_min、q_max：六轴角度下限、上限，单位 rad。
        - velocity_max：六轴标称速度上限，单位 rad/s。
        - effort_max：六轴标称力矩上限，单位 N·m。
        - source：URDF 文件的绝对路径；home 属性为零位末端变换矩阵。
    Raises:
        ValueError: 关节链或基座变换不受支持，必要模型元素缺失，或读取的数值、关节轴、惯量及限位不符合相应检查要求。
        ET.ParseError: URDF 的 XML 格式不合法。
        OSError: 文件不存在、不可读或发生其他文件读取错误。
        TypeError: 必需的数值属性缺失等情况导致数值转换失败。
    """
    urdf = Path(urdf) if urdf else REPO / "models/urdf/piper_description.urdf"
    doc = ET.parse(urdf).getroot()
    joints = {e.attrib['name']: e for e in doc.iter() if _tag_name(e.tag) == 'joint' and 'name' in e.attrib}
    links = {e.attrib['name']: e for e in doc.iter() if _tag_name(e.tag) == 'link' and 'name' in e.attrib}

    def triplet(e, key, default):#从e（XML 元素对象）中读取key属性，再把它转换成三个数。
        if e is None:
            return finite(np.fromstring(default, sep=' '), (3,))
        return finite(np.fromstring(e.get(key, default), sep=' '), (3,))

    def origin(e):#将 URDF 中的 <origin> 元素转换成一个 4×4 位姿变换矩阵，包含旋转和平移。
        if e is None:
            return np.eye(4)
        return transform(Rotation.from_euler('xyz', triplet(e, 'rpy', '0 0 0')).as_matrix(),
                         triplet(e, 'xyz', '0 0 0'))

    base = joints.get('world_to_base_link')
    if base is None:
        fixed = [j for j in joints.values() if j.get('type') == 'fixed']
        for candidate in fixed:
            parent = candidate.find('parent')
            child = candidate.find('child')
            if parent is not None and child is not None and child.get('link') == 'base_link':
                base = candidate
                break
    #基座关节必须存在、必须是固定关节，而且它的变换必须是单位矩阵。
    if base is None or base.get('type') != 'fixed' or not np.allclose(origin(base.find('origin')), np.eye(4), atol=1e-12):
        raise ValueError("Nonidentity world/base extrinsics not supported")

    screws, homes, inertias, limits = [], [], [], []
    T = np.eye(4)
    for i in range(1, 7):
        j = joints.get(f'joint{i}')
        if j is None:
            raise ValueError(f"Joint joint{i} not found")
        expected = 'base_link' if i == 1 else f'link{i-1}'
        parent = j.find('parent')
        child = j.find('child')
        if (j.get('type') != 'revolute' or (parent is not None and parent.get('link') != expected)
                or (child is not None and child.get('link') != f'link{i}')):
            raise ValueError("Unsupported serial chain")
        M = origin(j.find('origin'))
        homes.append(M)
        T = T @ M
        axis = triplet(j.find('axis'), 'xyz', '1 0 0')
        axis_norm = np.linalg.norm(axis)
        if axis_norm == 0:
            raise ValueError("Zero joint axis")
        w = T[:3, :3] @ (axis / axis_norm)
        screws.append(np.r_[w, -np.cross(w, T[:3, 3])])
        limit = j.find('limit')
        if limit is None:
            raise ValueError(f"Joint limit for joint{i} not found")
        limits.append([float(limit.get(key)) for key in ('lower', 'upper', 'velocity', 'effort')])

        link = links.get(f'link{i}')
        if link is None:
            raise ValueError(f"Link link{i} not found")
        e = link.find('inertial')
        if e is None:
            raise ValueError(f"Inertial for link{i} not found")
        o = origin(e.find('origin'))
        inertia = e.find('inertia')
        if inertia is None:
            raise ValueError(f"Inertia for link{i} not found")
        mass = e.find('mass')
        if mass is None:
            raise ValueError(f"Mass for link{i} not found")
        a = inertia.attrib
        I = np.array([[float(a.get('ixx', 0.0)), float(a.get('ixy', 0.0)), float(a.get('ixz', 0.0))],
                      [float(a.get('ixy', 0.0)), float(a.get('iyy', 0.0)), float(a.get('iyz', 0.0))],
                      [float(a.get('ixz', 0.0)), float(a.get('iyz', 0.0)), float(a.get('izz', 0.0))]])
        #新的惯性矩阵就是 \(RIR^T\)，惯量矩阵
        ## G = np.block([
    #                   [I_com + m * C @ C.T,m * C],
    #                   [m * C.T,            m * np.eye(3)]])
        inertias.append(spatial_inertia(float(mass.get('value', 0.0)),
                                       o[:3, :3] @ I @ o[:3, :3].T, o[:3, 3]))
    limits = finite(limits, (6, 4))
    if np.any(limits[:, 0] >= limits[:, 1]) or np.any(limits[:, 2:] <= 0):
        raise ValueError("Invalid joint limits")
    return RobotModel(np.array(screws).T, np.array(homes+[np.eye(4)]),
                      np.array(inertias), np.array([0., 0., -9.81]),
                      *[limits[:, k] for k in range(4)], source=str(urdf.resolve()))
