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

def load_piper(urdf: str | Path | None = None) -> RobotModel:
    """与MATLAB相同：只取base_link到link6，不自动合并夹爪/负载。"""
    urdf = Path(urdf) if urdf else REPO / "models/urdf/piper_description.urdf"
    doc = ET.parse(urdf).getroot()
    joints = {e.attrib['name']: e for e in doc.findall('joint')}
    links = {e.attrib['name']: e for e in doc.findall('link')}

    def triplet(e, key, default):
        return finite(np.fromstring(e.get(key, default), sep=' '), (3,))

    def origin(e):
        if e is None:
            return np.eye(4)
        return transform(Rotation.from_euler('xyz', triplet(e, 'rpy', '0 0 0')).as_matrix(),
                         triplet(e, 'xyz', '0 0 0'))

    base = joints['world_to_base_link']
    if base.get('type') != 'fixed' or not np.allclose(origin(base.find('origin')), np.eye(4), atol=1e-12):
        raise ValueError("Nonidentity world/base extrinsics not supported")
    screws, homes, inertias, limits = [], [], [], []
    T = np.eye(4)
    for i in range(1, 7):
        j = joints[f'joint{i}']
        expected = 'base_link' if i == 1 else f'link{i-1}'
        if (j.get('type') != 'revolute' or j.find('parent').get('link') != expected
                or j.find('child').get('link') != f'link{i}'):
            raise ValueError("Unsupported serial chain")
        M = origin(j.find('origin'))
        homes.append(M)
        T = T @ M
        axis = triplet(j.find('axis'), 'xyz', '1 0 0')
        if np.linalg.norm(axis) == 0:
            raise ValueError("Zero joint axis")
        w = T[:3, :3] @ (axis / np.linalg.norm(axis))
        screws.append(np.r_[w, -np.cross(w, T[:3, 3])])
        limits.append([float(j.find('limit').get(key)) for key in ('lower', 'upper', 'velocity', 'effort')])
        e = links[f'link{i}'].find('inertial')
        o = origin(e.find('origin'))
        a = e.find('inertia').attrib
        I = np.array([[float(a['ixx']), float(a['ixy']), float(a['ixz'])],
                      [float(a['ixy']), float(a['iyy']), float(a['iyz'])],
                      [float(a['ixz']), float(a['iyz']), float(a['izz'])]])
        inertias.append(spatial_inertia(float(e.find('mass').get('value')),
                                       o[:3, :3] @ I @ o[:3, :3].T, o[:3, 3]))
    limits = finite(limits, (6, 4))
    if np.any(limits[:, 0] >= limits[:, 1]) or np.any(limits[:, 2:] <= 0):
        raise ValueError("Invalid joint limits")
    return RobotModel(np.array(screws).T, np.array(homes+[np.eye(4)]),
                      np.array(inertias), np.array([0., 0., -9.81]),
                      *[limits[:, k] for k in range(4)], source=str(urdf.resolve()))
