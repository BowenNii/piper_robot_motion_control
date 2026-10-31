"""动力学公共入口；旧版混入的功能通过惰性兼容入口转发。"""
from __future__ import annotations
from .spatial import motion_cross, force_cross, chain_transforms
from .spatial_inertia import spatial_inertia
from .rnea import rnea
from .crba import crba
from .terms import gravity, velocity_bias, forward_dynamics
from .friction import friction

def __getattr__(name):
    """兼容首版导入，不让新算法依赖这些旧入口。"""
    if name == 'with_payload':
        from ..robot_model.payload import with_payload
        return with_payload
    if name == 'computed_torque':
        from ..control.computed_torque import computed_torque
        return computed_torque
    if name in ('regressor', 'inertial_parameters', 'fit_parameters'):
        from importlib import import_module
        modules = {'regressor': 'regressor', 'inertial_parameters': 'parameters', 'fit_parameters': 'fitting'}
        return getattr(import_module('..identification.'+modules[name], __package__), name)
    raise AttributeError(name)

