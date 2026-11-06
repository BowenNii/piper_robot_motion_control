"""SE(3)变换；长度m、角度rad，旋量[omega;v]。"""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation
from ..common.validation import finite

def skew(v):
    """将三维向量转换为反对称矩阵，使 skew(v) @ u 等于 v × u。

    Args:
        v: 形状 (3,) 的有限数值向量，可传列表或 NumPy 数组。
    Returns:
        形状 (3, 3) 的反对称矩阵，不进行归一化或单位转换。
    Raises:
        ValueError: 输入不能转换为数值、形状不符或包含非有限值。
    """
    x, y, z = finite(v, (3,))
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

def transform(R=None, p=None):
    """将旋转矩阵和位置向量组装成 4×4 齐次变换矩阵。

    Args:
        R: 形状 (3, 3) 的旋转矩阵；None 表示单位旋转。
        p: 形状 (3,) 的位置向量，单位 m；None 表示零平移。
    Returns:
        形状 (4, 4) 的矩阵 [[R, p], [0, 0, 0, 1]]。
        两个参数都省略时返回单位矩阵。
    Raises:
        ValueError: 输入不能转换为数值、形状不符或包含非有限值。

    只检查输入形状与有限性，不检查 R 的正交性或行列式，
    不会把任意矩阵自动修正为合法旋转矩阵。
    """
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = finite(R, (3, 3))
    if p is not None:
        T[:3, 3] = finite(p, (3,))
    return T

def inverse(T):
    """利用刚体变换结构求逆，不调用通用矩阵求逆。

    Args:
        T: 合法的 (4, 4) 齐次刚体变换，平移单位 m。
    Returns:
        逆变换 [[R.T, -R.T @ p], [0, 0, 0, 1]]。
        若 T 将 B 系坐标转换到 A 系，返回值将 A 系转换到 B 系。
    Raises:
        ValueError: 输入不能转换为数值、形状不符或包含非有限值。

    本函数不完整校验 SE(3) 合法性；调用者必须保证 R 是旋转矩阵，
    否则 R.T 不一定等于 R 的逆矩阵。
    """
    T = finite(T, (4, 4))
    R = T[:3, :3].T
    return transform(R, -R @ T[:3, 3])

def adjoint(T):
    """构造刚体变换的 6×6 伴随矩阵，用于转换空间运动向量。

    Args:
        T: 合法的 (4, 4) NumPy 齐次变换矩阵，平移单位 m。
    Returns:
        Ad(T) = [[R, 0], [skew(p) @ R, R]]，形状 (6, 6)。
        对按 [角速度; 线速度] 排列的空间运动向量，若 T 为 T_AB，
        则 V_A = Ad(T_AB) @ V_B。不能直接用同一公式转换力旋量。
    Raises:
        ValueError: 平移分量未通过 skew() 的有限性或形状检查等。

    本函数没有完整的输入形状、有限性和 SE(3) 校验；调用者应先
    保证输入合法，错误输入也可能触发索引或矩阵运算异常。
    """
    R, p = T[:3, :3], T[:3, 3]
    return np.block([[R, np.zeros((3, 3))], [skew(p) @ R, R]])

def exp_twist(x):
    """将六维指数坐标映射为 SE(3) 齐次变换，支持纯平移和小转角。

    Args:
        x: 形状 (6,) 的有限向量，顺序为 [omega; v]，例如 S * q。
            前三项是旋转向量（rad），后三项是指数坐标的平移部分
            （m），不是一般情况下可直接填入矩阵的最终位置。
            输入已包含运动增量，本函数不会再乘关节角或时间。
    Returns:
        形状 (4, 4) 的变换矩阵；旋转为 Exp(skew(omega))，
        平移为 SO(3) 左雅可比乘 v。omega 为零时，平移直接等于 v。
    Raises:
        ValueError: 输入不能转换为数值、形状不符或包含非有限值。

    小转角使用级数避免除以接近零的数；本函数只做数值计算。
    """
    x = finite(x, (6,))
    w, v = x[:3], x[3:]
    theta = np.linalg.norm(w)
    W = skew(w)
    if theta < 1e-6:
        a = .5 - theta**2 / 24 + theta**4 / 720
        b = 1/6 - theta**2 / 120 + theta**4 / 5040
    else:
        a = (1 - np.cos(theta)) / theta**2
        b = (theta - np.sin(theta)) / theta**3
    return transform(Rotation.from_rotvec(w).as_matrix(),
                     (np.eye(3) + a*W + b*W@W) @ v)

def log_transform(T):
    """计算 SE(3) 变换的主值对数，返回六维指数坐标。

    Args:
        T: 合法的 (4, 4) 齐次刚体变换矩阵，平移单位 m。
    Returns:
        形状 (6,) 的 [omega; v]：前三项为主值旋转向量（rad），
        后三项为解线性方程得到的平移指数坐标（m），不一定等于
        T 的位置列。对合法输入，exp_twist(返回值) 应重建原变换。
    Raises:
        ValueError: 输入形状、有限性或旋转转换不符合要求。
        np.linalg.LinAlgError: 求解平移指数坐标的线性方程失败。

    只显式检查 T 的形状和有限性，不完整校验 SE(3)；调用者应
    保证输入合法。主值分支不保留多圈旋转，接近 180° 时旋转轴
    表示可能发生分支变化，不能直接当作连续的多圈关节角使用。
    """
    T = finite(T, (4, 4))
    w = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    W = skew(w)
    theta = np.linalg.norm(w)
    if theta < 1e-6:
        a, b = .5-theta**2/24+theta**4/720, 1/6-theta**2/120+theta**4/5040
    else:
        a, b = (1-np.cos(theta))/theta**2, (theta-np.sin(theta))/theta**3
    v = np.linalg.solve(np.eye(3)+a*W+b*W@W, T[:3, 3])
    return np.r_[w, v]
