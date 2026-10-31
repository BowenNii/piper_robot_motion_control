"""前三阶段的绘图，只使用传入结果，不读取实验数据。"""
from __future__ import annotations
import numpy as np
from .style import pyplot, save_history_figure as save

def plot_static_pose(d, q, p, r, folder, file):
    plt = pyplot()
    fig, ax = plt.subplots(3, 1, figsize=(11, 8))
    ax[0].plot(d['t_s'], np.rad2deg(q))
    ax[0].set_ylabel('q / deg')
    ax[0].legend([f'J{i}' for i in range(1, 7)], ncol=6, fontsize=8)
    ax[1].plot(d['t_s'], p)
    ax[1].set_ylabel('Model position difference / mm')
    ax[2].plot(d['t_s'], r)
    ax[2].set(ylabel='Rotation difference / deg', xlabel='Time / s')
    fig.suptitle(f'{file.stem}: static, not external positioning accuracy')
    save(fig, folder, file.stem)

def plot_static_overview(rows, folder):
    plt = pyplot()
    fig, ax = plt.subplots(2, 1, figsize=(12, 7))
    x = np.arange(1, len(rows) + 1)
    ax[0].plot(x, [[r[f'q{j}_mean_deg'] for j in range(1, 7)] for r in rows], 'o')
    ax[0].set_ylabel('Joint mean / deg')
    ax[0].legend([f'J{i}' for i in range(1, 7)], ncol=6, fontsize=8)
    ax[1].bar(x, [r['position_max_mm'] for r in rows])
    ax[1].set(ylabel='Max model difference / mm', xlabel='Independent pose number')
    save(fig, folder, 'overview')

def plot_dynamic_run(t, qdeg, p, r, span, folder, file):
    plt = pyplot()
    fig, ax = plt.subplots(4, 1, figsize=(12, 10))
    ax[0].plot(t, qdeg)
    ax[0].set_ylabel('q / deg')
    ax[0].legend([f'J{i}' for i in range(1, 7)], ncol=6, fontsize=8)
    for a, y, ylabel in zip(ax[1:], [p, r, span], ['Model position difference / mm', 'Rotation difference / deg', 'CAN group span / ms']):
        a.plot(t, y)
        a.set_ylabel(ylabel)
    ax[-1].set_xlabel('Time / s')
    fig.suptitle(f'{file.stem}: model consistency, no time-varying target reference')
    save(fig, folder, file.stem)

def plot_ik(t, q, p, r, cp, cr, sigma, d, dq, ddq, folder):
    plt = pyplot()
    for name, arrays, labels in [('01_joint_angles', [np.rad2deg(q)], ['q / deg']),
            ('03_pose_residuals', [p*1000, np.rad2deg(r)], ['Position residual / mm', 'Rotation residual / deg']),
            ('04_solver', [np.array(sigma), d['iterations']], ['Weighted sigma min', 'Iterations']),
            ('05_crosscheck', [cp, cr], ['Python-C++ position / m', 'Python-C++ rotation / rad']),
            ('06_joint_velocity', [dq], ['dq / rad/s']), ('07_joint_acceleration', [ddq], ['ddq / rad/s²'])]:
        fig, axs = plt.subplots(len(arrays), 1, figsize=(12, 7), squeeze=False)
        for ax, values, label in zip(axs[:, 0], arrays, labels):
            ax.plot(t, values)
            ax.set(xlabel='Time / s', ylabel=label)
            ax.grid(True)
            if values.ndim == 2 and values.shape[1] == 6:
                ax.legend([f'J{i}' for i in range(1, 7)], ncol=6, fontsize=8)
        fig.suptitle('Offline IK verification (not hardware accuracy)')
        fig.tight_layout()
        fig.savefig(folder / f'{name}.png', dpi=140)
        fig.savefig(folder / f'{name}.svg')
        plt.close(fig)
