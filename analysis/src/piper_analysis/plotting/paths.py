"""收尾图形构建；不计算实验误差、不读取CSV。"""
from __future__ import annotations
import numpy as np
from .style import pyplot, MODES, COLORS, LABELS

def draw_c_path(context, d, direction) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    pick = context.pick
    finish = context.finish
    z = pick('stream', direction)
    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(221, projection='3d')
    for P, style, label in [(z['refP'], 'k--', 'published reference'), (z['ptrace'], 'b-', 'feedback-angle FK')]:
        ax.plot(*P.T, style, label=label)
    ax.set(xlabel='X / mm', ylabel='Y / mm', zlabel='Z / mm')
    ax.legend()
    ax.set_box_aspect(np.maximum(np.ptp(np.vstack([z['refP'], z['ptrace']]), axis=0), .1))
    for pos, (a, b) in enumerate([(0, 1), (0, 2), (1, 2)], 2):
        ax = fig.add_subplot(2, 2, pos)
        ax.plot(z['refP'][:, a], z['refP'][:, b], 'k--')
        ax.plot(z['ptrace'][:, a], z['ptrace'][:, b], 'b-')
        ax.set(xlabel=f'{'XYZ'[a]} / mm', ylabel=f'{'XYZ'[b]} / mm')
        ax.set_aspect('equal', adjustable='datalim')
        ax.margins(.08)
    finish(fig, 2+d, f'C_path_{direction}')

def draw_abc_paths(context) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    modes = MODES
    colors = COLORS
    labels = LABELS
    pick = context.pick
    finish = context.finish
    fig = plt.figure(figsize=(13, 8))
    for d, direction in enumerate(['outbound', 'return']):
        ax = fig.add_subplot(1, 2, d + 1, projection='3d')
        paths = []
        for mode, color, label in zip(modes, colors, labels):
            z = pick(mode, direction)
            mask = (z['t'] >= 0) & (z['t'] <= z['duration'])
            P = z['p'][mask]
            paths.append(P)
            ax.plot(*P.T, color=color, label=label)
        ax.set(xlabel='X / mm', ylabel='Y / mm', zlabel='Z / mm', title=direction)
        ax.set_box_aspect(np.maximum(np.ptp(np.vstack(paths), axis=0), 0.1))
        ax.legend(fontsize=8)
    finish(fig, 6, 'ABC_executed_paths')
