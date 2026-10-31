"""收尾图形构建；不计算实验误差、不读取CSV。"""
from __future__ import annotations
import numpy as np
from .style import pyplot, MODES, LABELS

def draw_joint_angles(context, d, direction) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    modes = MODES
    labels = LABELS
    pick = context.pick
    finish = context.finish
    z = pick('stream', direction)
    fig, axs = plt.subplots(6, 3, figsize=(15, 13), sharex=True)
    for j in range(6):
        for m, mode in enumerate(modes):
            v = pick(mode, direction)
            ax = axs[j, m]
            ax.plot(v['t'], np.rad2deg(v['q'][:, j]), label='feedback')
            if m == 2:
                ax.step(v['ct'], np.rad2deg(v['qc'][:, j]), where='post', color='k', ls='--')
            else:
                ax.axhline(np.rad2deg(v['qg'][j]), color='k', ls='--')
            ax.set(ylabel=f'J{j+1} / deg', xlim=(0, 13.76))
            if j == 0:
                ax.set_title(labels[m])
            if j == 5:
                ax.set_xlabel('Time / s')
    finish(fig, 4+d, f'joint_angles_{direction}')
