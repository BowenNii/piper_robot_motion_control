"""收尾图形构建；不计算实验误差、不读取CSV。"""
from __future__ import annotations
from .style import pyplot

def draw_joint_errors(context) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    pick = context.pick
    finish = context.finish
    fig, axs = plt.subplots(6, 2, figsize=(13, 13))
    for d, direction in enumerate(['outbound', 'return']):
        z = pick('stream', direction)
        for j in range(6):
            ax = axs[j, d]
            ax.plot(z['tr'], z['jerror'][:, j], label='original')
            ax.plot(z['aligned_t'], z['jaligned'][:, j], label='post-hoc aligned')
            ax.set(ylabel=f'J{j+1} error / deg', title=direction)
            if j == 0:
                ax.legend()
            if j == 5:
                ax.set_xlabel('Time / s')
    finish(fig, 8, 'C_joint_tracking')

def draw_pose_errors(context) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    pick = context.pick
    finish = context.finish
    fig, axs = plt.subplots(3, 2, figsize=(13, 10))
    for d, direction in enumerate(['outbound', 'return']):
        z = pick('stream', direction)
        for j, (field, aligned, label) in enumerate([('pe', 'paligned', 'Position error / mm'),
                ('re', 'raligned', 'Rotation error / deg'), ('path_forward', None, 'Polyline distance / mm')]):
            ax = axs[j, d]
            ax.plot(z['tr'], z[field], label='original')
            if aligned:
                ax.plot(z['aligned_t'], z[aligned], label='post-hoc aligned')
            ax.set(xlabel='Time / s', ylabel=label, title=direction)
            if j == 0:
                ax.legend()
    finish(fig, 9, 'C_pose_and_path_errors')
