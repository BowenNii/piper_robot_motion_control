"""力矩预测图，只接收已经计算好的结果。"""
from __future__ import annotations
from .style import pyplot


def plot_decomposition(t, prediction, output) -> None:
    """合成演示的四项分解；只画传入结果，不调用动力学算法。"""
    plt = pyplot()
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    for joint, axis in enumerate(axes.flat):
        for key, label in [('total', 'total'), ('gravity', 'gravity'),
                           ('velocity_bias', 'velocity bias'), ('inertial', 'inertia')]:
            axis.plot(t, prediction[key][:, joint], label=label)
        axis.set(title=f'J{joint + 1}', xlabel='Time / s', ylabel='Nominal torque / Nm')
        axis.grid(True)
    axes[0, 0].legend()
    fig.suptitle('Synthetic offline demo: nominal URDF, no payload or friction, not measured data')
    fig.tight_layout()
    fig.savefig(output / 'torque_decomposition.png', dpi=140)
    fig.savefig(output / 'torque_decomposition.svg')
    plt.close(fig)

def plot_prediction(t, prediction, measured, output) -> None:
    plt = pyplot()
    fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    for j, ax in enumerate(axs.flat):
        ax.plot(t, prediction['total'][:, j], label='nominal model')
        ax.plot(t, prediction['gravity'][:, j], label='gravity')
        if measured is not None:
            ax.plot(t, measured[:, j], label='input measured torque')
        ax.set(title=f'J{j + 1}', xlabel='Time / s', ylabel='Torque / Nm')
        ax.grid(True)
    axs[0, 0].legend()
    fig.suptitle('Nominal model, no payload/friction; residual is not measured external force')
    fig.tight_layout()
    fig.savefig(output / 'torque_prediction.png', dpi=140)
    fig.savefig(output / 'torque_prediction.svg')
    plt.close(fig)
