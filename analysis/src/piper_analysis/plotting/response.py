"""收尾图形构建；不计算实验误差、不读取CSV。"""
from __future__ import annotations
import numpy as np
from .style import pyplot, MODES, COLORS, LABELS

def draw_response(context) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    response = context.response
    selected = context.selected
    modes = MODES
    colors = COLORS
    labels = LABELS
    finish = context.finish
    fig, axs = plt.subplots(3, 1, figsize=(12, 9))
    fields = ['onset_1pct_s', 'arrival_99pct_s', 'settled_1pct_band_s']
    for ax, field in zip(axs, fields):
        for m, mode in enumerate(modes):
            rows = [r for r in response if r['record'] in selected and r['group'] == 'multi_abc' and r['mode'] == mode]
            ax.scatter([r['joint']+.1*(m-1) for r in rows], [r[field] for r in rows], color=colors[m], label=labels[m])
        ax.set(xlabel='Joint (outbound and return)', ylabel=field, xticks=np.arange(1, 7))
        ax.legend()
    finish(fig, 7, 'response_times')

def draw_delay_validation(context) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    cache = context.cache
    selected = context.selected
    finish = context.finish
    fig, axs = plt.subplots(2, 1, figsize=(13, 8))
    streams = [z for z in cache if z['record']['record'] in selected and 'tr' in z]
    x = np.array([z['record']['record'] for z in streams])
    axs[0].bar(x, [z['row']['lag_from_outbound_ms'] for z in streams])
    axs[0].set_ylabel('Outbound-fitted lag / ms')
    for offset, field, label in [(-.18, 'raw_holdout_joint_rms_deg', 'original'), (.18, 'aligned_holdout_joint_rms_deg', 'aligned')]:
        axs[1].bar(x+offset, [z['row'][field] for z in streams], .36, label=label)
    axs[1].set(ylabel='Holdout joint RMS / deg', xlabel='C record number')
    axs[1].legend()
    finish(fig, 10, 'delay_validation')
