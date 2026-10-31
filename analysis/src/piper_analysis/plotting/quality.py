"""收尾图形构建；不计算实验误差、不读取CSV。"""
from __future__ import annotations
import numpy as np
from .style import pyplot

def draw_quality(context) -> None:
    plt = pyplot(interactive=context.show_number is not None)
    cache = context.cache
    quality = context.quality
    finish = context.finish
    fig, axs = plt.subplots(4, 1, figsize=(13, 10))
    x = np.arange(1, len(cache)+1)
    axs[0].bar(x-.2, quality['old_max_span_ms'], .4, label='old assembly')
    axs[0].bar(x + 0.2, quality['new_max_span_ms'], 0.4, label='ordered cycles')
    axs[0].legend()
    axs[0].set_ylabel('Max span / ms')
    axs[1].bar(x-.2, quality['old_over_2ms']/quality['old_samples']*100, .4)
    axs[1].bar(x + 0.2, quality['new_over_2ms_fraction'] * 100, 0.4)
    axs[1].set_ylabel('Span >2 ms / %')
    axs[2].bar(x, [z['row']['pc_lateness_max_ms'] for z in cache])
    axs[2].axhline(20, color='r', ls='--')
    axs[2].set_ylabel('Max PC lateness / ms')
    axs[3].bar(x, [z['row']['ros_receive_gap_max_ms'] for z in cache])
    axs[3].set_ylabel('Max ROS gap / ms')
    axs[3].set_xlabel('Record number')
    finish(fig, 1, 'data_quality')
