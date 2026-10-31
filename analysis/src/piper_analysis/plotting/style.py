"""公共样式与保存/交互；不在导入时设置全局Matplotlib后端。"""
from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from pathlib import Path


MODES = ['once', 'repeat', 'stream']
COLORS = ['tab:blue', 'tab:orange', 'tab:green']
LABELS = ['A: one endpoint', 'B: repeated endpoint', 'C: streamed targets']


def pyplot(interactive: bool = False):
    if not interactive:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def save_history_figure(fig, folder, name) -> None:
    plt = pyplot()
    fig.tight_layout()
    fig.savefig(folder / f'{name}.png', dpi=140)
    fig.savefig(folder / f'{name}.svg')
    plt.close(fig)


@dataclass
class PlotContext:
    cache: list[dict]
    response: list[dict]
    selected: list[int]
    quality: dict
    output: Path
    show_number: int | None = None

    def pick(self, mode, direction, group='multi_abc'):
        cache, selected = self.cache, self.selected
        matches = [z for z in cache if z['record']['record'] in selected and z['record']['group'] == group
                   and z['record']['manifest']['mode'] == mode and z['record']['manifest']['direction'] == direction]
        if len(matches) != 1:
            raise ValueError(f'Expected unique selection: {group}/{mode}/{direction}')
        return matches[0]

    def finish(self, fig, number, name):
        out, show_number = self.output, self.show_number
        plt = pyplot(interactive=show_number is not None)
        fig.suptitle(name.replace('_', ' '), fontsize=15)
        fig.tight_layout(rect=(0, .08, 1, .96))
        fig.text(0.02, 0.008, 'link6 from feedback angles; not external metrology. A/B internal reference unknown.', fontsize=9)
        if show_number is None:
            fig.savefig(out/'figures'/f'{number:02d}_{name}.png', dpi=140)
            fig.savefig(out/'figures'/f'{number:02d}_{name}.svg')
            plt.close(fig)
        else:
            # 原生工具栏支持平移/放大；悬停坐标显示9位有效数字。
            for ax in fig.axes:
                if getattr(ax, 'name', '') != '3d':
                    ax.format_coord = lambda x, y: f'x={x:.9g}, y={y:.9g}'
                for line in ax.lines:
                    line.set_picker(5)
            status = fig.text(.02, .025, 'Click a curve to inspect a sample.', fontsize=10)
            def inspect(event):
                if not len(event.ind):
                    return
                line = event.artist
                if hasattr(line, 'get_data_3d'):
                    x, y, z = line.get_data_3d()
                    i = event.ind[0]
                    text = f'index={i}, x={x[i]:.12g}, y={y[i]:.12g}, z={z[i]:.12g}'
                else:
                    x, y = line.get_data()
                    indices = event.ind
                    if event.mouseevent.xdata is not None:
                        i = indices[np.argmin(abs(np.asarray(x)[indices]-event.mouseevent.xdata))]
                    else:
                        i = indices[0]
                    text = f'index={i}, x={x[i]:.12g}, y={y[i]:.12g}'
                status.set_text(text)
                fig.canvas.draw_idle()
            fig.canvas.mpl_connect('pick_event', inspect)
            plt.show()
