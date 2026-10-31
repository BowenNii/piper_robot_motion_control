"""十图组织器，具体图形分别位于quality/paths/joints/response/errors。"""
from __future__ import annotations
from pathlib import Path
from ..data_io.plot_cache import save_plot_cache, load_plot_cache
from .style import PlotContext, pyplot
from .quality import draw_quality
from .paths import draw_c_path, draw_abc_paths
from .joints import draw_joint_angles
from .response import draw_response, draw_delay_validation
from .errors import draw_joint_errors, draw_pose_errors


def draw_closeout(cache, response, selected, quality, output, show_number=None) -> None:
    plt = pyplot(interactive=show_number is not None)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.grid': True})
    context = PlotContext(cache, response, selected, quality, Path(output), show_number)
    if show_number is None:
        save_plot_cache(cache, response, selected, quality, context.output)
    builders = {1: draw_quality, 6: draw_abc_paths, 7: draw_response,
                8: draw_joint_errors, 9: draw_pose_errors, 10: draw_delay_validation}
    if show_number is None or show_number == 1:
        draw_quality(context)
    for d, direction in enumerate(['outbound', 'return']):
        if show_number is None or show_number == 2 + d:
            draw_c_path(context, d, direction)
        if show_number is None or show_number == 4 + d:
            draw_joint_angles(context, d, direction)
    for number in (6, 7, 8, 9, 10):
        if show_number is None or show_number == number:
            builders[number](context)


def view(output, number: int) -> None:
    if not 1 <= number <= 10:
        raise ValueError('Figure number must be 1..10')
    cache, response, selected, quality = load_plot_cache(output)
    draw_closeout(cache, response, selected, quality, output, number)
