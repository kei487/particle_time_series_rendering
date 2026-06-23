import numpy as np

from particle_time_series_rendering.render_utils import (
    build_overlay_colors,
    occupancy_grid_to_bgr,
    world_to_pixel,
)


def test_occupancy_grid_to_bgr_maps_known_values() -> None:
    image = occupancy_grid_to_bgr(
        occupancy_data=[0, 100, -1, 0],
        width=2,
        height=2,
    )

    assert tuple(image[0, 0]) == (127, 127, 127)
    assert tuple(image[0, 1]) == (255, 255, 255)
    assert tuple(image[1, 0]) == (255, 255, 255)
    assert tuple(image[1, 1]) == (0, 0, 0)


def test_world_to_pixel_uses_bottom_left_map_origin() -> None:
    px, py = world_to_pixel(
        x=1.5,
        y=2.5,
        resolution=0.5,
        origin_x=1.0,
        origin_y=2.0,
        image_height=10,
    )

    assert (px, py) == (1, 8)


def test_build_overlay_colors_spans_gradient() -> None:
    colors = build_overlay_colors(3)

    assert len(colors) == 3
    assert colors[0][0] > colors[0][2]
    assert colors[-1][2] > colors[-1][0]
    assert np.array(colors).dtype == np.int64 or np.array(colors).dtype == np.int32
