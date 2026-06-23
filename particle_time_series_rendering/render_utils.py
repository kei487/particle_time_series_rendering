from __future__ import annotations

import colorsys
from typing import Iterable, Sequence

import numpy as np


UNKNOWN_VALUE = -1


def occupancy_grid_to_bgr(
    occupancy_data: Sequence[int], width: int, height: int
) -> np.ndarray:
    """Convert an OccupancyGrid data array into a BGR image."""
    grid = np.asarray(occupancy_data, dtype=np.int16).reshape((height, width))
    image = np.full((height, width, 3), 255, dtype=np.uint8)

    image[grid == 100] = (0, 0, 0)
    image[grid == UNKNOWN_VALUE] = (127, 127, 127)

    return np.flipud(image)


def world_to_pixel(
    x: float,
    y: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    image_height: int,
) -> tuple[int, int]:
    """Convert map coordinates into image pixel coordinates."""
    px = int((x - origin_x) / resolution)
    py = image_height - 1 - int((y - origin_y) / resolution)
    return px, py


def particle_arrays(
    poses: Iterable, resolution: float, origin_x: float, origin_y: float, image_height: int
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorize pose positions into pixel arrays."""
    points = np.array([(pose.position.x, pose.position.y) for pose in poses], dtype=np.float64)
    if points.size == 0:
        return np.array([], dtype=np.int32), np.array([], dtype=np.int32)

    px = ((points[:, 0] - origin_x) / resolution).astype(np.int32)
    py = image_height - 1 - ((points[:, 1] - origin_y) / resolution).astype(np.int32)
    return px, py


def build_overlay_colors(overlay_count: int) -> list[tuple[int, int, int]]:
    """Create a blue-to-red BGR gradient for overlay rendering."""
    if overlay_count <= 0:
        raise ValueError("overlay_count must be greater than 0")

    if overlay_count == 1:
        hue_values = [0.0]
    else:
        hue_values = np.linspace(2.0 / 3.0, 0.0, overlay_count)

    colors: list[tuple[int, int, int]] = []
    for hue in hue_values:
        red, green, blue = colorsys.hsv_to_rgb(float(hue), 1.0, 1.0)
        colors.append((int(blue * 255), int(green * 255), int(red * 255)))

    return colors
