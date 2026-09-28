"""Pure point selection for the Livox robot filter (no ROS dependencies)."""
import numpy as np


def keep_mask(points_sensor, rotation, translation, footprint_bounds, band=None):
    """Boolean mask of points to keep.

    points_sensor: (N, 3) ROS xyz in the sensor frame.
    rotation, translation: sensor -> base_footprint (3x3, 3).
    footprint_bounds: (x_min, x_max, y_min, y_max) of the active footprint polygon.
    band: optional (min_height, max_height) in base_footprint.

    Drops UnitySensors misses (exact zero vectors) and robot self-returns: every point
    whose base_footprint projection lies inside the footprint polygon (the robot and its
    panel occupy that whole column). Navigation consumers get no band: ground returns
    below their marking height are the rays that clear low voxels near the robot.
    """
    points = np.asarray(points_sensor, dtype=np.float64)
    miss = ~np.any(points != 0.0, axis=1)
    base = points @ np.asarray(rotation).T + np.asarray(translation)
    x_min, x_max, y_min, y_max = footprint_bounds
    inside = ((base[:, 0] >= x_min) & (base[:, 0] <= x_max) &
              (base[:, 1] >= y_min) & (base[:, 1] <= y_max))
    if band is None:
        return ~miss & ~inside
    in_band = (base[:, 2] >= band[0]) & (base[:, 2] <= band[1])
    return ~miss & ~inside & in_band


def footprint_bounds(polygon):
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return min(xs), max(xs), min(ys), max(ys)
