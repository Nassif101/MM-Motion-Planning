from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
LOCAL = yaml.safe_load((ROOT / "config" / "nav2_local_costmap.yaml").read_text())[
    "local_costmap"]["local_costmap"]["ros__parameters"]
GLOBAL = yaml.safe_load((ROOT / "config" / "nav2_global_planning.yaml").read_text())[
    "global_costmap"]["global_costmap"]["ros__parameters"]


def test_local_costmap_follows_the_perception_contract():
    assert LOCAL["plugins"] == ["stvl_layer", "inflation_layer"]
    layer = LOCAL["stvl_layer"]
    assert layer["plugin"] == "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    assert layer["update_footprint_enabled"] is True
    mark, clear = (layer[name] for name in layer["observation_sources"].split())
    for source in (mark, clear):
        assert source["topic"] == "/livox/points_filtered"
    assert mark["marking"] and not mark["clearing"]
    assert (mark["min_obstacle_height"], mark["max_obstacle_height"]) == (0.05, 2.0)
    assert clear["clearing"] and not clear["marking"] and clear["model_type"] == 1


def test_decay_outlasts_approach_through_the_blind_zone():
    # A 0.2 m obstacle leaves view about 1.5 m ahead; at the qualified 0.3 m/s the
    # front edge reaches it after ~4 s. Voxels nearer than the frustum only decay by time.
    layer = LOCAL["stvl_layer"]
    clear = layer["livox_clear"]
    assert clear["min_z"] >= 1.5
    assert layer["decay_model"] == 0 and layer["voxel_decay"] >= (1.5 - 0.3) / 0.3 * 2


def test_local_and_global_costmaps_share_footprint_and_inflation():
    assert LOCAL["footprint"] == GLOBAL["footprint"]
    assert LOCAL["footprint_padding"] == GLOBAL["footprint_padding"]
    assert LOCAL["inflation_layer"]["inflation_radius"] == GLOBAL["inflation_layer"]["inflation_radius"]
    assert LOCAL["resolution"] == GLOBAL["resolution"]


def test_global_obstacle_modes():
    # The planner-only launch holds the static map only (it has no lidar filter); the full
    # navigation launch adds persistent obstacles by default (2026-10-01), live on request.
    assert GLOBAL["plugins"] == ["static_layer", "inflation_layer"]
    launch = (ROOT / "launch" / "global_planning.launch.py").read_text()
    assert '"persistent": "persistent_obstacle_layer"' in launch
    assert '"live": "obstacle_layer", "true": "obstacle_layer"' in launch
    assert '"global_obstacles",\n                default_value="static"' in launch
    navigation = (ROOT / "launch" / "navigation.launch.py").read_text()
    assert 'DeclareLaunchArgument("global_obstacles", default_value="persistent"' in navigation


def test_persistent_obstacle_layer_follows_the_sensor_contract():
    layer, live = GLOBAL["persistent_obstacle_layer"], GLOBAL["obstacle_layer"]
    mark = live[live["observation_sources"].split()[0]]
    assert layer["plugin"] == "mobile_manipulator_navigation::PersistentObstacleLayer"
    assert layer["topic"] == mark["topic"] == "/livox/points_filtered"
    for key in ("min_obstacle_height", "max_obstacle_height", "obstacle_range"):
        assert layer[key] == mark[key], key
    assert layer["decay"] == live["voxel_decay"]
    # A 0.5 m worker at 0.8 m/s covers a cell for about 0.6 s; it must never be confirmed,
    # and the lidar's sparse hits on a stationary obstacle must not break the count.
    assert layer["persistence"] >= 2 * 0.5 / 0.8
    assert 0.1 < layer["max_gap"] < layer["persistence"]


def test_live_global_obstacle_layer_follows_the_local_sensor_contract():
    local, glob = LOCAL["stvl_layer"], GLOBAL["obstacle_layer"]
    for key in ("plugin", "voxel_decay", "decay_model", "voxel_size", "mark_threshold",
                "observation_sources", "combination_method"):
        assert glob[key] == local[key], key
    for source in local["observation_sources"].split():
        assert glob[source] == local[source], source
