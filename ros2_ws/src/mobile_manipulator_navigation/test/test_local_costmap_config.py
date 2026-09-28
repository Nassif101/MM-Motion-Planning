from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
LOCAL = yaml.safe_load((ROOT / "config" / "nav2_local_costmap.yaml").read_text())[
    "local_costmap"]["local_costmap"]["ros__parameters"]
GLOBAL = yaml.safe_load((ROOT / "config" / "nav2_global_planning.yaml").read_text())[
    "global_costmap"]["global_costmap"]["ros__parameters"]


def test_local_costmap_follows_the_perception_contract():
    assert LOCAL["plugins"] == ["voxel_layer", "inflation_layer"]
    voxel = LOCAL["voxel_layer"]
    assert voxel["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert voxel["footprint_clearing_enabled"] is True
    assert voxel["z_voxels"] <= 16
    assert voxel["origin_z"] + voxel["z_resolution"] * voxel["z_voxels"] >= voxel["max_obstacle_height"]
    source = voxel[voxel["observation_sources"]]
    assert source["topic"] == "/livox/points_filtered"
    assert source["obstacle_min_range"] >= 0.15 and source["raytrace_min_range"] >= 0.15
    assert (source["min_obstacle_height"], source["max_obstacle_height"]) == (0.05, 2.0)


def test_local_and_global_costmaps_share_footprint_and_inflation():
    assert LOCAL["footprint"] == GLOBAL["footprint"]
    assert LOCAL["footprint_padding"] == GLOBAL["footprint_padding"]
    assert LOCAL["inflation_layer"]["inflation_radius"] == GLOBAL["inflation_layer"]["inflation_radius"]
    assert LOCAL["resolution"] == GLOBAL["resolution"]
