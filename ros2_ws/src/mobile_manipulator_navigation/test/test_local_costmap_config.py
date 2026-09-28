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
