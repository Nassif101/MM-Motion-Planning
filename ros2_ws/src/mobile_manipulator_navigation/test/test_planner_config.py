import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load(
    (PACKAGE_ROOT / "config" / "nav2_global_planning.yaml").read_text(encoding="utf-8")
)
PLANNER = CONFIG["planner_server"]["ros__parameters"]
COSTMAP = CONFIG["global_costmap"]["global_costmap"]["ros__parameters"]
PRIMITIVES = (
    Path(get_package_share_directory("nav2_smac_planner"))
    / "sample_primitives" / "5cm_resolution" / "0.5m_turning_radius" / "diff" / "output.json"
)


def test_navfn_baseline_and_footprint_aware_lattice_are_both_available():
    assert PLANNER["planner_plugins"] == ["GridBased", "Lattice"]
    assert PLANNER["GridBased"]["plugin"] == "nav2_navfn_planner::NavfnPlanner"
    assert PLANNER["Lattice"]["plugin"] == "nav2_smac_planner::SmacPlannerLattice"
    assert PLANNER["Lattice"]["allow_unknown"] is False


def test_lattice_primitives_match_skid_steer_and_map_resolution():
    metadata = json.loads(PRIMITIVES.read_text(encoding="utf-8"))["lattice_metadata"]
    # Differential-drive primitives include in-place rotation, as the skid-steer base allows.
    assert metadata["motion_model"] == "diff"
    assert metadata["grid_resolution"] == COSTMAP["resolution"]
