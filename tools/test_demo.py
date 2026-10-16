"""tools/demo.py decisions that need no Docker or Unity.

Run: python3 -m pytest tools/test_demo.py
"""
import sys
import types

import pytest

sys.modules.setdefault("analyze_arm_qualification", types.SimpleNamespace(analyze=None))
import demo  # noqa: E402


class NoContainer:
    """A runner that fails the test when the demo reaches the container."""

    def ros(self, *args, **kwargs):
        raise AssertionError(f"reached the container: {args}")


def test_arm_refuses_carry_before_reaching_moveit():
    # carry is qualified only through home with arm_experiment and as B5's comfort pose, not as a
    # MoveIt (reconfigure_panel) target.
    with pytest.raises(SystemExit) as refused:
        demo.arm(NoContainer(), "carry")
    message = str(refused.value)
    assert "not qualified with MoveIt" in message
    assert "--arm-pose carry" in message and "B5" in message


def test_arm_offers_only_the_moveit_qualified_poses():
    assert demo.MOVEIT_ARM_POSES == ("home", "vertical_carry")
    assert set(demo.MOVEIT_ARM_POSES) < set(demo.ARM_POSES)


def test_demo_site_defaults_to_the_scenario_site():
    table = {"open_space_nav": ("navigate_to_pose", "d", "construction_site"),
             "v2_open_nav": ("navigate_to_pose", "d", "construction_site_v2")}
    assert demo.choose_site(table, "v2_open_nav", None) == "construction_site_v2"
    assert demo.choose_site(table, "open_space_nav", None) == "construction_site"
    assert demo.choose_site(table, None, None) == "construction_site"
    assert demo.choose_site(table, "v2_open_nav", "construction_site") == "construction_site"
