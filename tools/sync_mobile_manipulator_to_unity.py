"""Copy generated robot-description assets into the Unity project.

The ROS package remains the source of truth. This script only copies importable
artifacts and never edits the generated URDF.
"""

from __future__ import annotations

import shutil
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_PACKAGE = REPOSITORY_ROOT / "ros2_ws" / "src" / "mobile_manipulator_description"
UNITY_TARGET = (
    REPOSITORY_ROOT
    / "motion-planning-sim"
    / "Assets"
    / "Robots"
    / "MobileManipulator"
)


def main():
    required = [
        SOURCE_PACKAGE / "package.xml",
        SOURCE_PACKAGE / "urdf" / "mobile_manipulator.urdf",
        SOURCE_PACKAGE / "meshes" / "visual",
    ]
    missing = [path for path in required if not path.exists()]
    if missing:
        joined = "\n".join(str(path) for path in missing)
        raise FileNotFoundError(f"Generate the robot assets before syncing:\n{joined}")

    UNITY_TARGET.mkdir(parents=True, exist_ok=True)
    shutil.copy2(SOURCE_PACKAGE / "package.xml", UNITY_TARGET / "package.xml")
    unity_urdf_root = UNITY_TARGET / "urdf"
    unity_urdf_root.mkdir(parents=True, exist_ok=True)

    # Unity URDF Importer resolves package:// paths from the URDF directory and
    # expects the package name to be omitted. Keep the ROS artifact standard and
    # translate only the generated Unity copy.
    source_urdf = (SOURCE_PACKAGE / "urdf" / "mobile_manipulator.urdf").read_text(
        encoding="utf-8"
    )
    unity_urdf = source_urdf.replace(
        "package://mobile_manipulator_description/meshes/",
        "package://meshes/",
    )
    (unity_urdf_root / "mobile_manipulator.urdf").write_text(
        unity_urdf,
        encoding="utf-8",
    )

    shutil.copytree(
        SOURCE_PACKAGE / "meshes",
        unity_urdf_root / "meshes",
        dirs_exist_ok=True,
    )
    shutil.copy2(SOURCE_PACKAGE / "README.md", UNITY_TARGET / "README.md")
    print(f"Synced mobile manipulator assets to {UNITY_TARGET}")


if __name__ == "__main__":
    main()
