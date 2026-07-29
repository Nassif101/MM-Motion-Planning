# Mobile manipulator description

This package is the source of truth for the project's four-wheel, six-axis mobile manipulator.

## Regeneration

Use the repository-local `.venv-cad` Python environment and the installed text-to-CAD launchers.
Generate CAD before regenerating the URDF whenever mesh geometry changes.

The generated URDF must not be edited directly; edit `urdf/mobile_manipulator.py`.

## Unity

Run `tools/sync_mobile_manipulator_to_unity.py` from the repository root after regenerating the model.
In Unity, import:

`Assets/Robots/MobileManipulator/urdf/mobile_manipulator.urdf`

The synchronization script translates standard ROS package URIs only in the Unity copy because the installed Unity URDF Importer resolves mesh paths relative to the URDF directory.

After synchronization, use Unity's `Tools > Motion Planning > Import Mobile Manipulator` command. The project-owned importer validates the hierarchy and repairs the installed URDF Importer's Windows mesh-reference issue before saving the prefab.

The installed URDF Importer may log `meshes cannot be created! It may already exist` warnings while reusing its generated cylinder-collision asset. These warnings originate in the package's existing-folder branch; the project importer validates the resulting colliders and prefab before reporting success.

The initial import intentionally contains no sensor components, ROS publishers, drive controller, or arm command subscriber.
