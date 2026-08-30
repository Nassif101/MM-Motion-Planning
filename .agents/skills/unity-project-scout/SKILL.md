---
name: unity-project-scout
description: Inspect this repository's Unity simulation before proposing changes. Use when first approaching a Unity task, auditing the setup, or before structural work that depends on Unity version, packages, scenes, project-owned scripts, ROS integration, sensors, assemblies, tests, or existing conventions.
---

# Unity Project Scout

Use the repository contents and, when available, read-only Unity CLI/Pipeline commands to establish the relevant baseline. Run `unity status --format json` before live Editor inspection. Unity MCP is not part of this project's toolchain.

## Project Anchors

- Unity project: `motion-planning-sim/`
- ROS workspace: `ros2_ws/`
- Primary domain: mobile-robot motion-planning experiments in construction-like environments

These anchors describe the current repository, but verify files rather than assuming the setup is unchanged.

## Inspect

- `ProjectSettings/ProjectVersion.txt`
- `Packages/manifest.json` and relevant locked versions
- Project-owned folders, scripts, scenes, prefabs, settings, and assembly definitions under `Assets/`
- ROS connection prefabs, generated messages, topic/service/action contracts, and coordinate-frame conventions
- Sensor packages and scene configuration
- Existing EditMode, PlayMode, and cross-process tests
- Naming, serialization, lifecycle, and code-organization patterns in project-owned code
- Unity console errors when the Editor is available

Current notable dependencies include HDRP, UnitySensors, UnitySensorsROS, and ROS-TCP Connector. Confirm their presence before relying on them.

## Scope Rules

- Distinguish project-owned assets from `Library/PackageCache`, imported vendor assets, generated code, and package samples.
- Do not infer project conventions from package or sample code.
- Inspect only the ROS packages and interfaces relevant to the Unity task.
- Report missing foundations—such as an empty project script folder or absent project assembly definitions—instead of inventing conventions.

## Output

- Technical baseline
- Project-owned implementation surface
- Unity–ROS integration points
- Existing conventions worth preserving
- Risks, inconsistencies, and generated/vendor boundaries
- Constraints for the proposed work
- Unknowns requiring runtime confirmation

## Guardrails

- Never infer topic names, controller interfaces, frame IDs, rates, QoS, or joint ordering from package samples or common ROS conventions. Verify them on both the ROS and Unity sides and report mismatches before implementation.
- Do not recommend a clean-slate structure before checking what exists.
- Do not add dependencies until the current stack and need are clear.
- Do not edit `Library/`, package cache contents, or imported samples as the default solution.
