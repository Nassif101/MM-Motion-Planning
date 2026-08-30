---
name: unity-architecture
description: Plan or review architecture for this Unity–ROS robotics simulation, including module boundaries, ownership between Unity and ROS, scene composition, sensor integration, experiment control, and refactoring. Use before adding structural code, splitting responsibilities, reducing coupling, or changing how simulation systems communicate.
---

# Unity–ROS Architecture

Start with [`../unity-project-scout/SKILL.md`](../unity-project-scout/SKILL.md) when the relevant project area has not been inspected.

## Workflow

1. Define the requested simulation or experiment behavior.
2. Identify whether Unity, ROS 2, or an external experiment runner owns each responsibility.
3. Preserve existing package and scene conventions where they are sound.
4. Propose the smallest set of project-owned modules needed.
5. Define data flow, lifecycle, failure handling, and validation before implementation.

## Project Boundaries

- Treat Unity primarily as the environment, rendering, physics, sensor, and visualization side unless repository evidence assigns it more responsibility.
- Treat ROS 2 as the default owner of navigation and motion-planning behavior.
- Keep ROS transport code at an adapter boundary. Do not spread message types and connection lookups through unrelated components.
- Keep third-party package code, imported assets, and generated ROS messages separate from project-owned code.
- Make coordinate-frame conventions, units, timestamps, topic names, and message ownership explicit.

## Design Guidance

- Prefer thin `MonoBehaviour` components for scene integration and lifecycle hooks.
- Put reusable calculations, state transitions, conversions, and experiment rules in plain C# when practical.
- Use `ScriptableObject` for authored, reusable configuration—not mutable runtime state by default.
- Prefer explicit serialized references or composition over global lookups and hidden singleton dependencies.
- Add assembly definitions only when project-owned code has stable module boundaries or test isolation needs them.
- Avoid per-frame allocations and repeated lookups in sensors, publishers, physics callbacks, and visualization loops.

## Startup and Readiness

- Use one clear scene entry point when ordered initialization is required.
- Do not rely on incidental `Awake` order.
- Gate publishing and processing on explicit readiness: required references, ROS connection, message registration, and sensor initialization.
- Define behavior for disconnects, scene reloads, disabled objects, and editor shutdown.

## Output

- Responsibility map: Unity / ROS 2 / external tooling
- Recommended project-owned modules and one-line responsibilities
- Scene/bootstrap plan
- Data and configuration ownership
- Communication and lifecycle rules
- ROS contract and coordinate-frame constraints
- Performance-sensitive paths
- Do now / defer

## Guardrails

- Do not introduce a framework or dependency without evidence that the current project needs it.
- Do not move planning or navigation logic into Unity merely for convenience.
- Do not modify package cache, imported vendor code, or samples when a project-owned adapter is viable.
- Use [`../unity-adr/SKILL.md`](../unity-adr/SKILL.md) for consequential choices.
- Use [`../unity-async/SKILL.md`](../unity-async/SKILL.md), [`../unity-scene-contracts/SKILL.md`](../unity-scene-contracts/SKILL.md), and [`../unity-scriptdesign/SKILL.md`](../unity-scriptdesign/SKILL.md) for focused follow-up.
