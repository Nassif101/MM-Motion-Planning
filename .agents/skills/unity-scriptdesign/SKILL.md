---
name: unity-scriptdesign
description: Review or refactor project-owned C# for this Unity–ROS robotics simulation, focusing on responsibility, coupling, lifecycle, ROS boundaries, timing, performance, maintainability, and editor usability. Use when reviewing scripts, diagnosing tangled components, or planning a maintainable implementation.
---

# Unity Script Design Review

Review project-owned code in its actual scene, prefab, package, and ROS context.

## Checklist

- Responsibility: does each type have one coherent job?
- Ownership: is behavior correctly owned by Unity, ROS 2, or external experiment tooling?
- Role: should the type be a `MonoBehaviour`, `ScriptableObject`, editor tool, or plain C# class?
- Dependencies: are required references explicit and validated?
- ROS boundary: are transport, message conversion, and domain behavior separated?
- Frames and units: are coordinate transforms, timestamps, and units named and tested?
- Timing: are physics, rendering, sensor, and publication rates intentionally related?
- Lifecycle: are subscriptions, callbacks, coroutines, and resources cleaned up?
- Threading: are Unity APIs confined to the main thread?
- Performance: are hot sensor/publishing paths free of repeated lookup and avoidable allocation?
- Inspector UX: are serialized fields private, grouped, constrained, and explained?
- Testability: can conversions, filters, state transitions, or experiment rules be isolated?

## State Classification

Classify important fields before refactoring:

- Authored configuration: sensor rates, noise parameters, topic/frame names, robot or payload setup
- Composition references: transforms, rigid bodies, sensors, publishers, connection objects
- Runtime state: latest commands, connection state, measurements, experiment phase, faults

Give each value one clear owner. Avoid fields that are simultaneously designer-authored and freely mutated by unrelated runtime components.

## Repository Boundaries

- Prefer project-owned adapters over edits to UnitySensors, ROS-TCP Connector, package cache, Quixel importer code, or copied samples.
- Do not treat generated ROS message classes as the location for domain logic.
- Preserve externally defined ROS message and frame contracts unless the corresponding ROS side is updated deliberately.

## Output

- Keep
- Simplify
- Refactor
- Unity–ROS contract issues
- Lifecycle/threading issues
- Performance notes
- Test seams and validation steps

## Guardrails

- Prioritize demonstrated risks over theoretical patterns.
- Do not introduce abstraction that exceeds the experiment's likely evolution.
- Use [`../unity-testability/SKILL.md`](../unity-testability/SKILL.md) when a testing design is needed.
