---
name: unity-scene-contracts
description: Define and verify required objects, components, ROS connections, sensor wiring, coordinate frames, bootstrap order, runtime-spawned objects, and validation for scenes in this Unity–ROS simulation. Use when creating or changing scenes, prefabs, robot setups, sensors, or experiment initialization.
---

# Unity Scene Contracts

Make scene requirements explicit enough that a fresh scene or experiment can be validated without relying on hidden lookups.

## Define

- Required root objects and prefabs
- Required components and serialized references
- ROS connection ownership and configuration
- Robot, payload, sensor, obstacle, and environment object roles
- Coordinate-frame names, axes, units, and parent-child relationships
- Topic names, message types, publication rates, and timestamps
- Objects authored in the scene versus spawned at runtime
- Initialization and shutdown sequence
- Preconditions for starting an experiment

## Project Checks

- Reuse a project-owned ROS connection prefab where appropriate; verify its current configuration rather than assuming defaults.
- Ensure each sensor's Unity pose, ROS frame, topic, rate, and noise configuration agree.
- Keep scene-specific values in the scene or prefab and reusable experiment configuration in an appropriate asset.
- Validate missing references and incompatible configuration before publishing data or applying robot commands.
- Treat imported sample scenes as references, not as the project's scene contract.

## Output

- Scene object/component contract
- Bootstrap and readiness sequence
- Inspector wiring rules
- ROS and coordinate-frame contract
- Runtime-spawn rules
- Validation checklist
- Hidden dependency risks

## Guardrails

- Prefer explicit references over chains of `Find` calls.
- Keep bootstrap objects focused on composition.
- Do not duplicate ROS connection ownership across unrelated objects.
- Do not silently create missing critical dependencies at runtime.
