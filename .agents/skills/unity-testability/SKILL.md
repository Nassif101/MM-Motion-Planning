---
name: unity-testability
description: Design tests and test seams for this Unity–ROS robotics simulation. Use when extracting logic from MonoBehaviours, validating coordinate or message conversions, planning EditMode or PlayMode coverage, testing scenes and sensors, or defining cross-process Unity–ROS integration tests.
---

# Unity–ROS Testability

Choose the lowest-cost test level that can prove the behavior.

## Separate

- Keep calculations, coordinate conversions, filters, state machines, and experiment rules in plain C# when they do not require Unity objects.
- Keep physics, rendering, scene wiring, sensors, and Unity lifecycle behavior Unity-facing.
- Isolate ROS transport behind small adapters so domain behavior can be tested without a live connection.
- Make clocks, randomness, and input data controllable where reproducibility matters.

## Test Levels

- EditMode: pure calculations, transforms and units, message mapping, configuration validation, state transitions
- PlayMode: component lifecycle, prefab and scene wiring, physics behavior, sensor scheduling, cleanup
- Unity–ROS integration: connection, message schemas, topic/frame names, timestamps, rates, reconnect behavior, and end-to-end experiment flow
- Manual/visual validation: rendering and environment appearance only when an automated assertion is impractical

## Robotics-Specific Checks

- Coordinate handedness, axes, units, and frame hierarchy
- Deterministic inputs or bounded tolerances for physics and sensor tests
- Stale command and disconnect behavior
- Publication rate and timestamp monotonicity
- Scene reloads without duplicate publishers or subscriptions
- Representative bulky-object and constrained-passage configurations

## Output

- Logic to isolate
- Unity-facing behavior to retain
- Suggested seams or adapters
- EditMode cases
- PlayMode cases
- Unity–ROS integration cases
- Determinism and tolerance strategy

## Guardrails

- Do not require a live ROS graph for logic that can be tested locally.
- Do not mock away the message and coordinate contracts that integration tests must verify.
- Prefer a few meaningful tests over abstraction created solely for coverage.
