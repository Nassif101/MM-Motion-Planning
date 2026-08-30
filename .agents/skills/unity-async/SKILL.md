---
name: unity-async
description: Choose and review scheduling, asynchronous work, and lifecycle cleanup for this Unity–ROS simulation. Use when deciding among events, Update or FixedUpdate, coroutines, tasks, timers, ROS callbacks, sensor loops, or when diagnosing cancellation, shutdown, or main-thread problems.
---

# Unity Async and Lifecycle

## Decision Order

1. Prefer an event, callback, or explicit method call when work is not continuous.
2. Use `FixedUpdate` for physics-timestep work that must follow Unity physics.
3. Use `Update` or `LateUpdate` only for genuine frame-driven behavior.
4. Use a coroutine for short Unity-bound sequences or polling with Unity yield instructions.
5. Introduce a task library only if it is already present or its benefit justifies a new dependency. This project does not currently declare UniTask.

## Unity–ROS Rules

- Assume ROS or network callbacks may occur outside the Unity main thread until verified.
- Marshal Unity object, scene, and rendering changes to the Unity main thread.
- Decouple inbound message receipt from frame consumption when message rates can exceed rendering or physics rates.
- Define queue size, stale-data policy, timestamp handling, and backpressure for sensor and control streams.
- Keep physics sampling and ROS publication rates explicit; do not assume frame rate equals either one.

## Lifecycle Ownership

For every scheduled operation, identify:

- who starts it
- what owns its cancellation
- what happens on `OnDisable`, `OnDestroy`, scene reload, ROS disconnect, and editor exit
- whether reconnecting creates duplicate subscriptions or loops

Use symmetric subscribe/unsubscribe and start/stop paths. Cache references used in hot loops.

## Output

- Recommended scheduling model
- Thread and timing assumptions
- Lifecycle and cancellation owner
- Queue, rate, and stale-data policy
- Failure/reconnect behavior
- Performance risks

## Guardrails

- Do not add async machinery when an event or coroutine is sufficient.
- Do not touch Unity APIs from an unverified background thread.
- Do not allow disabled or destroyed scene objects to retain subscriptions or background work.
