# Runtime Workflows

## Graph and Interfaces

1. Confirm the ROS-MCP connection.
2. Discover the exact graph entity.
3. Resolve its type and full request, goal, or message fields.
4. Inspect publishers, subscribers, server availability, or action details.
5. Act only with a schema-valid payload.
6. Verify resulting state through a distinct read.

Do not treat name similarity as identity. Namespaces, remaps, and generated interface versions matter.

## Topics and QoS

- Inspect type and endpoint details before subscribing or publishing.
- Sample first when units, ranges, frame IDs, timestamps, or enumeration values are unclear.
- Check that a consumer exists before publishing a command.
- For sensors, expect best-effort QoS to be common but verify it; do not infer it.
- Diagnose silence by separating: no publisher, type mismatch, QoS incompatibility, wrong namespace/domain, stale timestamps, and callback/executor blockage.
- Prefer bounded publication durations. Stop publication explicitly and verify the commanded effect ends.

## Services and Parameters

- Resolve the exact service type and request fields before calling.
- Treat transport success separately from application-level success fields.
- Read parameter descriptors, type, range, and current value before setting.
- Read back the value and observe its behavioral effect. Some nodes accept a value without applying it immediately.

## Actions

- Confirm the action server and goal schema.
- Check command ownership and existing goals before sending a goal.
- Observe acceptance, feedback or status, and terminal result.
- On timeout, determine whether the goal remains active before retrying.
- Cancel only the intended goal, then verify cancellation or another terminal state.

## Lifecycle

- Discover lifecycle state and available transition services.
- Apply only a valid transition for the current state.
- Verify the new state and inspect logs if callbacks fail.
- Avoid assuming node existence means it is configured or active.

## ros2_control

- Discover controller-manager namespace, service types, controller states, claimed interfaces, hardware component state, and joint ordering.
- Before switching, identify conflicts and confirm the intended controller will own every required command interface.
- Use strict switching semantics when partial success would be unsafe.
- After changing state, re-list controllers and hardware interfaces, then verify feedback moves only when commanded.
- Never infer command topic names from controller class names.

## Nav2 and MoveIt

- Resolve the installed action and service interfaces rather than assuming a distribution-specific API.
- Confirm localization, TF, costmaps or planning scene, current state, controller availability, and command ownership.
- For Nav2, verify map/odom/base frame connectivity and whether another navigation goal is active.
- For MoveIt, verify robot description, joint state freshness, planning group, constraints, collision state, controller mapping, and execution ownership.
- Treat plan success and execution success as separate outcomes.

## Sensors

- Check topic type, frame ID, timestamp freshness, rate, QoS, and plausible value ranges.
- Distinguish simulated ground truth from noisy sensor output.
- For images or point clouds, use ROS-MCP sampling/viewing capabilities first and avoid sustained high-bandwidth subscriptions without need.
- Correlate sensor timestamps with the active ROS time source.
