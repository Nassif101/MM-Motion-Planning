# Motion and Hardware Safety

Apply this gate before publishing velocity, position, effort, trajectory, navigation, manipulation, controller-switch, or hardware-state commands.

## Preflight

1. Identify the exact robot, namespace, environment, and whether the target is simulation or physical hardware.
2. Resolve the command interface, schema, units, frame, joint order, limits, and expected controller.
3. Confirm fresh feedback and timestamps.
4. Confirm lifecycle, hardware, and controller readiness.
5. Determine command ownership: active goals, teleoperation, safety nodes, controller claims, and arbitration behavior.
6. Identify the stop, cancel, or rollback path before sending the command.
7. Bound magnitude, duration, goal tolerance, and timeout to the smallest useful values.
8. Inspect the local surroundings or collision model when the motion can contact anything.

If identity, units, frame, ownership, feedback freshness, stopping behavior, or limits remain ambiguous, do not command motion.

## Execution

- Prefer one bounded command or goal over an open-ended stream.
- Keep status observation active when practical.
- Do not disable safety systems, limits, collision checking, watchdogs, or emergency-stop logic to make a test pass.
- Treat an MCP timeout as unknown command state, not automatic failure.
- Avoid controller switches while commands are in flight unless the established recovery procedure requires it.

## Verification and Recovery

- Verify velocity, pose, joint state, controller state, and action result as applicable.
- Confirm the robot stops when the bounded command ends or a goal is cancelled.
- If response differs from expectation, issue the established safe stop/cancel action when doing so is itself unambiguous, then diagnose.
- Classify before retry: transport failure, rejected command, stale feedback, frame error, ownership conflict, controller/hardware failure, collision/planning failure, or continuing execution.
- Never blind-retry a motion command.

The user authorizing a robotics task does not remove these technical preconditions.
