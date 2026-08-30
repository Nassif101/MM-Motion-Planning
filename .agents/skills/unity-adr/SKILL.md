---
name: unity-adr
description: Record Unity architecture decisions for this robotics simulation by comparing options, tradeoffs, consequences, and revisit conditions. Use when choosing between technical approaches, packages, communication patterns, scene structures, ROS integration strategies, or test approaches whose rationale should survive beyond the current task.
---

# Unity Architecture Decisions

Record only decisions that materially affect the Unity simulation, its ROS boundary, or future implementation work.

## Workflow

1. Inspect the current Unity and ROS setup before comparing options.
2. State the concrete constraint or problem.
3. Compare the smallest credible set of options.
4. Choose one option and explain why it fits this project now.
5. Record consequences, validation evidence, and conditions that would justify revisiting it.

## Output

- Decision
- Context and constraints
- Options considered
- Chosen option and rationale
- Consequences and risks
- Validation plan
- Revisit triggers

## Relevant Decisions

- Whether Unity or ROS owns a piece of runtime state or behavior
- ROS-TCP Connector messages versus another integration mechanism
- Scene-authored configuration versus `ScriptableObject` assets
- Direct references, interfaces, or events between simulation components
- Coroutine, `Update`, or another scheduling mechanism
- EditMode, PlayMode, or Unity–ROS integration testing
- When project-owned scripts need assembly definitions

## Guardrails

- Keep ADRs short and tied to evidence from this repository.
- Treat Unity–ROS message schemas, coordinate frames, time behavior, and ownership boundaries as explicit compatibility constraints.
- Do not preserve options or terminology inherited from another repository unless they apply here.
