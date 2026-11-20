# ADR 0020: Unity as the renderer of the ROS-free MuJoCo plant (plant viewer mode)

Date: 2026-11-20. Status: accepted (design approved by the user, 2026-11-20). Design:
[plant viewer design](../superpowers/specs/2026-11-20-plant-unity-viewer-design.md); plan:
[stage 4A](../superpowers/plans/2026-11-20-plant-viewer-4a.md); record:
[plant-viewer README](../experiments/plant-viewer/README.md). Builds on [ADR 0016](0016-mujoco-campaign-plant.md)
(the MuJoCo plant) and [ADR 0019](0019-route-search-ports-smac-lattice.md) (its route search).

## Context

The project's original split ([AGENTS.md](../../AGENTS.md)) has ROS 2 own navigation, planning and control, and
Unity own physics, simulated sensors and rendering. The MuJoCo plant (spec 2) runs the framework without ROS: MuJoCo
simulates the robot and the lidar, the route planner, F2 and B5 plan and control in process, and `plant_demo_server`
streams the session over TCP. Demonstration videos of that framework need a renderer. Unity already has the
construction site, the robot model and the camera rig.

## Options considered

- **A: Unity as a second simulator.** Unity's physics and sensors drive the robot, as in the ROS mode. Rejected: two
  plants would disagree, and a video would no longer show the system that was measured.
- **B: Unity as a renderer only.** Unity draws the backend's state and sends only explicit user requests (goals,
  cancel, run, pause, step, speed, reset). Chosen.
- **B inside the ROS simulator scene.** Rejected: the scene's ROS, physics and sensor components would have to be
  disabled at runtime, and a missed component would simulate behind the viewer's back.

## Decision

A second mode, beside the unchanged ROS simulator mode:

- **MuJoCo** owns physics and the simulated lidar.
- **The framework** (route planner, goal manager, F2, B5 in `mobile_manipulator_campaign`) owns planning and control.
- **Unity** owns rendering, the cameras, the HUD, the minimap and the view journal. It sends only commands the user
  asked for, and never a reset on reconnect.

The mode lives in its own scene, `Assets/Scenes/PlantViewer.unity`, built from ConstructionSiteV2 by
`build_plant_viewer`. It has no ROS, physics, sensor or controller component, and a scene-contract test enforces
that. The macOS player contains this scene only. The stream protocol (schema 3,
[stream-protocol.md](../experiments/mujoco-plant/stream-protocol.md)) is the only interface between the two sides.

## Consequences

- The ROS simulator mode, its scenes and its ownership rules are unchanged.
- A video shows the measured system: the robot pose, routes and goal states come from the backend, and the view
  journal lets 4C re-render a take from its run log.
- The viewer adds no package. It depends on the protocol: a backend change to frames or commands needs a viewer
  change, and the viewer refuses another schema.
- Unity's ConstructionSiteV2 stays the art source. The viewer scene is rebuilt from it, and its site checksum must equal
  the backend's, or the HUD warns.

## Revisit when

- the viewer must show something the protocol does not carry (then extend the protocol, not the scene);
- the Windows player is built (4C), or a second robot (Husky) joins the plant.
