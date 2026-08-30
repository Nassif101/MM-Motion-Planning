# Agent skill policy

Project-owned skills under `.agents/skills` are versioned with the repository. Upstream Unity skills are installed per host and listed in `.agents/official-skills.txt`; this avoids silently vendoring an entire changing upstream catalog.

Install or refresh the Unity CLI skill for Codex at user scope from the installed Unity CLI:

```bash
unity skill install codex --yes
unity skill refresh --yes
```

Do not use `--local` with the current Unity CLI beta: it appends the full CLI manual to `AGENTS.md` instead of creating a compact discoverable project skill.

`unity-package-management` comes from the official `Unity-Technologies/skills` collection. Install other listed specialists only when their domain is actually part of the requested work.

## Enabled project domains

- Unity CLI/Pipeline and UPM package management
- Unity/ROS architecture, scene contracts, scripts, async lifecycle, tests, and ADRs
- ROS 2 runtime and engineering
- Blender asset construction when explicitly requested
- 3D PhysX collision diagnostics for the simulated robot and environment

## Excluded by default

- URP post-processing and URP Render Graph validation: this project uses HDRP
- Unity AI Navigation for the robot: ROS 2 owns navigation and planning
- new-project scaffolding: this is an established repository
- ads, IAP, live services, multiplayer, and Vivox: outside the research simulation scope
- WebGL/WebGPU optimization: no web build target is currently defined

All other currently available Unity specialist skills were removed because the repository has no corresponding project-owned implementation surface. A deliberate architecture or product-scope change should update `AGENTS.md`, `.agents/official-skills.txt`, and this document together before reinstalling one.
