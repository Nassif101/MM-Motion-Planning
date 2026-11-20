# Plant viewer stage 4A (first demo) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** a macOS standalone player that shows the live MuJoCo/F2/B5 backend and lets the user drive it to minimap
destinations, cancel, pause, step, change speed, reset and switch cameras, while writing a view journal.

**Architecture:** the container's `plant_demo_server` gains one command, `cancel_goal`. A pure C# core
(`MotionPlanningSim.Viewer.Core`: codec, client, state, commands, journal) has no Unity references and is tested in
EditMode. Thin Unity presenters (`MotionPlanningSim.Viewer`) draw a dedicated physics-free viewer scene built by an
Editor command.

**Tech Stack:** C++17/gtest/colcon (container); Unity 6000.5.2f1, HDRP 17.5.0, Input System 1.19.0, Newtonsoft
Json 3.0.2, NUnit EditMode tests, Unity CLI (`unity test`, `unity build`) on the macOS host.

**Spec:** [2026-11-20-plant-unity-viewer-design.md](../specs/2026-11-20-plant-unity-viewer-design.md) (sections 5-10).
Roadmap: [2026-11-18-plant-unity-viewer.md](2026-11-18-plant-unity-viewer.md).

## Global Constraints

- Wire format unchanged, schema 3: `[u32 length][u16 type][u32 json_len][json][u32 n_blocks]{[u32 n][float32 x n]}`,
  little endian; frame types Hello 1, Control 2, Event 3, State 10, Plans 11, Scan 12, Goal 20, Route 21.
- Frames over 16 MiB are malformed. Client queue 256 frames; State, Plans and Scan coalesce; Hello, Event, Goal and
  Route are never dropped.
- Reconnect every 2 s; never send `reset` on reconnect. Stale: running and no State for 0.25 s of wall time.
- Display lag 0.04 s behind the newest State; no extrapolation.
- Speed ladder 0.25, 0.5, 1, 2, 4. Hotkeys: Space run/pause, `.` step, `[`/`]` speed, C camera, M minimap,
  X cancel, Shift+R reset, F1 scenario picker. Free camera: WASD, Q/E, Shift, scroll speed, right-drag or
  Control-drag look.
- Map: `construction_site_v2.{yaml,pgm,metadata.json}`, resolution 0.05 m, origin (-25, -20); occupancy layer at
  40% opacity; goals disabled when the SHA-256s disagree with Hello `routing`.
- Wheels: `wheel_radius_m` 0.14, `effective_track_m` 3.0; arm joints `shoulder_pan_joint` ... `wrist_3_joint`.
- Journal: `~/Movies/PlantViewer/<UTC timestamp>/journal.jsonl`; at most 100 lines/s and 50 MB; cursor and
  free-camera lines at 10 Hz and thinned first.
- FPS target: 99th percentile frame time at most 33.3 ms at 1920 x 1080 in the macOS player, screen recorder running.
- Unity via CLI and `com.unity.pipeline` only, never Unity MCP; no new packages; never commit `Library/`, builds,
  journals, captures or host-absolute paths. The viewer scene holds no ROS, sensor, physics or controller component.
- Process: one commit per task on `plant-routing-unity-viewer`; after each task, stop and hand the user a review
  prompt (`docs/reviews/plant-viewer/`, git-ignored); stop Play mode when a task finishes; never overlap Unity with
  timed runs (fanless Mac). `unity test` needs the Editor closed on this project.

## Review Focus

1. **Reset or goal replacement while frames are in flight:** an old epoch's Goal/Route frame arriving after the new
   session Hello must not redraw an old marker or route. Test: Task 3 `IgnoresGoalAndRouteFromOlderEpoch`.
2. **A slow main thread (Editor hitch, minimap render) during a burst:** coalescing must keep every Goal/Route/Event in
   order. Test: Task 3 `OverflowKeepsKeptFramesInOrder`.
3. **Server killed and restarted mid-run:** the viewer reconnects and shows the new session without sending a reset.
   Test: Task 3 `ReconnectsWithoutSendingReset`.
4. **A click without a drag, or a drag across the map edge:** heading falls back to robot-to-point; off-map presses
   still submit and show the backend's rejection. Test: Task 6 `ClickWithoutDragUsesRobotToPointHeading`,
   `PressOutsideMapStillSubmits`.
5. **Cancel when nothing is active, or with only a scheduled goal:** a clear rejection, or the scheduled goal is
   canceled. Test: Task 1 `CancelWithoutGoalIsRejected`, `CancelRemovesScheduledSubmissions`.

---

## File map

| Path | Responsibility |
|---|---|
| `ros2_ws/src/mobile_manipulator_campaign/{include/.../goal_manager.hpp,src/goal_manager.cpp}` | `GoalManager::cancel` |
| `.../{include/.../session.hpp,src/session.cpp}` | `Session::cancel_goal`, capability |
| `.../src/demo_server.cpp` | the `cancel_goal` command |
| `docs/experiments/mujoco-plant/stream-protocol.md` | the command and capability |
| `motion-planning-sim/Assets/Scripts/Viewer/Core/*.cs` | pure C#: `FrameCodec`, `ViewerCommands`, `ViewJournal`, `StreamClient`, `ViewerState`, `GoalMarkerModel`, `MapFrame` |
| `motion-planning-sim/Assets/Scripts/Viewer/Unity/*.cs` | presenters, input, cameras, HUD, minimap, bootstrap |
| `motion-planning-sim/Assets/Scripts/Editor/{PlantViewerSetup,PlantViewerBuild}.cs` | builds scene/prefab; builds the player |
| `motion-planning-sim/Assets/Tests/EditMode/Viewer/*.cs` | `MotionPlanningSim.Viewer.EditModeTests` |
| `docs/experiments/plant-viewer/README.md` | results, problems and decisions |
| `docs/adr/0020-plant-viewer-renderer-mode.md`, `AGENTS.md` | ownership record |

Test commands used below:
- Backend (container): `colcon build --packages-select mobile_manipulator_campaign --build-base /tmp/mm-viewer/build --install-base /tmp/mm-viewer/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` then `colcon test --packages-select mobile_manipulator_campaign --build-base /tmp/mm-viewer/build --install-base /tmp/mm-viewer/install --ctest-args -R "<regex>"` and `colcon test-result --verbose --test-result-base /tmp/mm-viewer/build/mobile_manipulator_campaign` (source `ros2_ws/install/setup.bash` first, `MAKEFLAGS=-j2`).
- Unity (host, Editor closed): `unity test motion-planning-sim --mode EditMode --filter "<filter>" --timeout 900` (exit 0 pass, 8 failures).

---

### Task 1: `cancel_goal` in the backend

**Files:** Modify `goal_manager.hpp/.cpp`, `session.hpp/.cpp`, `demo_server.cpp`, `stream-protocol.md`. Test:
`test/test_goal_manager.cpp`, `test/test_generated_session.cpp`, `test/test_demo_server.cpp`.

**Interfaces:**
- Produces: `GoalAcknowledgement GoalManager::cancel(std::string reason);` and
  `GoalAcknowledgement Session::cancel_goal();` (reason `"client"`); command `{"cmd":"cancel_goal","client_request":n}`
  answered by the existing `goal_ack` event (`command: "cancel_goal"`); Hello `capabilities` of a generated-route
  session: `["goal_frames", "route_frames", "cancel_goal"]`.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(GoalManager, CancelEndsTheActiveGoal)          // submit, advance; cancel("client") -> action Applied,
                                                     // one Canceled event with that goal, active() false, reason "client"
TEST(GoalManager, CancelRemovesScheduledSubmissions) // schedule at 5 s; cancel -> one Canceled event with request,
                                                     // no goal record; advance(6) applies nothing
TEST(GoalManager, CancelWithoutGoalIsRejected)       // fresh manager: action Rejected, reason "no_goal", no event
TEST(GoalManager, CancelUnderBackpressureAnswersBackpressure) // journal full: action Backpressure, nothing changes
TEST(GeneratedSession, CancelHoldsTheRobot)          // v2_open_yard_nav lattice; goal tracking; cancel_goal();
                                                     // the next State header has "execution": "holding" and
                                                     // goal_canceled is the last Goal frame's event
TEST(DemoServer, CancelGoalAcknowledgesAndAdvertises) // generated server Hello capabilities contains "cancel_goal";
                                                     // reply goal_ack {command cancel_goal, action, client_request 7}
TEST(DemoServer, CancelGoalNeedsGeneratedRoutes)     // recorded-route server: error event
```

- [ ] **Step 2: Run them to verify they fail** — backend build fails with "no member named 'cancel'".
- [ ] **Step 3: Implement** `GoalManager::cancel` by reusing `reset`'s cancellation path (active goal to `Canceled`,
  each scheduled item canceled, room check first), without changing the epoch; `Session::cancel_goal` calls it and
  lets the existing terminal-event handling archive the goal and hold; add the `cancel_goal` branch next to
  `request_replan` in `DemoServer::handle` (generated servers only) and the capability in `session.cpp:327`.
- [ ] **Step 4: Run** `--ctest-args -R "test_goal_manager|test_generated_session|test_demo_server"` — all pass.
- [ ] **Step 5: Document** the command and capability in stream-protocol.md's Control table and Hello `capabilities`.
- [ ] **Step 6: Commit** `Spec 4A Task 1: cancel_goal ...`; write the review prompt; stop.

### Task 2: C# core — codec, commands and journal

**Files:** Create `Assets/Scripts/Viewer/Core/{MotionPlanningSim.Viewer.Core.asmdef,FrameCodec.cs,ViewerCommands.cs,
ViewJournal.cs}`; `Assets/Tests/EditMode/Viewer/{MotionPlanningSim.Viewer.EditModeTests.asmdef,FrameCodecTests.cs,
ViewerCommandsTests.cs,ViewJournalTests.cs}`.
Core asmdef: `noEngineReferences: true`, `overrideReferences: true`, `precompiledReferences: ["Newtonsoft.Json.dll"]`.
Test asmdef: references Core and (Task 4 on) `MotionPlanningSim.Viewer`, `nunit.framework.dll`, Editor only,
`UNITY_INCLUDE_TESTS`. Fixtures are read from `Path.Combine(Application.dataPath, "../../tools/plant/fixtures")`.

**Interfaces (namespace `MotionPlanningSim.Viewer.Core`):**
- `enum FrameType : ushort { Hello = 1, Control = 2, Event = 3, State = 10, Plans = 11, Scan = 12, Goal = 20, Route = 21 }`
- `sealed class Frame { FrameType Type; JObject Header; float[][] Blocks; }`
- `static class FrameCodec { const int MaxFrameBytes = 16 << 20; byte[] Encode(FrameType, JObject, IReadOnlyList<float[]>);
  bool TryDecode(ReadOnlySpan<byte> buffer, out Frame frame, out int consumed); }` — false when incomplete;
  throws `MalformedFrameException` on disagreeing lengths, unknown type, bad JSON or length > MaxFrameBytes.
- `sealed class ViewerCommands { JObject Hello(); JObject ListScenarios(); JObject Reset(string scenario, int seed);
  JObject Run(double speed); JObject Pause(); JObject Step(int n = 1); JObject SubmitGoal(double x, double y, double yaw);
  JObject CancelGoal(); long LastClientRequest { get; } }` — `client_request` increases by one per goal/cancel.
- `static class SpeedLadder { double Up(double s); double Down(double s); }` over 0.25, 0.5, 1, 2, 4 (clamped).
- `sealed class ViewJournal : IDisposable { ViewJournal(TextWriter w, Func<double> wallClock, int maxLinesPerSecond = 100,
  long maxBytes = 50_000_000); void WriteHeader(JObject sessionHello); bool Write(double simTime, string kind,
  JObject fields = null, bool thinnable = false); long Lines; long Thinned; }`

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void EncodesTheCommittedFixtureByteForByte()   // Encode(State, {"t":1.25,"name":"fixture","list":[1,2,3]},
                                                               // [[1.5,-2.0],[],[3.25]]) == frame_v1.bin
[Test] public void PartialBufferIsIncomplete()                // every prefix of frame_v1.bin: TryDecode false
[Test] public void DecodesGoalAndRouteFixtures()              // goal_v3: Goal, goal.goal_id 4, target [1.5,-2.25,0.5],
                                                               // state "planning"; goal_rejected_v3: goal null, request
                                                               // target [101.0,-2.25,0.5]; route_v3: 3 path poses, last
                                                               // (1.5,-2.25,0.25), points[1] (0.5,-0.5)
[Test] public void OversizedOrInconsistentFramesAreMalformed()// length 16 MiB + 1; json_len past the end; type 99
[Test] public void ClientRequestIncreasesPerGoalAndCancel()   // SubmitGoal -> 1, CancelGoal -> 2; JSON
                                                               // {"cmd":"submit_goal","target":[x,y,yaw],"client_request":1}
[Test] public void SpeedLadderClamps()                        // Up(4)=4, Down(0.25)=0.25, Up(1)=2
[Test] public void JournalHeaderCarriesSessionIdentity()      // first line has scenario, seed, mode, site_checksum,
                                                               // routing, provenance, epoch
[Test] public void JournalThinsThinnableLinesFirst()          // 150 thinnable + 20 normal in one wall second:
                                                               // all 20 normal kept, Lines <= 100, Thinned >= 70
[Test] public void JournalStopsAtByteCap()                    // maxBytes 1000: Write returns false past the cap
```

- [ ] **Step 2: Run** `--filter "MotionPlanningSim.Tests.Viewer"` — fails to compile (types missing).
- [ ] **Step 3: Implement** the four types. JSON serialization with `Formatting.None` (matches the C++ compact dump).
- [ ] **Step 4: Run** the same filter — all pass; the whole EditMode suite still passes (no filter).
- [ ] **Step 5: Commit** `Spec 4A Task 2: ...`; review prompt; stop.

### Task 3: C# core — stream client and viewer state

**Files:** Create `Viewer/Core/{StreamClient.cs,ViewerState.cs,GoalMarkerModel.cs}`; tests
`Viewer/{StreamClientTests.cs,ViewerStateTests.cs,GoalMarkerModelTests.cs}`. Tests use an in-process
`TcpListener` fake server on port 0 that writes encoded frames.

**Interfaces:**
- Consumes: Task 2's `FrameCodec`, `Frame`, `ViewerCommands`.
- `enum ConnectionStatus { Connecting, Connected, SchemaMismatch, Closed }`
- `sealed class StreamClient : IDisposable { StreamClient(string host, int port, Func<JObject> helloCommand, int capacity = 256,
  double retrySeconds = 2.0); void Start(); bool TryDequeue(out Frame f); void Send(JObject command);
  ConnectionStatus Status; long Dropped; long Malformed; }` — read thread decodes; on full queue the oldest
  State/Plans/Scan frame is discarded; a queue of more than 4 x capacity kept frames disconnects (counted in
  `Dropped`) and reconnects. Server Hello with `schema` != 3 sets `SchemaMismatch` and stops retrying.
- `readonly struct RobotSample { double T, X, Y, Yaw, V, Omega; double[] Joints; }` (6 joints)
- `sealed class ViewerState { void Apply(Frame f, double wallNow); void NoteRunning(bool running);
  JObject SessionHello; long Epoch; bool GeneratedRoutes; bool Has(string capability);
  bool TrySample(double lag, out RobotSample s); bool IsStale(double wallNow);
  double AchievedSpeed(double wallNow); IReadOnlyList<(string name, double x, double y)> Movers;
  IReadOnlyList<(string name, double x, double y, double yaw)> Bodies; JArray Boxes;
  IReadOnlyList<(double x, double y, double yaw)> RoutePath; long RouteId; GoalMarkerModel Goal; }`
  — keeps the last 64 States; samples at `newest.T - lag`, linear position, shortest-angle yaw and joints.
- `enum MarkerState { None, Pending, Planning, Tracking, Rejected, Superseded, Reached, Failed, TimedOut, Canceled }`
- `sealed class GoalMarkerModel { void NoteSubmitted(long clientRequest, double x, double y, double yaw);
  void ApplyAck(JObject goalAckEvent); void ApplyGoalFrame(JObject header); MarkerState State; string Reason;
  (double x, double y, double yaw)? Target; long GoalId; }`

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void HandshakeSendsHelloAfterServerHello()     // fake server sends Hello{schema:3}; receives
                                                               // {"cmd":"hello","schema":3}; Status Connected
[Test] public void SchemaMismatchStopsRetrying()              // server Hello schema 4 -> SchemaMismatch, one connect
[Test] public void OverflowKeepsKeptFramesInOrder()           // capacity 8; 40 States interleaved with Goal g1..g5 and
                                                               // Event e1..e3 before any dequeue: all Goal/Event dequeue
                                                               // in sent order, Dropped > 0, newest State is last State
[Test] public void ReconnectsWithoutSendingReset()            // server closes; second listener accepts within 3 s;
                                                               // the client sent only "hello" commands
[Test] public void FragmentedBytesDecode()                    // fixture written one byte per write
[Test] public void IgnoresGoalAndRouteFromOlderEpoch()        // Hello epoch 2 applied, then Route epoch 1: RoutePath empty
[Test] public void SessionHelloClearsSessionState()           // route + goal present; new session Hello -> none
[Test] public void SampleInterpolatesAtLag()                  // States t=1.00 x=0, t=1.02 x=1, newest t=1.06 x=3:
                                                               // TrySample(0.04) -> T 1.02, X 1; yaw 3.1 -> -3.1 wraps
[Test] public void StaleOnlyWhileRunning()                    // last State at wall 10.0; IsStale(10.3) true when
                                                               // running, false paused; false at 10.2
[Test] public void MarkerPendingUntilBackendAnswers()         // NoteSubmitted -> Pending; ack rejected "out_of_map" ->
                                                               // Rejected + reason; goal_v3 frame -> Planning, GoalId 4
[Test] public void MarkerFollowsSupersedeNotTheClick()        // tracking g4; NoteSubmitted new -> g4 still drawn until
                                                               // goal_superseded for g4, then the new goal's state
```

- [ ] **Step 2: Run** `--filter "MotionPlanningSim.Tests.Viewer"` — new tests fail to compile.
- [ ] **Step 3: Implement** the three types (one read thread; `ConcurrentQueue` plus a lock for coalescing;
  `Socket` with `NoDelay`; cancellation through `Dispose`).
- [ ] **Step 4: Run** — all pass, each socket test under 5 s; whole EditMode suite passes.
- [ ] **Step 5: Commit** `Spec 4A Task 3: ...`; review prompt; stop.

### Task 4: viewer scene, physics-free robot and environment

**Files:** Create `Assets/Scripts/Viewer/Unity/{MotionPlanningSim.Viewer.asmdef,ViewJoint.cs,PlantRobotView.cs,
PlantEnvironmentView.cs}`, `Assets/Scripts/Editor/PlantViewerSetup.cs`; generated (committed)
`Assets/Scenes/PlantViewer.unity`, `Assets/Robots/MobileManipulator/Prefabs/MobileManipulatorView.prefab`; tests
`Viewer/{ViewerSceneContractTests.cs,PlantRobotViewTests.cs}`.

**Interfaces:**
- Consumes: `RobotSample`, `ViewerState.Movers/Bodies/Boxes`; `RosMapPose.ToUnityPosition(double, double, float)`,
  `RosMapPose.ToUnityRotation(double)`.
- `sealed class ViewJoint : MonoBehaviour { string JointName; Vector3 Axis; Quaternion Rest; void SetAngle(float rad); }`
- `sealed class PlantRobotView : MonoBehaviour { float WheelRadius = 0.14f; float Track = 3.0f;
  void Apply(in RobotSample s, float dt); Transform PayloadAnchor; }` — base from x, y, yaw; arm joints by
  `ViewJoint.JointName`; wheels integrate `(V ∓ Omega * Track / 2) / WheelRadius * dt` (left minus, right plus).
- `sealed class PlantEnvironmentView : MonoBehaviour { void ApplySession(JObject hello); void ApplyState(ViewerState s); }`
  — boxes from Hello `boxes` skipping names with a `site/<name>` object in the scene, movers and bodies as boxes.
- Menu/CLI entry `PlantViewerSetup.Build()` (static, `[MenuItem("Tools/Plant Viewer/Build Scene")]`): copies V2's
  static art, creates the view prefab (records `ViewJoint` from each `ArticulationBody` before removing it, removes
  colliders and every component of the forbidden assemblies), stores `site_checksum` read from
  `ros2_ws/src/mobile_manipulator_plant/models/site_v2.xml` on a `PlantViewerSite` component.

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void ViewerSceneHasNoForbiddenComponents()   // open PlantViewer.unity: no ArticulationBody, Collider,
                                                            // no type from MotionPlanningSim.Runtime.ROS/Sensors/
                                                            // Control (except ThirdPersonRobotCamera), ROSTCPConnector
                                                            // or UnitySensors assemblies
[Test] public void ViewPrefabHasTenViewJoints()            // 4 wheel + 6 arm names from the URDF
[Test] public void FixturePosePlacesBaseAndArm()           // Apply(x 2, y -1, yaw pi/2, joints [0.1..0.6]): root
                                                            // position == ToUnityPosition(2,-1,h), rotation within
                                                            // 1e-4; each arm ViewJoint at its angle about its axis
[Test] public void WheelsSpinFromTwist()                   // V 0.14, Omega 0, dt 1: every wheel advanced 1 rad;
                                                            // V 0, Omega 0.1: left -1.0714, right +1.0714 rad
[Test] public void SceneRecordsSiteChecksum()              // PlantViewerSite.Checksum == value in site_v2.xml
```

- [ ] **Step 2: Run** `--filter "MotionPlanningSim.Tests.Viewer"` — fail (no scene/types).
- [ ] **Step 3: Implement** the components and `PlantViewerSetup`; run it with
  `unity run motion-planning-sim -- -executeMethod MotionPlanningSim.Editor.PlantViewerSetup.Build -logFile -`.
- [ ] **Step 4: Run** the tests — pass; whole EditMode suite passes; `ConstructionSiteV2.unity` unchanged in `git diff`.
- [ ] **Step 5: Commit** `Spec 4A Task 4: ...` (scene and prefab included); review prompt; stop.

### Task 5: bootstrap, cameras, controls, HUD and journal wiring

**Files:** Create `Viewer/Unity/{PlantViewerBootstrap.cs,PlantViewerInput.cs,FreeCamera.cs,PlantViewerHud.cs}`;
modify `PlantViewerSetup.cs` (adds them to the scene); test `Viewer/PlantViewerInputTests.cs`.

**Interfaces:**
- Consumes: Tasks 2-4. `ThirdPersonRobotCamera.SetView(RobotCameraView)`; its `target` and `payloadTarget` set to
  the view prefab's root and `PlantRobotView.PayloadAnchor`.
- `enum ViewerAction { RunPause, Step, SpeedDown, SpeedUp, NextCamera, ToggleMinimap, CancelGoal, Reset, ScenarioPicker }`
- `static class ViewerKeymap { ViewerAction? Resolve(Key key, bool shift, bool textFieldFocused); }` — the hotkeys of
  Global Constraints; R without Shift resolves to null.
- `PlantViewerBootstrap` — reads `-host`/`-port` (defaults `localhost`, `7700`), owns `StreamClient`, `ViewerState`,
  `ViewJournal` (opened at the first session Hello), drains the queue once per `Update`, applies to presenters,
  turns `ViewerAction`s into `ViewerCommands` (cancel only when `Has("cancel_goal")`), journals every action,
  camera change and the free camera pose at 10 Hz while it moves (`thinnable: true`). Compares Hello
  `site_checksum` with `PlantViewerSite.Checksum` and flags "site differs" on the HUD. F1 lists the reply to
  `ListScenarios()` and resets to the chosen one with seed 1. Disposes the client in `OnDestroy`.
- `FreeCamera : MonoBehaviour` — WASD/Q/E, Shift x4, scroll scales base speed (0.5-20 m/s), look on right button or
  Control+left drag.
- `PlantViewerHud` — one line: connection status, scenario/seed, sim time, requested/achieved speed, FPS, dropped and
  malformed counts, "paused", red "stale <age>", "site differs".

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void ResetNeedsShift()                  // Resolve(R,false,false) null; Resolve(R,true,false) Reset
[Test] public void HotkeysIgnoredWhileTyping()        // Resolve(Space,false,true) null
[Test] public void EveryActionHasOneKey()             // the nine actions resolve from exactly the listed keys
```

- [ ] **Step 2: Run** — fail to compile.
- [ ] **Step 3: Implement** the four components; rerun `PlantViewerSetup.Build`.
- [ ] **Step 4: Run** the tests — pass. Live check (Editor Play, demo server running in the container): connect,
  run/pause/step/speed, every camera, Shift+R, kill and restart the server (reconnects, no reset sent), journal file
  lines appear. Stop Play. Record what was seen in `docs/experiments/plant-viewer/README.md`.
- [ ] **Step 5: Commit** `Spec 4A Task 5: ...`; review prompt; stop.

### Task 6: minimap, goals and route

**Files:** Create `Viewer/Core/MapFrame.cs`, `Viewer/Unity/{PlantMinimap.cs,PlantGoalPresenter.cs,
PlantRoutePresenter.cs}`; modify `PlantViewerSetup.cs`, add `PlantViewerBuild.CopyMap()` (copies the three map
files from `ros2_ws/src/mobile_manipulator_navigation/maps/` into `Assets/StreamingAssets/PlantViewer/`, git-ignored);
tests `Viewer/{MapFrameTests.cs,MinimapGestureTests.cs}`.

**Interfaces:**
- `sealed class MapFrame { static MapFrame Load(string yamlPath); double Resolution; double OriginX, OriginY; int Width, Height;
  (double x, double y) PixelToWorld(double px, double py); (double px, double py) WorldToPixel(double x, double y);
  bool MatchesFingerprint(JObject routing, out string reason); }` — continuous pixel coordinates from the image's
  top-left corner (row 0 is map y max); `MatchesFingerprint` compares the SHA-256 of the three bundled files with
  `map_yaml_sha256`, `map_image_sha256` and `map_metadata_sha256` in Hello `routing` (plain file digests,
  `routing_inputs.cpp:195`).
- `static class MinimapGesture { (double x, double y, double yaw) Target((double x, double y) press,
  (double x, double y) release, (double x, double y) robot, double minDragM = 0.3); }`
- `PlantGoalPresenter` draws `GoalMarkerModel` on minimap and in scene (colour per `MarkerState`, reason text);
  `PlantRoutePresenter` draws `ViewerState.RoutePath` as a ground line and on the minimap. Goals disabled (reason
  shown) when not `GeneratedRoutes` or the fingerprint check fails. `PlantMinimap` journals open/close, press, drag
  and release (`thinnable: false`) and the cursor at 10 Hz while open (`thinnable: true`).
  `.gitignore` gains `motion-planning-sim/Assets/StreamingAssets/PlantViewer/` (and its `.meta`).

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void PixelWorldRoundTripAtCorners()        // (0,0) and (W,H): round trip within 1e-9;
                                                          // PixelToWorld(0.5, H - 0.5) == (-24.975, -19.975)
[Test] public void FingerprintMismatchDisablesGoals()    // altered image SHA -> false, reason names the file
[Test] public void DragSetsHeading()                     // press (0,0) release (1,1) -> yaw pi/4
[Test] public void ClickWithoutDragUsesRobotToPointHeading() // press=release (2,0), robot (0,0) -> yaw 0
[Test] public void PressOutsideMapStillSubmits()         // press (-30,0): Target returned unchanged (no clamping)
```

- [ ] **Step 2: Run** — fail to compile.
- [ ] **Step 3: Implement**; the minimap image is one orthographic top-down HDRP render at startup with the
  occupancy layer at 40%.
- [ ] **Step 4: Run** the tests — pass. Live check: a destination in open yard is accepted and reached; a
  replacement mid-drive supersedes; X cancels and the robot holds; an occupied-cell press shows the rejection
  reason; route line appears and clears. Stop Play. Record in the README.
- [ ] **Step 5: Commit** `Spec 4A Task 6: ...`; review prompt; stop.

### Task 7: macOS player, ownership record and the 4A evidence

**Files:** `Assets/Scripts/Editor/PlantViewerBuild.cs` (`Build()`: copy map, scene list = `PlantViewer.unity` only,
`Viewer` HDRP quality level as default, output `motion-planning-sim/Builds/PlantViewer-macOS/` git-ignored);
`.gitignore` (`motion-planning-sim/Builds/`); `docs/adr/0020-plant-viewer-renderer-mode.md`; `AGENTS.md`;
`docs/experiments/plant-viewer/README.md`;
`docs/running-the-stack.md` (a short "Plant viewer" section); roadmap decision/outcome lines.

- [ ] **Step 1: Build** `unity build motion-planning-sim --target StandaloneOSX --execute-method MotionPlanningSim.Editor.PlantViewerBuild.Build`
  — exit 0; the app launches and connects.
- [ ] **Step 2: Verify the player holds no ROS simulator:** the player log shows no ROS-TCP-Connector connection
  attempt and the build report lists only `PlantViewer.unity`.
- [ ] **Step 3: End-to-end in the player** (server: `plant_demo_server --mode realistic --routes lattice --lidar --log FILE`):
  the Task 5/6 live checks again, plus a 60 s screen recording of a minimap destination reached at 1x.
- [ ] **Step 4: FPS smoke test:** 1920 x 1080, screen recorder on, 10 s warm-up, 60 s measured on the longest V2
  destination; record p50/p99 frame time, whether 60 FPS was reached, and thermal state before and after. If p99 >
  33.3 ms, lower the `Viewer` quality level (shadows, volumetrics, resolution scale) and re-measure; record each change.
- [ ] **Step 5: Ownership:** ADR 0020 (renderer mode: MuJoCo physics and lidar, framework planning and control, Unity
  rendering plus explicit user requests; the ROS simulator mode unchanged) and the mode-specific `AGENTS.md` lines.
- [ ] **Step 6: Commit** `Spec 4A Task 7: ...`; review prompt; stop. 4A is done when Steps 3-4 are recorded.
