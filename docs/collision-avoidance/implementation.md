# C++ collision module — implementation status

Revision 2, 2026-09-14. This document records what the repository implements from the [architecture](architecture.md) and what remains dependent on physical machine evidence.

## Implemented path

`cDrive::Initialize` constructs the compiled pheno-inspired/fixed-table reference scene and starts `cCollisionSupervisor`. Start now carries its real joystick direction from the HTTP/CAN mocker. `cDriveController` creates a session, holds zero speed, publishes a preflight request and waits without running geometry on the drive callback.

The collision worker calculates a conservative travel distance covering the configured reaction interval and braking distance. It maps each predicted pose through the existing relative-A2 kinematics, transforms all link/C-arm box bodies, and checks robot/environment plus selected self-collision pairs. A separating-axis OBB test supplies exact pose overlap and a separating gap. Adaptive interval subdivision certifies that the gap exceeds geometric margin plus an upper bound on body movement throughout the interval. A result that cannot be certified within bounded subdivision is `Unknown`, which has the same stop effect as `Hazard`.

The worker returns a finite permit with session, request sequence, scene generation and expiration time through a bounded single-producer/single-consumer mailbox. The drive consumes that exact permit once, issues one motion update, then immediately requests the next permit. Queue failure, result mismatch, expiry, direction change or collision-worker revocation calls the protective-stop path. A 1 ms independent monitor observes sticky worker revocation and the 150 ms renewal deadline and writes zero speed even if no further joystick callback arrives. The final position-command gate uses a non-waiting lock and rechecks revocation before transport. The stopped state stays latched until `StopDrive` acknowledges the session.

## Source map

| Source | Responsibility |
| --- | --- |
| `includes/iCollisionSupervisor.h` | Drive-facing asynchronous supervision contract |
| `collision/include/cCollisionTypes.h` | SI geometry, request, permit, verdict and prediction settings |
| `collision/include/cSpscMailbox.h` | Fixed-capacity wait-free SPSC publication boundary |
| `collision/src/cSceneRegistry.cpp` | Compiled simulation scene matching reference parameters |
| `collision/src/cBodyKinematics.cpp` | RTMC A1/A2 and A3 alignment and provisional A4/A5 3D body transforms and motion bounds |
| `collision/src/cProximityBackend.cpp` | Oriented-box separating-axis overlap and separation gap |
| `collision/src/cTrajectoryPredictor.cpp` | Braking travel, kinematic clipping and conservative interval certification |
| `collision/src/cCollisionSupervisor.cpp` | Worker lifecycle, mailboxes, session filtering and sticky stop request |
| `drive/src/cDriveController.cpp` | Preflight/running/latch states, 1 ms stop monitor and final permit gate |
| `CANMocker/src/cCANMocker.cpp` | Preserves selected direction in Start frames |
| `tests/test_collision.cpp` | Geometry, future-path, worker and integrated lifecycle tests |

The scene is compiled from the same numerical assumptions as `data/collision/reference/parameters.json`; the checked-in JSON/OBJ files remain inspection assets. Future measured models should be generated into a typed scene artifact or loaded through a validated parser so data and runtime constants cannot drift. Current artifact tests and collision tests independently check their respective representations but do not yet compare every compiled body field to generated scene JSON.

## Simulation constants

Prediction settings currently use 250 ms reaction allowance, 0.20 m/s maximum linear speed, 0.40 m/s² possible linear acceleration, 0.50 m/s² braking magnitude, 60°/s maximum angular speed, 120°/s² possible angular acceleration and braking, 20 mm pair margin, and 0.5 mm interval-motion tolerance. The worker assumes the maximum configured speed because measured velocity is absent. This yields 152.5 mm linear continuation/braking travel. All settings are checked for finite, valid ranges before any clear permit can be issued. These are simulation settings, not measured guarantees.

The application callback cadence remains input-driven at 50 ms. A permit lives for 150 ms and the controller allows 150 ms for preflight/result delivery. The reference-scene unit check requires one +X prediction at home to complete within 50 ms on the test host. This is a development threshold rather than target-hardware worst-case timing evidence.

## Known limits before physical use

- Position and velocity are commanded simulation state; there is no coherent measured feedback, sample age, tracking error or measured standstill.
- `SetSpeed(0)` is the only backend stop action and has no delivery, braking or standstill acknowledgement.
- The motion backend sends five independent CAN position frames without atomic sequence/commit semantics.
- Stopping travel uses configured scalar bounds and assumes monotonic movement along the requested command coordinate. Coordinated asynchronous physical axle braking remains unmodeled.
- The broad phase is pair-level separating-axis rejection for the current small scene. A static BVH/dynamic tree is still needed if measured scene density makes brute-force pair enumeration miss its deadline.
- Version one accepts one active axis direction. A direction change invokes a stop and requires Stop plus a new Start.
- The compiled scene is immutable after construction. The common geometry/frame representation supports future mobility changes, but atomic runtime scene generation replacement and tracked-object uncertainty are not implemented.
- Permanent link1/link2, link2/alignment, alignment/C-arm and legacy link2/C-arm interface pairs are excluded at rigid-body-pair level because the synthetic boxes overlap at their connections. Measured geometry needs reviewed local contact masks.
- Housing, table, patient fixture and A3/A4/A5 frame values remain synthetic and `simulation_only`.

## Verification

The collision test executable covers rotated OBB overlap, future obstacle detection, invalid input and settings, asynchronous publication, sticky revocation, stationary preflight, permit renewal, direction-change stop, collision-triggered stop, duplicate Start rejection and Stop acknowledgement. The supplied home/+X reference query must be clear and complete within its simulation timing threshold, and the compiled application scene must grant multiple initial +X permits. Existing kinematic tests remain a separate regression suite.

See [verification.md](verification.md) for current commands and results. Hardware release still requires architecture gates G1–G5 and AVOID-20 evidence.

## Revision 2: head frame, alignment and live integration

The fixed patient head center is (0,0,1.20) m, with a distinct static head proxy.
A1/A2 keep the original planar solve. A3=-(A1+A2) holds support/C-arm heading
constant in the patient frame; A4=LAO and A5=CRAN rotate about the current imaging
center. All five joints participate in the movement budget and CAN target layout.

`pcan_demo` serves the joystick console and live viewer at localhost:8082.
The viewer consumes backend commanded axle state rather than animating local
inputs. It displays head and imaging-center markers, alignment angle, lifecycle,
reason and permit count. Manual pose changes remain available only offline.
Startup validates assets, worker initialization and socket binding.

Collision calculations include five-axis transforms and box-corner sweep radii.
A telemetry mutex protects pose/status snapshots. The final command gate and
independent monitor coordinate their outputs; a late Running transition cannot
overwrite the sticky avoidance lifecycle. A3 support is grouped separately from
link 2; synthetic interface exclusions remain an explicit limitation.

The mandatory completion checklist is architecture section 17.5. CTest includes
`runtime_avoidance` when Python is installed. It tests the real executable,
including predictive pedestal stopping and persistent latch, rather than only
the collision library. See verification.md for actual results.
