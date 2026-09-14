# C++ collision module — implementation status

Revision 1, 2026-09-14. This document records what the repository implements from the [architecture](architecture.md) and what remains dependent on physical machine evidence.

## Implemented path

`cDrive::Initialize` constructs the compiled pheno-inspired/fixed-table reference scene and starts `cCollisionSupervisor`. Start now carries its real joystick direction from the HTTP/CAN mocker. `cDriveController` creates a session, holds zero speed, publishes a preflight request and waits without running geometry on the drive callback.

The collision worker calculates a conservative travel distance covering the configured reaction interval and braking distance. It maps each predicted pose through the existing relative-A2 kinematics, transforms all link/C-arm box bodies, and checks robot/environment plus selected self-collision pairs. A separating-axis OBB test supplies exact pose overlap and a separating gap. Adaptive interval subdivision certifies that the gap exceeds geometric margin plus an upper bound on body movement throughout the interval. A result that cannot be certified within bounded subdivision is `Unknown`, which has the same stop effect as `Hazard`.

The worker returns a finite permit with session, request sequence, scene generation and expiration time through a bounded single-producer/single-consumer mailbox. The drive consumes that exact permit once, issues one motion update, then immediately requests the next permit. Queue failure, result mismatch, expiry, direction change or collision-worker revocation calls the protective-stop path. A 1 ms independent monitor observes sticky worker revocation and writes zero speed even if no further joystick callback arrives. The final position-command gate uses a non-waiting lock and rechecks revocation before transport. The stopped state stays latched until `StopDrive` acknowledges the session.

## Source map

| Source | Responsibility |
| --- | --- |
| `includes/iCollisionSupervisor.h` | Drive-facing asynchronous supervision contract |
| `collision/include/cCollisionTypes.h` | SI geometry, request, permit, verdict and prediction settings |
| `collision/include/cSpscMailbox.h` | Fixed-capacity wait-free SPSC publication boundary |
| `collision/src/cSceneRegistry.cpp` | Compiled simulation scene matching reference parameters |
| `collision/src/cBodyKinematics.cpp` | RTMC A1/A2 and provisional A3/A4 3D body transforms and motion bounds |
| `collision/src/cProximityBackend.cpp` | Oriented-box separating-axis overlap and separation gap |
| `collision/src/cTrajectoryPredictor.cpp` | Braking travel, kinematic clipping and conservative interval certification |
| `collision/src/cCollisionSupervisor.cpp` | Worker lifecycle, mailboxes, session filtering and sticky stop request |
| `drive/src/cDriveController.cpp` | Preflight/running/latch states, 1 ms stop monitor and final permit gate |
| `CANMocker/src/cCANMocker.cpp` | Preserves selected direction in Start frames |
| `tests/test_collision.cpp` | Geometry, future-path, worker and integrated lifecycle tests |

The scene is compiled from the same numerical assumptions as `data/collision/reference/parameters.json`; the checked-in JSON/OBJ files remain inspection assets. Future measured models should be generated into a typed scene artifact or loaded through a validated parser so data and runtime constants cannot drift. Current artifact tests and collision tests independently check their respective representations but do not yet compare every compiled body field to generated scene JSON.

## Simulation constants

Prediction settings currently use 80 ms reaction time, 0.20 m/s maximum linear speed, 0.40 m/s² possible linear acceleration, 0.50 m/s² braking magnitude, 60°/s maximum angular speed, 120°/s² possible angular acceleration and braking, 20 mm pair margin, and 0.5 mm interval-motion tolerance. The worker assumes the maximum configured speed because measured velocity is absent. This yields 71.104 mm linear continuation/braking travel. All settings are checked for finite, valid ranges before any clear permit can be issued. These are simulation settings, not measured guarantees.

The application callback cadence remains input-driven at 50 ms. A permit lives for 150 ms and the controller allows 200 ms for preflight/result delivery. The reference-scene unit check requires one +X prediction at home to complete within 50 ms on the test host. This is a development threshold rather than target-hardware worst-case timing evidence.

## Known limits before physical use

- Position and velocity are commanded simulation state; there is no coherent measured feedback, sample age, tracking error or measured standstill.
- `SetSpeed(0)` is the only backend stop action and has no delivery, braking or standstill acknowledgement.
- The motion backend sends four independent CAN position frames without atomic sequence/commit semantics.
- Stopping travel uses configured scalar bounds and assumes monotonic movement along the requested command coordinate. Coordinated asynchronous physical axle braking remains unmodeled.
- The broad phase is pair-level separating-axis rejection for the current small scene. A static BVH/dynamic tree is still needed if measured scene density makes brute-force pair enumeration miss its deadline.
- Version one accepts one active axis direction. A direction change invokes a stop and requires Stop plus a new Start.
- The compiled scene is immutable after construction. The common geometry/frame representation supports future mobility changes, but atomic runtime scene generation replacement and tracked-object uncertainty are not implemented.
- Permanent link1/link2 and link2/C-arm joint pairs are excluded at rigid-body-pair level because the synthetic boxes overlap at their connections. Measured geometry needs reviewed local contact masks.
- Housing, table, patient fixture and A3/A4 frame values remain synthetic and `simulation_only`.

## Verification

The collision test executable covers rotated OBB overlap, future obstacle detection, invalid input and settings, asynchronous publication, sticky revocation, stationary preflight, permit renewal, direction-change stop, collision-triggered stop, duplicate Start rejection and Stop acknowledgement. The supplied home/+X reference query must be clear and complete within its simulation timing threshold, and the compiled application scene must grant multiple initial +X permits. Existing kinematic tests remain a separate regression suite.

See [verification.md](verification.md) for current commands and results. Hardware release still requires architecture gates G1–G5 and AVOID-20 evidence.
