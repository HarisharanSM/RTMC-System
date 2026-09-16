# C++ collision module — implementation status

Revision 4, 2026-09-15. This document records what the repository implements from the [architecture](architecture.md) and [head-side redesign](head-side-clearance-design.md), and what remains dependent on physical machine evidence.

The integrated `index.html` canvas and drive-originated CAN feedback workflow
are implemented according to [architecture revision 3, section 18](architecture.md#18-integrated-joystick-c-arm-display-and-can-workflow).
The runtime has no iframe. UI press/hold/Stop requests use revision-3 CAN fields;
drive commits are encoded as five fixed-point angle frames, speed and a commit
marker. Only a complete matching sequence updates the pose returned by `/state`.
An unchanged-pose CAN heartbeat keeps feedback age meaningful while stationary.

## Implemented path

`cDrive::Initialize` constructs the compiled pheno-inspired/fixed-table reference scene and starts `cCollisionSupervisor`. Start now carries its real joystick direction from the HTTP/CAN mocker. `cDriveController` creates a session, holds zero speed, publishes a preflight request and waits without running geometry on the drive callback.

The collision worker calculates a conservative travel distance covering the configured reaction interval and braking distance without accelerating beyond the enforced drive speed cap. It maps each predicted pose through the existing relative-A2 kinematics, transforms all link/C-arm box bodies, and checks robot/environment plus selected self-collision pairs. A separating-axis OBB test supplies exact pose overlap and a cheap separation bound; unresolved diagonal cases use exact box closest-feature distance. Adaptive interval subdivision certifies that distance exceeds the 10 mm residual gap plus an upper bound on body movement throughout the interval. A result that cannot be certified within bounded subdivision is `Unknown`, which has the same stop effect as `Hazard`.

The worker returns a finite permit with session, request sequence, scene generation and expiration time through a bounded single-producer/single-consumer mailbox. The drive consumes that exact permit once, issues one motion update, then immediately requests the next permit. Queue failure, result mismatch, expiry, direction change or collision-worker revocation calls the protective-stop path. A 1 ms independent monitor observes sticky worker revocation and the 150 ms renewal deadline and writes zero speed even if no further joystick callback arrives. The final position-command gate uses a non-waiting lock and rechecks revocation before transport. The stopped state stays latched until `StopDrive` acknowledges the session.

## Source map

| Source | Responsibility |
| --- | --- |
| `includes/iCollisionSupervisor.h` | Drive-facing asynchronous supervision contract |
| `collision/include/cCollisionTypes.h` | SI geometry, request, permit, verdict and prediction settings |
| `collision/include/cSpscMailbox.h` | Fixed-capacity wait-free SPSC publication boundary |
| `collision/src/cSceneRegistry.cpp` | Compiled simulation scene matching reference parameters |
| `collision/src/cBodyKinematics.cpp` | RTMC A1/A2 and A3 alignment and provisional A4/A5 3D body transforms and motion bounds |
| `collision/src/cProximityBackend.cpp` | Oriented-box SAT overlap/broad rejection and closest-feature 3D distance |
| `collision/src/cTrajectoryPredictor.cpp` | Braking travel, kinematic clipping and conservative interval certification |
| `collision/src/cCollisionSupervisor.cpp` | Worker lifecycle, mailboxes, session filtering and sticky stop request |
| `drive/src/cDriveController.cpp` | Preflight/running/latch states, 1 ms stop monitor and final permit gate |
| `PCAN/src/cPCANController.cpp` | Revision-3 input decoding, fixed-point drive feedback, coherent assembly and heartbeat |
| `CANMocker/src/cCANMocker.cpp` | Versioned press/hold/Stop CAN frames, integrated page/model routes and state API |
| `ui/index.html` | Integrated model canvas, joystick lifecycle, five-axis CAN feedback and stale-state handling |
| `tests/test_collision.cpp` | Geometry, future-path, worker and integrated lifecycle tests |
| `tests/test_runtime.py` | Real executable, integrated page, CAN command/feedback, watchdog and predictive stop acceptance |

The scene bodies and joint-interface pair policy are emitted as typed C++ data from `data/collision/reference/parameters.json`, alongside the JSON/OBJ inspection assets. The compiled predictor and browser therefore consume artifacts from one generation step; reproducibility tests cover the complete generated set.

## Simulation constants

Prediction settings currently use 250 ms reaction allowance, 0.20 m/s maximum linear speed, 0.40 m/s² possible linear acceleration, 0.50 m/s² braking magnitude, 60°/s maximum angular speed, 120°/s² possible angular acceleration and braking, 10 mm residual pair gap, and 0.5 mm interval-motion tolerance. The worker assumes maximum configured speed because measured velocity is absent, but caps reaction acceleration at that speed. This yields 90 mm linear and 30° angular continuation/braking travel. Folded-home certification permits up to 20 interval subdivisions. All settings are validated before any clear permit can be issued. These are simulation settings, not measured guarantees.

The application callback cadence remains input-driven at 50 ms. A permit lives for 150 ms and the controller allows 150 ms for preflight/result delivery. The reference-scene unit check requires one +X prediction at home to complete within 50 ms on the test host. This is a development threshold rather than target-hardware worst-case timing evidence.

## Known limits before physical use

- Position and velocity are coherent, sequenced commanded feedback with sample age; there is no encoder measurement, tracking error or measured standstill.
- `SetSpeed(0)` is the only backend stop action and has no delivery, braking or standstill acknowledgement.
- Actuator targets still lack a physical multi-axis commit acknowledgement. The simulator's UI feedback uses five sequenced angle frames, speed and an atomic commit marker.
- Stopping travel uses configured scalar bounds and assumes monotonic movement along the requested command coordinate. Coordinated asynchronous physical axle braking remains unmodeled.
- The broad phase is pair-level separating-axis rejection for the current small scene. A static BVH/dynamic tree is still needed if measured scene density makes brute-force pair enumeration miss its deadline.
- Version one accepts one active axis direction. A direction change invokes a stop and requires Stop plus a new Start.
- The compiled scene is immutable after construction. The common geometry/frame representation supports future mobility changes, but atomic runtime scene generation replacement and tracked-object uncertainty are not implemented.
- Eight declared synthetic bearing/interface pairs are excluded at rigid-body-pair level because their boxes overlap at connections. They are generated with the scene data. Measured geometry still needs reviewed local contact masks.
- Housing, table, patient fixture and A3/A4/A5 frame values remain synthetic and `simulation_only`.

## Verification

The collision test executable covers rotated OBB overlap, future obstacle detection, invalid input and settings, asynchronous publication, sticky revocation, stationary preflight, permit renewal, direction-change stop, collision-triggered stop, duplicate Start rejection and Stop acknowledgement. The supplied home/+X reference query must be clear and complete within its simulation timing threshold, and the compiled application scene must grant multiple initial +X permits. Existing kinematic tests remain a separate regression suite.

See [verification.md](verification.md) for current commands and results. Hardware release still requires architecture gates G1–G5 and AVOID-20 evidence.

## Revision 3: head frame and integrated CAN display

The fixed patient head center is (0,0,1.20) m, with a distinct static head proxy.
A1/A2 keep the original planar solve. A3=-(A1+A2) holds support/C-arm heading
constant in the patient frame; A4=LAO and A5=CRAN rotate about the current imaging
center. All five joints participate in the movement budget and CAN target layout.

`pcan_demo` serves one joystick and C-arm canvas page at localhost:8082. The
canvas loads the generated scene and consumes only coherent drive CAN feedback;
it does not animate input or use an iframe. It displays head and imaging-center
markers, all five angles, lifecycle, reason, permit count and feedback age.
Manual pose changes remain available only in the offline generated viewer.
Startup validates the page/model assets, worker initialization and socket binding.

Collision calculations include five-axis transforms and box-corner sweep radii.
A telemetry mutex and serialized feedback transmitter protect pose/status
assembly. Five fixed-point angle frames, speed and a matching commit are required
before a new display pose is published. The final command gate and
independent monitor coordinate their outputs; a late Running transition cannot
overwrite the sticky avoidance lifecycle. A3 support is grouped separately from
link 2; synthetic interface exclusions remain an explicit limitation.

The mandatory completion checklists are architecture sections 17.5 and 18.12. CTest includes
`runtime_avoidance` when Python is installed. It tests the real executable,
including predictive fixed-table stopping and persistent latch, rather than only
the collision library. See verification.md for actual results.

## Revision 4: head-side geometry and clearance behavior

A5/CRAN/CAUD now clamps to ±90° in the drive authority and generated model.
The integrated index and offline viewer use the same revised transform chain:
A3 continues to cancel A1+A2 yaw, the non-tilting rear beam/column receives a
fixed −90° mount rotation toward patient −X, and the imaging arc remains in its
neutral YZ plane around the patient. A4 rotates that arc about longitudinal X;
A5 rotates about the subsequent Y axis. The synthetic under-table link heights
are now 0.10/0.28 m and rear support reach is 0.75 m to avoid manufacturing an
unmodelled non-adjacent overlap at folded home.

The residual surface gap is 10 mm. SAT remains the cheap overlap/broad rejection
stage; near diagonal boxes use vertex/face and edge/edge closest-feature distance.
Reaction acceleration is capped at the configured drive speed, reducing the
unmeasured maximum-speed horizons from 152.5 to 90 mm and from 52.5° to 30°.
The fold singularity requires up to 20 bounded interval subdivisions. Unknown,
deadline failure and collision retain fail-closed stop/latch behavior.

The HTTP listener now closes its listening descriptor before joining its worker,
so SIGTERM reliably performs supervised shutdown on macOS. This change does not
alter command protocol or motion authority.

## Revision 5: drawing-based connected assembly

The physical collision frames now separate link 2, the column, compensated A3
boom, A4 carrier, A5 carrier and C-arm. A fixed K-to-world registration moves the
physical base and column headward while preserving the existing 75/100 cm drive
solve and keeping the imaging centre at patient-head X/Y zero. The neutral C is
authored in XZ and opens toward the table. A declared synthetic remote-centre
carrier keeps A4/A5 rotation about the imaging centre; its dimensions are
provisional because the source drawings are not dimensioned.

`parameters.json` now generates the browser scene, OBJ poses and the typed C++
scene/pair-policy include. The integrated index uses the same eight-frame chain,
offers side/back/top views and continues to move only from coherent drive CAN
feedback. Collision-stop telemetry includes the limiting 3D body pair.

The revision-5 host verification passed 25/25 kinematic scenarios (58 checks),
42 collision checks, 12 artifact tests with 18 reproducible generated files, and
24 real-executable runtime checks. CMake was unavailable on the host, so the
C++17 targets were compiled directly from the source lists in `CMakeLists.txt`.
