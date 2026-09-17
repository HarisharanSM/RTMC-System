# A3 left/right controls: detailed design and implementation plan

Date: 2026-09-17. Revision 6. Status: implemented for the simulation runtime; physical release and the extended geometry-audit cases listed below remain pending.

This extends [the revision-5 assembly](diagram-assembly-design.md) and [the avoidance architecture](architecture.md). It specifies two hold-to-run A3 buttons in the existing index page, independent axle rotation, retained heading during subsequent translation, coherent feedback and predictive collision supervision. The supplied machine geometry remains a synthetic simulation model.

## 1. Design decision and intended behavior

Interpret “rotate the A3 axle” as a joint jog: hold A1, A2, A4 and A5 fixed and change A3 about the column's vertical axis. The boom and downstream C-arm assembly rotate; the column stator and lower links remain stationary. Because the imaging centre is offset from A3, it moves along a horizontal arc. This motion differs from LAO/CRAN rotation around the imaging centre.

After release, retain the accepted A3 position. Later X/Y commands move in the fixed patient coordinate system while preserving the chosen world heading. Automatic alignment becomes `A3 = heading - (A1+A2)`, with heading initially zero. No release, Stop, reconnect or ordinary mode transition commands a return to zero.

This is the proposed interaction contract. Rotation about a fixed patient-head imaging centre would instead require coordinated A1/A2 compensation and is a separate motion mode, outside this two-button feature.

| Operator action | Required behavior |
| --- | --- |
| Hold left A3 button | Request negative A3 rotation, subject to preflight and stopping clearance |
| Hold right A3 button | Request positive A3 rotation under the same contract |
| Release either button | Controller Stop reaches drive and collision; retain last accepted pose |
| Hold while prediction becomes unsafe or unknown | Protective stop and latch; continued Hold cannot restart |
| Press opposite button before release | Stop current session; require release and a new press |
| Start X/Y after an A3 jog | Preserve selected world heading and move imaging centre along patient X/Y |
| Start LAO/CRAN after an A3 jog | Retain A3 and use existing downstream local A4/A5 axes |

Left and right specify placement and signed commands, not camera-dependent screen motion. Negative is clockwise and positive counterclockwise when viewed from +Z toward the floor. Camera changes never invert the controls.

## 2. Current source findings and required changes

The following are source-inspected findings, not new runtime test results.

| Current component | Finding | Required change |
| --- | --- | --- |
| `includes/commonDrive.h` | Four command coordinates and four joystick directions; five actuator angles already exist | Add explicit yaw state and A3 direction without adding a sixth actuator |
| `drive/src/cDriveCalculator.cpp` | IK always assigns `A3=-(A1+A2)`; FK reports the planar endpoint | Retain selected heading and report the actual offset imaging centre |
| `PCAN/src/cCANDriveHandler.cpp` | Eight-byte command directions 1–8 are converted to an eight-bit mask | Use explicit enum decoding for ten directions; never shift A3 into a byte mask |
| `CANMocker/src/cCANMocker.cpp` | Eight named joystick buttons | Add two named A3 commands using the existing session/sequence lifecycle |
| `collision/src/cBodyKinematics.cpp` | Frames already evaluate nonzero `A1+A2+A3`, including the rotating 1.15 m offset | Reuse transform order, generate its constants from shared model data and verify arbitrary A3 |
| `collision/src/cTrajectoryPredictor.cpp` | Reconstructs poses through compensated IK; interval extrema explicitly reset A3 to zero world heading | Add joint-jog prediction and heading-aware translation prediction |
| `ui/index.html` | A3 angle displayed; no A3 input; canvas uses actual five-angle frames | Add controls, heading/pivot readouts, capability gating and stale-pose handling |
| Generated scene | Body geometry and eight entire rigid-body pair exclusions generated together | Review exclusions under new yaw travel; replace broad exclusions with justified local interface handling |

Existing revision-5 pass counts do not establish independent A3 correctness. In particular, compensating A3 back to zero during prediction would authorize a different path from the one requested.

## 3. UI layout and feedback

Add a labelled horizontal A3 control row below the existing LAO/CRAN and X/Y pads, above Stop. It occupies the full sidebar width and remains a single row below the canvas on narrow displays.

```text
 Existing LAO / CRAN pad       Existing patient X / Y pad

                    A3 column rotation
 [ ↻ A3 − · hold ]     A3: −12.4°     [ A3 + · hold ↺ ]
               Heading: +8.0° from patient axis
          Rotates the boom around the column; centre moves

                     [ Stop / acknowledge ]
```

Suggested IDs: `a3-negative` and `a3-positive`; HTTP button names `A3-left` and `A3-right`. Both use the existing pointer capture, 50 ms ordered Hold loop, release/cancel, lost capture, window blur and visibility handlers. Permit only one active motion command across all ten buttons. A blocked or unsupported A3 feature must not disable Stop.

Accessible names: “Rotate A3 clockwise; hold to move” and “Rotate A3 counterclockwise; hold to move.” Use text as well as arrows and colour. If keyboard operation is added, Space/Enter keydown starts once, repeated keydown does not create sessions, and keyup/blur stops. Do not assign conflicting global left/right shortcuts.

Display physical joint A3 separately from world heading `A1+A2+A3`. The X/Y readout must show the imaging-centre displacement from H derived from the accepted CAN sample. Mark the A3 pivot at the boom frame origin; keep the fixed patient-head marker and moving imaging-centre marker visible. Side/back/top views continue to work. Optional limiting-pair highlighting consumes diagnostics associated with the same stopped session.

Never derive the rendered A3 angle from button duration or substitute the compensation formula for received A3. A heartbeat can refresh sample age without changing the pose; it cannot prove new movement. Partial, mismatched or stale feedback retains the previous complete pose and inhibits Start. The current page's pose assignment occurs before its age check; move acceptance behind both coherence and freshness validation.

## 4. State model and transform equations

### 4.1 Separate actuator state, Cartesian pose and heading

Use the accepted five actuator angles as the canonical committed state. Derive the imaging pose and world heading from those angles through one shared FK definition. Store selected heading for subsequent Cartesian commands from this accepted state, not from input accumulation.

Proposed additions:

- `joystickSignal.A3`, default zero, signed unit direction only.
- A typed motion kind: patient X, patient Y, A3 joint jog, A4 joint jog, A5 joint jog.
- `drivePosition.YawDeg`, default zero, meaning world heading; existing LAO/CRAN retain local A4/A5 joint values.
- Motion snapshot: accepted axles, derived pose, direction/kind, source time, session, sequence, model/policy generation, velocity bound source.
- Permit: exact request identity, covered trajectory kind/profile and expiry anchored to the snapshot timeline.

Append defaulted members carefully: audit all aggregate initializers, finite checks, equality/direction checks, serializers and tests. Never serialize native C++ structs directly on CAN. An omitted yaw in a legacy fixture means zero; a live nonzero yaw must not be erased by constructing an old four-field pose.

### 4.2 Forward kinematics

All equations below use metres and radians. Legacy boundaries convert cm/degrees explicitly.

```text
t = (-1.15, 0)                    K-to-world translation
B = (-0.25, 0)                   planar base in K
L1 = 0.75; L2 = 1.00; d = 1.15

Mxy = t + B + L1*[cos(q1),sin(q1)]
            + L2*[cos(q1+q2),sin(q1+q2)]
C = [Mxy.x, Mxy.y, 1.20]         A3 pivot / boom origin
psi = q1 + q2 + q3
I = C + Rz(psi)*[d,0,0]          imaging centre
reported_xy = I.xy - H.xy
R_carm = Rz(psi)*Rx(q4)*Ry(q5)
```

For a pure A3 jog, `q1,q2,q4,q5` remain fixed. Only q3 changes, so both I and the full upper assembly sweep about C. With A1/A2 fixed, `delta(psi)=delta(q3)`.

At closed home, a +10° jog gives approximately `I.xy=(-0.01747,+0.19970)` m; a −10° jog mirrors Y. The imaging centre travels about 0.2007 m along the arc. These are geometric examples, not authorization to move through that path.

### 4.3 Cartesian commands after a jog

For requested imaging target `I_target=H+[X/100,Y/100,0]` at retained psi:

```text
M_target_xy = I_target.xy - d*[cos(psi),sin(psi)]
M_target_K  = M_target_xy - t
(q1,q2)    = existing 2R geometric solve(M_target_K)
q3         = psi - q1 - q2
```

The mathematical 2R solve can be reused, but its internal target now represents M in K. Separate that solve from the public imaging-coordinate envelope check. Apply imaging limits to I, reachability to M relative to B, and all joint limits to q. Never pass shifted M through a check that assumes it is patient imaging X/Y.

At psi=0, this reduces to current numeric behavior. During X/Y translation the physical A3 angle can change to retain psi. During a pure A3 jog A1/A2 must not move. Validate accepted FK against requested pose after every commit and after switching motion kind.

Keep angles continuous in the permitted physical interval. Do not modulo-wrap A3 from +180° to −180° or select a different IK branch to hide a joint-limit violation. A4/A5 retain their existing local transform order; world anatomical naming after nonzero yaw must not be interpreted as a newly implemented patient-fixed tilt mode.

## 5. Workspace and speed policy

Preserve physical A3 limits of ±180° and A5 limits of ±90°. Preserve the existing patient imaging envelope X=[0,1.5] m, Y=[−0.25,0.25] m by default and evaluate it along the full stopping path for every motion kind.

This has a visible consequence: pure A3 jogging from exact home leaves X≥0 immediately in either direction. Therefore the default envelope denies home A3 rotation even if geometry is separated. Show “A3 rotation would leave the configured workspace,” with a distinct reason from collision. Permit valid rotation from suitable interior poses and test those explicitly. Do not silently enlarge the envelope to make home motion work. If home rotation is required later, provide a separately reviewed A3 workspace configuration; a fixed-centre coordinated mode would also change the requested motion semantics.

Proposed initial A3 simulation jog settings:

| Setting | Proposed value | Reason |
| --- | --- | --- |
| Maximum A3 jog rate | 10°/s | The 1.15 m imaging offset already moves about 0.201 m/s at this rate |
| A3 jog acceleration bound | 20°/s² | Explicit slower jog profile |
| Simulation braking magnitude | 20°/s² | Provisional fallback bound, independently named |
| Reaction allowance | 250 ms | Retain existing simulated timing budget |
| Permit lifetime / stop monitor | 150 ms / 1 ms | Retain existing supervisory contract |
| Residual pair gap | 10 mm | Separate from reaction and braking travel |

These are proposed software settings, not measured dynamics. The 0.20 m/s legacy limit applies to commanded X/Y translation, not automatically to every surface during joint rotation. If a universal tool/surface-speed limit is required, compute an additional radius-based angular cap and enforce it in both predictor and drive. Downstream body corners can move faster than I.

At 10°/s and the proposed braking bound, the maximum-speed reaction-plus-braking angle is 5°: 2.5° reaction plus 2.5° braking. Its imaging-centre arc length is approximately 100.4 mm. The actual forbidden-body sweep is larger and depends on the body, A4/A5 tilt and current pose.

Near limits, certify the complete stop. Do not reuse `FeasibleFraction` to clip a predicted stop to a reachable fraction and label it Clear. The first implementation may deny a full-speed jog before the boundary. Closer approach requires a lower profile whose transition and stop are explicitly permitted and enforced. Never clamp a reported initial speed down to a new cap before computing its stop.

## 6. CAN and controller protocol

Propose input protocol revision 4 with the existing eight-byte frame layout. Retain message IDs Start=0x002, Hold=0x001, Stop=0x003.

| Bytes | Meaning |
| --- | --- |
| 0 | Input protocol version 4 |
| 1 | Direction enum: 0 for Stop, 1–8 existing directions, 9 A3−, 10 A3+ |
| 2–3 | Little-endian 16-bit input sequence |
| 4–7 | Little-endian 32-bit session token |

Use a switch/table to decode enum values directly into motion kind and sign. Values 9/10 must not enter `1u << (direction-1)` followed by byte truncation. Reject invalid DLC, unknown version, enum or zero/multiple active motion directions before dispatch. Preserve duplicate, sequence-wrap and session validation.

Make command and feedback versions explicit. The five actuator target IDs 0x201–0x205 and five accepted angle feedback IDs 0x301–0x305 already carry A3. No extra A3 frame is needed. Retain the coherent commit/speed messages 0x306/0x307 and their established payload layout. If changing the existing global protocol constant, audit every producer and consumer rather than accidentally changing unchanged feedback semantics.

Startup capability metadata should advertise command version, A3 availability and model/policy identity. The index enables A3 only when supported. Default revision-4 runtime rejects revision-3 motion input; if backward compatibility is introduced, explicitly translate directions 1–8 without giving legacy clients A3 access. A legacy one-byte test frame cannot represent A3. Stop handling must remain available through the documented compatible path; malformed frames must never initiate movement.

## 7. Drive and supervisory sequence

```text
A3 press → HTTP command → CAN Start → controller → collision preflight
                                                  ↓ finite Clear
A3 Hold → HTTP command → CAN Hold → drive owner → covered A3 segment
                                                  ↓
                                  five-angle CAN sample + commit
                                                  ↓
                                       /state → index canvas

Hazard / Unknown / timeout → protective stop → session latched
Release → CAN Stop → controller → drive + collision acknowledge → Disarmed
Fresh press → new session and fresh preflight
```

The drive owner remains the sole writer of accepted position, joint state and motion profile. Preflight and all geometry execute on the collision worker. The command callback performs bounded intent and permit checks, proposes a deterministic segment, and commits only the covered profile. The worker/monitor revokes permission independently.

Bind permits to A3 direction, start snapshot, motion kind and model generation. A queued Clear for X/Y or the opposite A3 sign cannot authorize the current command. Release or a stop latch invalidates queued Hold and late permits. No automatic reverse, zeroing or obstacle-dependent restart is introduced.

Add source time to requests. Account for request queueing and calculation latency within the reaction budget; permit expiry must not be extended by worker delay. A held input only requests another segment; it does not renew permission on its own. Validate the starting axles and derived pose rather than reconstructing a different compensated state from stale X/Y.

## 8. Collision changes

### 8.1 Dedicated A3 trajectory

For a pure A3 jog, calculate `q3(f)=q3_start+sign*theta_stop*f` on [0,1], keeping the other four joints fixed. Evaluate actual frames for those q values. The swept set covers continued commanded motion and the modeled monotonic stop. If a future drive has nonmonotonic braking or other moving axes, expand the trajectory model before accepting those states.

Do not run this path through the current pose reconstruction or artificial A3 cancellation in `CheckPairInterval`. For X/Y at retained psi, obtain extrema from the shifted M path, retain `q3=psi-q1-q2`, and preserve the existing treatment of intermediate A1/A2 reversals. A4/A5 prediction must also preserve nonzero heading. A common trajectory evaluator shared with drive proposals prevents independent interpolation assumptions.

### 8.2 Body movement and distance bounds

For pure A3, World, Link1, Link2 and Column are stationary. Boom rotates about C. A4Carrier, A5Carrier and CArm move with the rotated imaging offset and their own extents. For angular change Δθ from an interval midpoint:

```text
boom_bound <= (norm(local_center)+norm(half_size))*abs(Δθ)
downstream_bound <= (d+norm(local_center)+norm(half_size))*abs(Δθ)
```

These arc-length bounds enclose all corners, including fixed nonzero A4/A5 tilts. Tighter bounds may use the projected distance of transformed corners from the A3 axis, but must enclose every pose in the interval. Existing body bounds already contain the offset term; verify them against the new nonzero-yaw trajectories instead of assuming prior compensated tests cover them.

Keep interval subdivision and pair-level broad rejection. Require a conservative distance lower bound greater than the residual gap, numerical allowance and both bodies' motion bounds. An approximate closest-feature result is not automatically a rigorous lower bound; use a documented numerical error budget or fall back to the separating-axis lower bound. Thin obstacles between clear endpoint poses remain mandatory fixtures. Solver budget exhaustion produces Unknown and stops.

### 8.3 Pair coverage and assembly limitations

The current generated policy excludes entire base/link, link/link, column/boom and boom/carrier/C-arm pairs. Arbitrary A3 yaw exposes parts of these bodies to each other outside the intended bearing region. Audit every excluded pair over the requested A3 range and nonzero A4/A5, especially boom–C-arm and carrier–C-arm. Do not approve unrestricted yaw by copying the old blanket policy.

Separate bearing-contact primitives from structural solids and define local intended-contact regions, or restrict the supported joint domain to one with justified interface coverage. Include source, detector, arc, carrier, upper boom, column and fixed table/patient/floor in all relevant checks. A3's stator/rotor classification must follow physical ownership.

The current `A5Carrier` frame has no separate occupied geometry, and the synthetic remote-centre carrier was not demonstrated to remain connected over arbitrary tilt. Existing passing tests do not resolve that assembly limitation. Record the domain actually supported by the proxy; if new A3 travel exposes a missing connection or unmodeled occupied region, correct the model or return unsupported rather than certify omitted geometry.

No new patient/table meshes are inherently required for this feature. Model data changes are required for control/profile metadata, frame/policy validation and any carrier/contact corrections discovered by the sweep audit. Generate constants as well as bodies so hardcoded C++ offsets cannot drift from browser parameters. Compare compiled model identity with served assets before enabling motion; browser-to-manifest matching alone is insufficient.

## 9. Implementation work packages

| Order | Files/components | Deliverable and exit condition |
| --- | --- | --- |
| 1 | `includes/commonDrive.h`, kinematic docs | Explicit heading/command semantics; all aggregate initializers and validation sites audited |
| 2 | `drive/include/cDriveCalculator.h`, `drive/src/cDriveCalculator.cpp` | Joint-jog proposal, full imaging FK, retained-heading IK, independent A3 speed profile and full-path envelope/limit checks |
| 3 | Parameters, generator, `cBodyKinematics`, `cSceneRegistry` | Generated transform/profile authority, arbitrary-yaw parity and reviewed contact policy; unsupported assembly ranges identified |
| 4 | Collision types and `cTrajectoryPredictor` | Motion-kind dispatch, A3 sweep, heading-aware other paths, complete stop and timestamp coverage |
| 5 | `cDriveController`, supervisor interface/worker | Finite A3 permissions, single owner, correct latch, bounded deadlines and retained state on Stop |
| 6 | `PCANTypes.h`, `cCANDriveHandler`, `cPCANController`, `cCANMocker` | Ten explicit directions, input version/capability handling, unchanged coherent five-angle feedback |
| 7 | `ui/index.html`, viewer template | Accessible left/right buttons, pivot and heading display, coherent/fresh feedback gating, lifecycle integration |
| 8 | Tests and existing architecture/implementation/verification docs | Fresh executable acceptance with actual A3 motion; status records only demonstrated results |

Implement the shared state and prediction contracts before exposing enabled UI buttons. Geometry generation and static previews can proceed independently of the protocol work once frame semantics are fixed. Publish model, executable and UI together while stopped; do not hot-replace geometry during an active permit.

## 10. Acceptance tests and completion gate

| ID | Scenario | Required evidence |
| --- | --- | --- |
| A3-01 | Decode every direction | 1–8 preserve mappings; 9/10 produce A3−/+; invalid version/DLC/enum creates no movement |
| A3-02 | Pure A3 at an interior clear pose | A1/A2/A4/A5 fixed; A3 changes with requested sign and enforced profile |
| A3-03 | Nonzero yaw FK/IK | Imaging X/Y and heading agree with accepted axles; zero-yaw regression preserved |
| A3-04 | Stop then X/Y | No snap to zero; heading retained and translation remains in patient axes |
| A3-05 | Stop then A4/A5 | A3 retained; existing local tilt order respected; imaging centre stays fixed for those tilt commands |
| A3-06 | Offset orbit | ±10° diagnostic FK matches the documented 1.15 m orbit; no physical motion implied by this fixture |
| A3-07 | Home/default envelope | Both pure A3 directions denied with workspace reason, without numeric creep; interior permitted fixtures demonstrated |
| A3-08 | Joint and workspace stopping limits | Full stop stays within bounds; invalid clipped stop not Clear; reverse requires Stop/new Start |
| A3-09 | Mid-arc obstacle | Clear endpoints with an obstructed intermediate pose denied in both signs and at nonzero tilt |
| A3-10 | Surface and self motion bounds | Corner motion enclosed over sampled intervals plus analytic bound review; column stationary during A3-only motion |
| A3-11 | Interface exclusions | Structural collision outside an intended bearing region detected; no undocumented blanket yaw exclusions |
| A3-12 | 9/10/11 mm fixtures | Residual gap and numerical allowance applied once; equality denied |
| A3-13 | UI lifecycle | Pointer/keyboard hold, release, cancellation, focus loss and conflicting press generate correct session behavior |
| A3-14 | Feedback | Dropped/partial/reordered/stale samples never animate a new pose; heading derived from same coherent sample |
| A3-15 | Worker delay and lost input | Expiry anchored to request time; independent stop and latch even with no further callbacks |
| A3-16 | Actual `pcan_demo` motion | Interior setup through supervised commands, A3 motion over CAN, predictive stop with reported limiting pair and full 3D gap check |
| A3-17 | Capability/model mismatch | Old client/server or stale compiled/served geometry inhibits A3; compatible Stop path remains available |
| A3-18 | Runtime visual check | Single index canvas shows signed A3 change, moving imaging centre and fixed pivot/column in top and oblique views |

Runtime tests must move to an interior fixture through supported supervised commands or launch with a validated, test-only initial fixture. Do not bypass collision with a hidden UI teleport. If that fixture is unreachable under the supported geometry, the run must report the limitation rather than silently weaken the model.

Run CMake build/CTest, generated-data checks and reference tests per AGENTS.md, or the documented direct C++17 fallback. Extend `tests/test_kinematics.cpp`, `tests/test_collision.cpp`, `tests/test_collision_reference.py` and `tests/test_runtime.py` with the relevant acceptance cases. Any changed concurrency needs appropriate sanitizer checks. Record executable/model revisions, tested poses, limiting pairs, minimum observed gap, worker latency, expiry events and shutdown result. A source-text assertion that the buttons exist is not runtime A3 acceptance.

Completion requires the existing architecture sections 17.5 and 18.12 plus the A3-specific gate above. Historical revision-5 tests remain regression evidence only.

## 11. Implemented result and evidence

The simulation runtime now implements the core contract in this document:

- `drivePosition.Yaw` retains `A1+A2+A3`; imaging-centre FK includes the 1.15 m carrier offset.
- Pure A3 commands hold A1/A2/A4/A5, use a 10 deg/s and 20 deg/s2 profile, and are refused at exact home by the existing X>=0 envelope.
- Subsequent X/Y IK shifts the carrier pivot target and adjusts A3 to retain the selected heading.
- Collision prediction has a dedicated joint-space A3 path, 5 degree worst-case A3 stop horizon, complete-stop workspace/joint validation, and heading-aware X/Y reconstruction.
- Input protocol revision 4 decodes ten explicit directions; coherent A1-A5 feedback remains unchanged.
- The integrated page provides A3 left/right hold buttons, an A3 pivot marker, heading readout and freshness-gated pose updates.

Fresh host evidence on 2026-09-17:

| Evidence | Result |
| --- | --- |
| Direct C++17 kinematics build/run | 28/28 scenarios, 65/65 checks |
| Direct C++17 collision build/run | Passed, including ten-direction decode, home denial and interior A3 preflight |
| Real `pcan_demo` runtime | Passed A3 UI hold, fixed A1/A2, retained-heading X move, opposite A3 restore, predictive 3D stop/latch and clean shutdown |
| Observed stop after A3 workflow | 0.0836 m current-pose gap, `carm_sector_05 / table_top` |

The generated geometry did not change because the revision-5 frames already consume arbitrary A3 and the feature changes motion/state/protocol behavior. The broad pair exclusions and synthetic remote-centre construction remain provisional. A3-09 through A3-12, the complete exclusion audit, browser accessibility/keyboard fault injection, sanitizer reruns and physical calibration/braking evidence remain open; they are not implied by the passing simulation workflow.
