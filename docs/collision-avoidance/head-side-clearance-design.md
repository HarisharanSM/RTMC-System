# Head-side C-arm and predictive-clearance redesign

Proposed architecture revision 4 — 2026-09-15.

Status: **implemented in the simulation on 2026-09-15; physical validation remains open.** Source baseline inspected at commit `8e35116b6a27af15870e99f6f8aa9c5544683c15`. This addendum extends [the existing architecture](architecture.md), particularly sections 17 and 18. Verification evidence and remaining limitations are recorded in the implementation and verification documents.

## 1. Requested outcome and decisions

| Observation | Proposed solution | Important qualification |
| --- | --- | --- |
| CRAN/CAUD travel should be limited to 90° | A5 range becomes −90° through +90°, with predictive stopping at both boundaries | Interpreted as 90° in each direction, not 90° total travel; collision can require stopping earlier |
| Avoidance is too sensitive; use one unit/step gap | Set nominal residual surface clearance to 0.010 m, with explicit numerical tolerance and separately modelled uncertainty | Assumes one unit means 1 cm, the full-speed linear increment in 50 ms; an angular step is not a distance |
| LAO/RAO appears blocked everywhere | Correct geometry and axis conventions, diagnose rejected body pairs, use tighter 3D distance checks and enforceable speed-bounded prediction | Existing collision is already 3D; never authorize a genuinely intersecting pose to improve movement |
| C-arm is on the patient's side | Turn the neutral rear C/support construction toward −X, the headward direction, without moving the fixed patient/table or changing A1/A2 geometry | Preserve A3 compensation and the patient-head X/Y origin |

The 10 mm interpretation and the signed LAO/CRAN axis convention below are explicit design assumptions to confirm before implementation. They are not manufacturer specifications or clinically validated clearances. All geometry remains an original synthetic simulation proxy, not Siemens/Artis pheno CAD. No physical collision-protection claim follows from this design.

## 2. Evidence and root-cause analysis

### 2.1 Travel limits

[`cDriveCalculator.h`](../../drive/include/cDriveCalculator.h) currently defines A5 limits as ±180°. [`cDriveCalculator.cpp`](../../drive/src/cDriveCalculator.cpp) maps CRAN directly to A5 and uses those limits in clamping/validation. A UI-only limit would leave the drive and prediction contract inconsistent.

Required change: use one authoritative ±90° A5 range throughout drive proposals, IK validation, prediction, asset metadata, UI labels and tests. Keep A1/A2/A3 and the existing A4 range unchanged unless a separately approved mechanical requirement changes them.

### 2.2 Minimum gap is not the dominant angular stopping horizon

[`PredictionSettings`](../../collision/include/cCollisionTypes.h) currently specifies a 20 mm pair margin, 250 ms reaction allowance, 150 ms permit lifetime and 0.5 mm interval-motion tolerance. [`SubmitCollisionRequest`](../../drive/src/cDriveController.cpp) supplies maximum speeds and marks velocity as unmeasured. [`StoppingTravel`](../../collision/src/cTrajectoryPredictor.cpp) therefore always begins with maximum speed and then adds acceleration without a speed cap:

```text
travel = v*T + 0.5*a*T² + (v + a*T)² / (2*b)
```

| Quantity | Current configured value / calculated consequence |
| --- | --- |
| Linear command speed; integration interval | 0.20 m/s; 0.050 s |
| Nominal full-speed linear increment | 0.010 m = 1 cm |
| Linear prediction: v=0.20, a=0.40, b=0.50, T=0.250 in SI | 0.1525 m = 152.5 mm |
| Angular command speed; nominal full-speed increment | 60°/s; 3° |
| Angular prediction: v=60, a=b=120, T=0.250 in degree units | 52.5° total: 18.75° reaction travel plus 33.75° braking travel |
| Angular speed assumed at brake initiation | 90°/s, above the nominal 60°/s command limit |

These are calculations from inspected source, not measured stopping performance. The conservative envelope can reject a small initial rotation because a much later pose intersects the table/patient. Reducing only the margin from 20 to 10 mm cannot remove that effect. Even substituting zero initial angular speed in the current uncapped formula still predicts 7.5° when reaction acceleration is retained.

Startup and near-fold linear increments can be smaller than 1 cm because of the joint profile/rate limits. At the 0.84 m nominal arc radius, a 3° rotation moves a point about 44 mm along the endpoint chord. Thus “one actual step” is neither a fixed gap nor dimensionally interchangeable between translation and rotation.

### 2.3 Existing collision is three-dimensional, but conservative

[`cProximityBackend.cpp`](../../collision/src/cProximityBackend.cpp) uses the 15 separating-axis tests for oriented boxes. [`cBodyKinematics.cpp`](../../collision/src/cBodyKinematics.cpp) transforms boxes in 3D, including LAO and CRAN. The predictor recursively checks motion intervals, not just X/Y endpoint overlap.

The reported restriction is therefore not evidence of a missing Z dimension. Relevant contributors are:

- A worst-case angular stopping sweep much larger than the next commanded step.
- Coarse box proxies around curved geometry and square patient envelopes.
- A largest separating-axis gap that is a conservative lower bound, not the exact Euclidean closest-surface distance.
- Conservative motion bounds and an `Unknown` result when an interval cannot be certified clear; Unknown correctly denies permission.
- Actual support/arc placement and potentially mismatched anatomical tilt axes.
- Other causes requiring runtime discrimination: joint limits, a previously latched stop, stale permits, or transport faults.

The statement “restricted at all positions” has **not** been reproduced with a workspace sweep in this analysis. A rejection map with pair IDs, pose, direction and reason is a mandatory implementation baseline, before and after changes. Do not assume every rejection is a false positive.

### 2.4 Side placement is in the model, not just the camera

In [`cSceneRegistry.cpp`](../../collision/src/cSceneRegistry.cpp), the arc is constructed in local YZ with its rear half at negative Y. The support beam is centred at `(0, −0.45, 0)` and its column at `(0, −0.90, 0.26)` in the support frame. At neutral A4/A5 and compensated yaw these remain transverse to the patient.

The patient reference instead defines +X toward the feet and −X toward the head. The current neutral structure therefore occupies the patient's side. Changing the camera cannot fix its collision placement.

Current C-arm orientation is `Rz(A1+A2+A3) * Ry(A4) * Rx(A5)`. With compensated yaw, A4 initially tilts about patient Y and A5 about the subsequent X axis. The proposed longitudinal LAO/transverse CRAN convention requires an explicit transform revision, not merely swapping button labels.

### 2.5 Visible geometry and collision geometry need stronger agreement

[`generate_collision_reference.py`](../../tools/generate_collision_reference.py) includes `robot_base`, whereas the compiled reference collision scene omits that body. Arc IDs are zero-padded in generated assets but not in C++. Generated pair policy lists no adjacent exclusions, whereas the predictor excludes several whole rigid-body group pairs.

These are independent representations, so a correct browser asset hash alone does not establish collision-model agreement. Reconcile geometry and pair policy before trusting an apparent visual clearance. Adding the base must include a review of intended base/link and floor contacts; blindly enabling all such pairs may make home invalid.

## 3. Coordinate and five-axis architecture

### 3.1 Invariants preserved

- World/scene units: metres, radians internally, Z up. Legacy drive adapter: centimetres and degrees.
- Initial patient head centre: `H=(0,0,1.20)` m. X/Y remain offsets from this fixed reference, not from the moving camera or C-arm.
- Imaging centre: `I=(X/100,Y/100,1.20)` m. At X=Y=0, I=H; rotations leave I fixed.
- A1/A2 keep 0.75/1.00 m links, base `(−0.25,0)` m and relative A2 convention.
- A3 remains `−(A1+A2)`; do not hide the mount correction in a 90° A3 offset.
- A4 remains the LAO/RAO numeric field; A5 remains CRAN/CAUD. CAN target IDs 0x201–0x205 retain their axis identities.

### 3.2 Proposed head-side transforms

Use active right-handed rotations on column vectors; products apply rightmost first. Let `yaw=A1+A2+A3` and introduce a **fixed** mount transform `Rmount=Rz(−π/2)`. This maps the current local negative-Y rear direction to patient negative-X. A positive 90° mount rotation would put it toward the feet and is incorrect.

Proposed frame equations, retaining the original primitive-local arc construction:

```text
T_support = Translate(I.x, I.y, 0.68) * Rz(yaw) * Rmount
T_carm    = Translate(I) * Rz(yaw) * Rx(A4) * Ry(A5)
T_body    = T_frame * Translate(localCentre) * R_localBody
```

At neutral, the imaging arc remains in its transverse YZ plane around the patient while its rear support beam and column extend toward −X. LAO tilts the arc about patient longitudinal X; CRAN tilts about the Y axis of the already LAO-rotated frame. This is an ordered gimbal, not two commuting world rotations. Positive angles use the right-hand convention. Confirm the desired clinical button signs using supine head/feet/left/right landmarks before freezing the mapping. If signs require inversion, define that once at the named-axis adapter and test every consumer; do not introduce independent UI and collision sign flips.

The support follows yaw/translation and the fixed mount but not A4/A5. The source, detector and arc follow the C-arm transform and remain centred on the imaging point. Rotate body orientations as well as centres. Do not rotate the fixed table/patient, and do not apply the mount rotation twice by also baking it into vertices.

Neutral support centres become `(I.x−0.375,I.y,0.28)` for the beam and `(I.x−0.75,I.y,0.74)` for the column. The planar links are lowered to Z centres 0.10/0.28 m and the synthetic rear reach is 0.75 m so the head-side support is constructible without suppressing non-adjacent self-collision at folded home. The neutral arc remains in YZ around I. The source/detector centre Z offsets remain −0.62/+0.60 m.

“Head side” here means rearward toward the head relative to the moving imaging centre at neutral tilt. At positive X travel the assembly translates with I; it is not guaranteed to stay globally behind X=0. Maintaining that global condition over the whole workspace would require a different mechanical layout or additional motion constraints. Tilt also changes the occupied region, which must be checked rather than constrained visually.

### 3.3 A5 range and stopping

Set A5/CRAN to `[−90°, +90°]`. Validate the entire permitted trajectory, including braking, against these limits. Merely truncating the collision sweep at the last reachable pose does not prove that the drive can stop there.

Near a boundary, choose a slower certified trajectory or stop before it. At a stationary boundary, reject outward movement; allow a new inward request after the required controller Stop/rearm and a clear preflight. No epsilon creep, wraparound or silent coordinate snap is allowed. Loading an old pose outside the new range must inhibit normal motion and report a configuration/state incompatibility; do not silently reinterpret it as ±90°.

## 4. Shared 3D data architecture

Use one parameter/model source to generate both browser geometry and a typed C++ immutable collision scene. Keep [`parameters.json`](../../data/collision/reference/parameters.json) and the generator as authoring inputs; never hand-edit `generated/` outputs. Extend their schema only during implementation.

| Data group | Required contract |
| --- | --- |
| Frames and axes | World, head reference, A1/A2 links, compensated support, ordered A4/A5 gimbal, fixed mount; explicit units and multiplication order |
| Each body | Stable ID, rigid group, parent frame, shape type, centre, full local orientation, dimensions/vertices, collision participation and provenance |
| Pair policy | Explicit checked pairs and narrowly documented designed-contact exclusions; same policy for runtime diagnostics and engineering viewer |
| Motion policy | Axis ranges, allowed speed/acceleration envelopes, assumed braking, reaction allowance, residual gap and numerical tolerances |
| Identity | Schema version, geometry/frame/pair-policy hash, motion-policy revision and generation; mismatches invalidate permits and inhibit Start |
| Provenance | `simulation_only`, synthetic/unmeasured dimension status, unknown calibration/braking evidence retained |

Retain existing dimensions initially to isolate the mount/algorithm changes: arc inner/outer radii 0.70/0.84 m, 24 sectors, width 0.22 m; current source, detector, table and patient sizes. Do not enlarge the opening or shrink patient/table volumes merely to obtain a passing demo.

Use conservative convex sector geometry for the curved arc in the refined narrow phase. Its construction must enclose the intended solid, including angular endpoints and the inner/outer boundaries; an inscribed polygon can under-represent the outer surface. Compare containment against analytic sector samples and retain an explicit approximation bound. Keep simpler boxes for box-shaped housings. Render and collision geometry may differ in tessellation, but the collision solid must conservatively contain the declared visible solid within the documented tolerance.

Include the robot base and every relevant moving/stationary component. Designed contacts at joints and floor mounts need explicit local contact geometry or split primitives; avoid excluding a whole articulated assembly if non-contact portions can collide. Patient, mattress and table must not receive blanket exclusions. Existing whole-group exclusions require review, not automatic carry-over.

Future dynamic obstacles use the same body/shape schema plus timestamped world transforms and bounded motion/uncertainty. Keep static geometry cached; publish immutable dynamic snapshots. Missing/stale dynamic data returns Unknown. Dynamic sensing is not implemented or required by this revision.

## 5. Gap and predictive avoidance algorithm

### 5.1 Residual surface clearance

Proposed nominal policy: `minimumSurfaceGapM=0.010`. It applies to each active body pair, including robot/environment and non-excluded self-collision. It is a required remaining separation, not a timestep, origin-to-origin distance or replacement for braking distance.

```text
requiredGap = 0.010 m + pairUncertaintyM + numericalAllowanceM
clear only if certified minimum surface distance over the complete
continued-motion-and-stop trajectory is strictly greater than requiredGap
```

Simulation fixtures may explicitly use zero model uncertainty; that does not assert zero physical uncertainty. Unknown physical calibration cannot be converted to a validated zero. Keep the existing 0.5 mm interval-motion resolution distinct from geometric uncertainty. Numerical error must be budgeted in the certificate; refinement exhaustion denies permission. Do not subtract tolerances to make a marginal case clear.

Equality at the required gap denies further approach conservatively. A static aligned-box fixture at 9 mm must deny; at 10 mm must deny equality; at 11 mm must clear when its configured total uncertainty/numerical allowance is less than 1 mm and no movement is requested. Runtime stopping generally occurs farther away because it must leave this gap after the remaining motion.

### 5.2 Hybrid 3D proximity pipeline

1. Validate model generation, state freshness, numeric values, axis limits and the requested single direction.
2. Generate a bounded proposed continuation and braking trajectory from an immutable drive snapshot.
3. Broad phase: conservative swept AABBs over each time interval, inflated by the required gap. Include all moving body corners and both bodies' motion when applicable. Cache stationary bounds.
4. Narrow phase: retain OBB SAT overlap/cheap separation tests; for unresolved non-overlapping candidates, use a bounded-iteration convex-distance solver returning closest points and a conservative distance lower bound. A GJK-distance implementation is a candidate, with degeneracy and numerical tests required before acceptance.
5. Continuous certification: at an interval midpoint require `distanceLowerBound > requiredGap + movementBoundA + movementBoundB`. Otherwise refine the interval or establish a hazard. Endpoint-only sampling cannot authorize the sweep.
6. On convergence failure, invalid geometry, exhausted refinement or missed deadline, return Unknown and deny/revoke permission. Never treat non-convergence as a clear result.

Preserve analytic full-interval joint-excursion bounds for Cartesian paths, including intermediate joint reversals near the fold. Re-derive body motion bounds for the revised frame chain and convex support radius. A frame correction without a corresponding sweep-bound review is incomplete.

Distinguish diagnostic outcomes: current overlap, predicted overlap, insufficient residual gap, unproven interval, joint stopping limit, model mismatch and stale state. Report the limiting pair and witness locations where available. Do not label a coarse lower bound as an exact measured gap.

### 5.3 Less conservative prediction requires an enforceable drive contract

Do not change `velocityMeasured` to true for commanded simulation feedback. Introduce explicit state quality such as `SimulatedBounded`, `MeasuredValidated` and `Unknown`. Only the first is available from this simulator without new hardware evidence.

The single drive owner must publish signed per-axis velocity/state bounds, pose sequence, snapshot timestamp, active profile and enforceable speed/acceleration limits. Prediction must cover state age, permitted continuation until revocation/expiry, dispatch latency and braking. Unknown velocity retains a conservative bound and can legitimately prevent near-obstacle startup.

For a scalar illustrative motion with known `0 ≤ v0 ≤ vmax`, acceleration `a`, reaction time T and braking b, integrate the capped reaction profile:

```text
t_accel = min(T, max(0, (vmax-v0)/a))   [a=0 handled separately]
reactionTravel = v0*t_accel + 0.5*a*t_accel² + vmax*(T-t_accel)
v_brake = min(vmax, v0+a*T)
totalTravel = reactionTravel + v_brake²/(2*b)
```

This is an explanatory one-dimensional bound, not a substitute for the five-axis body sweep. If initial speed exceeds a newly requested cap, explicitly model deceleration from the actual bounded speed; never clamp that initial speed down in the predictor. Signed reversals and profile transitions also need their own bounded trajectories.

At a validated 60°/s cap the full-speed illustrative bound becomes 30°, not 52.5°. A certified 5°/s cap gives about 1.354° using T=0.250 s and b=120°/s². Both still require actual full-body clearance. The cap is only usable once the drive enforces it, including acceleration and discrete tick/command behaviour.

At preflight, evaluate a bounded descending set of speed profiles in the requested direction, for example 60, 30, 15, 5 and 1°/s for angular motion. Issue only a profile with a complete clearance certificate; otherwise remain stopped. Do not assume arbitrary trajectory feasibility is monotonic enough for an unproved speed bisection. Translational candidates must also respect A1/A2/A3 rates near singularities.

This introduces explicitly scoped speed selection beyond revision 3's stop-only policy; it does not steer, reverse, or change direction. During motion, a lower cap can replace the active permit only if the transition and latest possible stop are certified. If not, protective-stop and latch. A predicted hazard must never be downgraded to an informal slow-down command without that certificate.

### 5.4 Finite permission fields and scheduling

Extend the permission contract with session, request sequence, source pose sequence/time, scene/policy generation, direction, trajectory/profile identity, enforceable per-axis speed/acceleration bounds, covered state region, latest start time and expiry. Renewal must not permit old certificates to be applied to newer, uncovered drive states.

Drive work remains bounded: read an immutable permit; verify identities, time and proposal containment; commit only a covered segment. Geometry and solver work stay on the collision worker. No concurrent mutation of `cDriveCalculator` or the drive profile is permitted.

Retain the independent 1 ms stop monitor, 150 ms renewal expiry and 250 ms reaction allowance until timing evidence supports a separately reviewed change. Timestamp certificates against the covered snapshot/trajectory timeline, not merely solver completion; solver delay cannot extend the coverage. The timing budget must include snapshot age, queued work, renewal expiry, dispatch and stop actuation. If the sum cannot fit the modelled allowance, deny/revoke rather than shortening prediction. Record scheduling overruns; a simulator host does not provide hard real-time physical guarantees.

## 6. Controller, CAN and integrated UI behaviour

The established lifecycle remains authoritative:

```text
index joystick press → CAN → controller → collision preflight
  CLEAR with finite profile → drive may start
held input → CAN → controller → drive, subject to matching permission
drive committed pose → coherent CAN feedback → /state → index C-arm canvas
prediction hazard/unknown/expiry → drive stop → latched until controller Stop
release/cancel → CAN → controller Stop to both collision and drive
standstill + required acknowledgements → new session may request preflight
```

The renderer remains inside `ui/index.html`; no separate live viewer or iframe. Apply the same revised frame chain to coherently received A1–A5 feedback on IDs 0x301–0x307. Do not animate from the joystick request or extrapolate movement while feedback is stale. This remains simulated commanded feedback, not encoder measurement.

Add a compact operator status showing requested direction, accepted motion state, angular limits, selected speed, predicted stopping restriction and limiting body names. Separate “minimum configured gap: 10 mm” from current/predicted clearance and label bounds as bounds. Engineering overlays may show axes, collision solids and witness lines without altering motion authority.

Do not repurpose existing actuator/feedback fields to carry permit caps. Version any new CAN/status payload explicitly, publish codecs and reject incompatible peers. If the simulator retains in-process permission delivery, document that boundary and trace it; do not claim new hardware CAN messages exist until implemented and tested. Release, duplicate Start, late Clear and a continuously held button must preserve the current stop-latch behaviour.

## 7. Implementation plan and affected components

The work packages below define the implementation order used for this revision. Hardware calibration, measured braking and released protection remain outside the simulation completion claim.

| Order | Work package and source areas | Exit criterion |
| --- | --- | --- |
| 1 | Establish reproducible baseline using `tests/test_collision.cpp`, `tests/test_runtime.py`, current executable/assets and pair diagnostics | Record actual LAO/RAO denials by pose/direction, not just the UI symptom; distinguish current overlap and predicted restriction |
| 2 | Confirm units/signs; update model schema, generator, `parameters.json`, data specification and generated typed scene path | One geometry/frame/policy authority; base and intended contacts reviewed; explicit 10 mm policy and A5 ±90° |
| 3 | Update `cDriveCalculator.h/.cpp`, kinematics docs and tests; implement head-side transforms in `cBodyKinematics`, generator, offline viewer template and index renderer | Exact X/Y/head pivot and A3 invariants; identical world corners in renderer and collision; no changes to planar lengths/envelope |
| 4 | Refine `cSceneRegistry`, `cCollisionTypes`, `cProximityBackend` and pair masks | Valid convex bounds, real distance certificates, no accidental omitted obstacles or blanket patient exclusions |
| 5 | Update single-owner drive snapshots/proposals, predictor, permit/service/controller contracts and protocol codecs where required | Capped profiles are actually enforced; state age, braking, joint limits, renewal and profile transitions are covered |
| 6 | Wire runtime status/overlays in `CANMocker`, `SystemController`, `PCAN` and `ui/index.html` | Integrated display uses drive CAN only; rejection reasons visible; Stop/rearm ordering unchanged |
| 7 | Run geometry, trajectory, concurrency, runtime and browser acceptance; regenerate reference assets | All gates below and architecture 17.5/18.12 pass against the same `pcan_demo` build; record evidence in `verification.md` |

Phases 3–5 must be integrated before claiming improved near-obstacle movement. A margin-only patch is not completion. Keep `implementation.md` and existing kinematics documents as current/as-built records until their corresponding code is changed, then update them with verified behaviour and remaining limitations.

## 8. Verification and completion gates

| ID | Required test | Acceptance evidence |
| --- | --- | --- |
| HC-01 | A5 boundaries at −90, +90 and just beyond; both hold directions and inward restart | No accepted/feedback angle outside bounds, no creep/wrap, inward motion after Stop and fresh permission; no silent snap of invalid loaded state |
| HC-02 | Neutral home and representative X/Y positions | I equals the FK position with head origin; rear support lies on negative-X side of I; A3 compensation and planar IK/FK remain unchanged |
| HC-03 | Positive/negative A4/A5 and combined tilts | Pivot invariant; explicitly approved beam/sign convention; renderer and collision corner transforms agree within declared numerical tolerance |
| HC-04 | Static box surface gaps 9/10/11 mm, with a documented sub-1 mm allowance | Reject 9 mm and equality at 10 mm; accept clear 11 mm static fixture; repeat rotated, edge/vertex and degenerate configurations |
| HC-05 | Pure angular sweep with clear endpoints but obstructed middle | Predictive denial; no tunnelling through table, head, support or thin test obstacle |
| HC-06 | Old and revised angular horizon arithmetic; low-speed and reversal trajectories | Caps enforced by every committed segment; no false measured-velocity flag; current speed above a new cap handled conservatively |
| HC-07 | Joint-limit stopping and scene collisions during braking | Complete bounded trajectory stays within A5 range and required pair clearance; reachable-fraction clipping alone cannot pass |
| HC-08 | Model parity and corruption | Compare all body IDs, shapes, world transforms, pair masks and policy hashes; base accounted for; mismatch inhibits Start |
| HC-09 | Workspace rotation map | Evaluate reachable X=0:10:150 cm, Y=−25:5:25 cm, A4=−180:15:180°, A5=−90:15:90°; classify infeasible/current-overlap/clear/future-hazard/unknown; refine transitions and test both requested signs |
| HC-10 | Selected clear rotation fixtures in actual runtime | Press/hold LAO and RAO at home and verified clear mid-workspace fixtures; accepted angle advances and CAN-fed canvas matches; documented real obstructions still deny |
| HC-11 | Approach known obstacle at multiple certified speeds in `pcan_demo` | Record source/pose sequences, trajectory, minimum gap through stop, latencies, selected caps and limiting pair; residual clearance remains above required bound |
| HC-12 | Worker stall, stale snapshot/feedback, lost/reordered CAN, model change and late permit | Independent stop deadlines preserved; no motion authorized by stale state or wrong generation |
| HC-13 | Predicted stop while button held; release/new Start | No restart while latched; controller Stop reaches both recipients, required acknowledgements precede fresh preflight |
| HC-14 | Geometry approximation and performance | Convex proxies enclose intended solids; bounds include all intermediate motion; measured worst observed/p99 latency and queue age fit the declared simulation budget; timeout returns Unknown |

The workspace map is a diagnostic sampling grid, not a continuous workspace safety proof. HC-10 fixtures must be proven clear with the corrected model before asserting movement should be allowed. If home or a desired operational pose is genuinely obstructed, report the intersecting components and request a mechanical/model decision; do not remove the obstacle. ±90° is an actuator range, not a guarantee of collision-free CRAN/CAUD travel at every X/Y/A4.

During implementation validation, stop the interactive demo before runtime tests because the suite owns port 8082. Run:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
python3 tools/generate_collision_reference.py --check
python3 tests/test_collision_reference.py
```

`runtime_avoidance`/`tests/test_runtime.py` must exercise the actual freshly built executable. Add the above runtime cases to that gate, not only to library tests. Also inspect the served index canvas for head-side placement, signed tilts and drive-feedback-only animation. Record executable revision, geometry/policy hashes, toolchain, fixtures and failures in `verification.md`; static viewer success is insufficient. Relevant concurrency checks remain required by architecture section 18.12.

## 9. Remaining validation decisions and non-goals

1. The simulation now interprets “one unit” as 1 cm and 90° as ±90°. Changing either interpretation requires a new policy/model revision and regeneration.
2. Confirm the implemented right-hand anatomical signs on the physical mechanism and whether head-side means neutral-relative-to-I, as implemented, or globally behind the head throughout travel.
3. Review synthetic support connectivity and intended joint contacts after reorientation. This design does not establish a mechanically manufacturable mounting arrangement.
4. Keep braking/calibration/physical feedback unknown for hardware. Simulator timing and 10 mm proxy clearance are not clinical release criteria.
5. No physical motion, automatic escape from existing overlap, path planning, patient sensing or released dynamic-obstacle protection is authorized or delivered by this simulation implementation.

The software implementation and runtime verification are recorded in `implementation.md` and `verification.md`. Those results establish simulator behavior only; the physical release gates remain open.
