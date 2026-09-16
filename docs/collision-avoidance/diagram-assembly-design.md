# Drawing-based C-arm assembly and collision avoidance

Design and implementation specification — proposed revision 5.

Document date: 2026-09-16.

Status: implemented as a synthetic simulation model on 2026-09-16. Source baseline inspected: `927f8153369a2e7bb204955a5d3d742b717702f8`. Read with [the main architecture](architecture.md), [current implementation status](implementation.md) and [kinematic conventions](../kinematics-model.md). Dimensions and the remote-centre carrier remain declared provisional inputs, so this is not physical-machine release evidence.

## 1. Objective and interpretation of the drawings

Reconstruct the assembly shown in the user's side, back and top views as one connected mechanism with consistent motion and collision geometry. Retain patient-head coordinates, five actuator identities, CRAN/CAUD ±90°, a 10 mm nominal residual gap, and the established controller/CAN/drive stop lifecycle.

The sketches establish arrangement and connectivity. They contain no dimension lines, scale, joint-axis arrows or detailed bearings. Pixel lengths, apparent overlap and drawing stroke width must not be treated as measurements or permission to exclude collision pairs. The selection rectangle/icon in the top image is a drawing-editor artifact and is excluded from the mechanical interpretation.

| Reference | Visible arrangement | Design interpretation | Not established by the image |
| --- | --- | --- | --- |
| Image 1: Side | Low A1/A2 structure; tall column; A3 near column top; upper horizontal boom; A4/A5 at rear of C; upper/lower end housings; fixed table extending away | Column rises from the second-link endpoint; upper boom reaches toward the table; neutral C is in the longitudinal vertical plane, with its opening toward the table | Exact heights, boom reach, source/detector identity, physical A4/A5 axes or remote-centre mechanism |
| Image 2: Back | Folded two-link structure from A1 through A2 to the column/A3 assembly | Links are distinct moving bodies; column belongs downstream of link 2 | Whether the projection is orthographic; link elevation changes or additional pitch joints |
| Image 3: Top | A1 base, offset A2 elbow, column/A3 location and narrow boom toward the table | A1/A2 articulate in the horizontal plane; A3 aligns the upper boom with the patient reference | Actual link contours, bend radii, dimensions, or exact home angles |

Use the existing interpretation of A1/A2/A3 as parallel Z axes unless a subsequent axis drawing changes it. The back sketch does not justify introducing extra actuators. Treat the existing upper detector/lower source identity as a simulator assumption because the sketches do not label those components.

## 2. Difference from the current implementation

The current [body kinematics](../../collision/src/cBodyKinematics.cpp) places the imaging centre directly above the planar link endpoint. Its rear support is offset backward from that centre. The arc remains in YZ, so its rear occupies the patient's transverse side. This is not the side-view arrangement supplied here.

| Current implementation | Required revision-5 arrangement |
| --- | --- |
| Planar link endpoint and imaging centre share X/Y | Distinct link endpoint/column, A3, rear attachment and imaging-centre frames |
| Rear column displaced from the link endpoint | Column starts at the physical end of link 2 |
| Low rear support beam | Upper boom runs from A3 at column top to A4/A5 mounting hardware |
| Neutral arc in YZ | Neutral arc in XZ, rear headward and opening toward +X/table |
| Simplified arc rotates without explicit gimbal housings | Separate bearing, carrier and arc bodies, with a stated remote-centre or compensated-pivot model |
| Independently maintained C++/generated scene | One typed model source for geometry, frames, limits and pair policy |

Existing parity gaps must be addressed during implementation: generated assets include a robot base absent from the compiled scene; arc IDs differ in zero padding; pair exclusions differ; and the compiled elbow centre offset remains 0.125 m while the generator derives 0.09 m from the current link heights. Successful tests of the two representations independently do not prove agreement.

The earlier change to link heights 0.10/0.28 m and rear reach 0.75 m was a synthetic workaround for the previous layout. These values are not dimensions inferred from these drawings. Do not keep adjusting heights or moving obstacles until a preferred joystick command passes; establish the new assembly first and report real interference.

## 3. Assembly and rigid-body ownership

```text
World
├── Fixed table, pedestal, base, mattress and patient fixture
└── Robot base
    └── A1 → link 1
        └── A2 → link 2 and column lower structure
            └── A3 at column top → upper boom
                └── A4 carrier / bearing
                    └── A5 carrier / orbital mechanism
                        └── C-arm arc, source and detector
```

This is the logical joint order. A4/A5 labels at a rear attachment do not establish that both mathematical rotation axes pass through that attachment. Section 5 defines the required distinction.

| Body/group | Pose owner | Geometry requirement |
| --- | --- | --- |
| Base and A1 stator | World | Separate pedestal/enclosure from any moving rotary platform |
| Link 1 and A2 stator | A1 | Compound solids following the drawn link contour; bearing interfaces explicit |
| Link 2 and column | A1+A2 | Column foot connects to link 2; column top reaches A3; A3 stationary housing belongs here |
| A3 rotor and upper boom | A1+A2+A3 | Boom projects toward the table; housing/boom connection has no unexplained gap |
| A4 carrier | A4 transform | Include arms, forks, bearings and their occupied volume |
| A5 carrier/orbital carriage | Ordered A4/A5 transform | Model actual or explicitly synthetic remote-centre linkage |
| Arc, source, detector | C-arm transform | Preserve free C opening and correct full housing extents |
| Table/patient/environment | World or later tracked snapshot | Stationary for this version; patient geometry remains a synthetic fixture |

The column lower structure is provisionally on the A3 stator side, so it follows A1/A2 yaw rather than A3 compensation. If the column itself rotates with A3, change its frame ownership explicitly in the model. This distinction affects every swept collision volume even when the imaging centre follows the same path.

## 4. Coordinate frames and planar kinematics

### 4.1 Frames and units

Use metres/radians/seconds internally; keep centimetres/degrees at the existing drive adapter. Use right-handed axes: patient +X head to feet, +Y transverse, +Z up. `H` is the initial patient-head imaging target. Retain nominal `H.z=1.20 m` only as a simulation datum.

Define:

- `K`: existing planar calculation frame, with base `B_K=(-0.25,0)` m and link lengths 0.75/1.00 m.
- `W`: patient/world frame used by collision and rendering.
- `E`: A2 elbow centre.
- `M`: end of link 2 and column foot.
- `C`: A3 origin at column top.
- `G`: boom-to-gimbal rear attachment.
- `I`: imaging centre between source and detector.

Never use `EOF` for both M and I without qualification. In public position feedback, X/Y continue to mean imaging-centre displacement from H. Internal joint/column positions are derived fields.

For the equations below, K and W have parallel axes and share the floor/Z datum; `T_W_K` is a horizontal translation. An installation with rotated axes requires an explicit rotation in the registration and corresponding conversion of patient commands and joint-frame orientations.

### 4.2 Existing solve and new tool offset

For q1/q2 in radians and lengths in metres:

```text
E_K = B_K + L1 * [cos(q1), sin(q1)]
M_K = E_K + L2 * [cos(q1+q2), sin(q1+q2)]
yaw = q1 + q2 + q3
q3 = -(q1+q2)                         [normal compensated mode]
I_W = M_W + Rz(yaw) * d_MI            [neutral/remote-centre mode]
I_target_W = H + [X_cm/100, Y_cm/100, 0]
M_target_W = I_target_W - Rz(yaw) * d_MI
```

`d_MI` is the measured or declared synthetic vector from column foot to imaging centre; it includes the column rise, upper boom and rear-to-isocentre offset. It is a model parameter, not a guessed angle-dependent correction in the UI.

Keep the existing numeric planar solve by adding an explicit fixed registration `T_W_K`. At closed home, `M_K=(0,0)`. Choose its horizontal translation so `M_W_home=H_xy-d_MI_xy` when yaw=0. The physical base then appears at `H_xy-d_MI_xy+(-0.25,0)` in W. Thus the old −25 cm base value stays in K; it must no longer be incorrectly advertised as the patient-frame base coordinate for a nonzero tool reach.

Under exact yaw cancellation and a fixed tool offset, patient X/Y displacements still equal the existing planar displacements. Accepted-pose/angle agreement and the 75/100 cm solve can be preserved. All collisions and drawing projections must use W, including the correctly registered robot base.

If an installation instead requires the physical base to remain at world `(−0.25,0)`, solve a different home pose or relocate the patient reference using an explicit installation model. Do not silently claim closed-home I=H, the same physical base coordinate and a nonzero forward tool reach simultaneously.

### 4.3 Frame-chain construction

Column-vector transforms apply rightmost first. Each physical joint has a calibrated parent-to-joint translation/orientation and a declared rotation axis. For example:

```text
T_W_link1 = T_W_K * Translate(B_K.x,B_K.y,z1) * Rz(q1)
T_W_link2 = Translate(E_W.x,E_W.y,z2) * Rz(q1+q2)
T_W_Cstator = T_W_link2 * Translate(column_foot_offset) * Translate(0,0,column_height)
T_W_Crotor = T_W_Cstator * Rz(q3)
T_W_G = T_W_Crotor * Translate(boom_to_gimbal_offset)
```

Explicitly include nonzero foot and bearing offsets. The simplified planar relationship above assumes the column centre is at the link endpoint and its vertical axis is parallel to A1/A2. Validate the general chain against that special case. Define all Z heights from the same floor datum.

In this special case, `column_foot_offset=(L2,0,0)` in the link-2 frame; any additional bearing or column-foot displacement must be separately named. The chosen column height and upper offsets must also satisfy `I_home.z=H.z`; horizontal registration cannot correct a vertical mismatch.

## 5. C-arm construction and A4/A5 motion

### 5.1 Neutral orientation matching the side view

Author the neutral arc directly in XZ, extruded in Y. With centre I and θ in `[π/2,3π/2]`, an arc point is:

```text
p_arc = [r*cos(theta), lateral_depth, r*sin(theta)]
r in [inner_radius, outer_radius]
```

The rear has negative X; the upper and lower ends lie near X=0; the opening faces positive X toward the table. Source/detector centres remain vertically opposed at neutral. If reusing existing YZ primitive coordinates, use one fixed `Rz(-π/2)` conversion. Apply it exactly once to centres and orientations, and ensure the housing cross-sections are intentionally transformed as well.

Use the same source for upper/lower housing geometry and focal/image markers. A published SID or internal face gap does not define the external enclosure. Retain existing 0.70/0.84 m arc radii and housing dimensions only as explicitly labelled starting proxies, subject to a connected-assembly review.

### 5.2 Imaging centre versus physical mounting point

The requested behavior keeps I fixed during LAO/CRAN at fixed X/Y. A rigid C-arm simply hinged at its rear G generally cannot provide that behavior. For a local vector `r_GI`, a conventional hinge produces:

```text
I(q) = G + R(q) * r_GI
```

That centre moves as q changes. Drawing an arc at `Translate(I)*R(q)` while leaving an incompatible rear mount stationary would disconnect the simulated mechanism.

Preferred interpretation for the simulator: an explicit remote-centre assembly whose effective A4/A5 axes intersect I. Define the synthetic carrier/orbital mechanism and include its collision solids. The logical arc orientation is:

```text
R_tilt = Rz(yaw) * Rx(q4) * Ry(q5)
T_W_arc = Translate(I) * R_tilt         [arc authored in XZ]
```

A4 is provisionally rotation about patient longitudinal X and A5 about the subsequent transverse Y; signs use the right-hand convention. Clinical naming/signs require axis drawings or calibration and cannot be read from the sketch. Keep A4=LAO/RAO and A5=CRAN/CAUD numeric fields.

For any moving rear point `g_arc` on the arc, its position is `I+R_tilt*g_arc`. Carrier geometry must connect the fixed boom assembly to that moving point over the allowed travel. It must have actual occupied volume; an invisible or zero-volume connector is unacceptable.

Alternative if the real assembly uses ordinary rear hinges: use the measured axis offsets and calculate compensating planar movement so I follows the commanded target. Any resulting vertical displacement requires a proven mechanical compensation path or an additional degree of freedom. A1/A2/A3 planar motion cannot cancel arbitrary Z displacement. Reject unreachable combined tilts; do not silently clamp Z or invent an actuator.

Remote-centre behavior is a provisional motion model. Detailed carrier geometry and which joint is orbital versus axial remain open mechanical inputs. Development can build parametric interfaces and unit fixtures now; final assembly acceptance requires that choice to be resolved and documented.

### 5.3 Travel limits

Retain A1 `[-180,10]°`, A2 `[0,180]°`, A3/A4 `[-180,180]°`, A5 `[-90,90]°` as software ranges. Geometry may restrict usable travel further. Certify stopping within every joint limit; a truncated reachable sweep is not proof of a feasible stop. Preserve inward restart following controller Stop and fresh preflight, and reject incompatible stored states without snapping them into range.

## 6. Static data specification and generation

| Parameter family | Source/status | Implementation requirement |
| --- | --- | --- |
| Planar link lengths and relative A2 | Existing geometry authority | Keep 0.75/1.00 m and current angle solve |
| Patient reference and world registration | Existing head reference plus new tool reach | Explicit `T_W_K`, H and I-home agreement |
| Column foot, height and section | Arrangement from drawings; dimensions unknown | Named parameters; separate stator/rotor solids |
| A3-to-G boom vector/section | Arrangement from drawings; dimensions unknown | Positive forward reach, connected end faces/bearings |
| G-to-I offset and carrier geometry | Undimensioned and mechanically ambiguous | Remote-centre or compensated-hinge configuration selected explicitly |
| Arc and source/detector | Current synthetic proxy dimensions available | Reorient to XZ; preserve opening and enclosure bounds |
| Table, pedestal, mattress and patient | Existing synthetic fixed fixtures | Keep dimensions unless a reviewed installation change is supplied |
| Pair policy | New assembly interfaces | Stable body IDs and explicit local intended-contact regions |
| Motion policy | Existing simulation settings | 10 mm residual gap, stated tolerances, speed/brake limits, policy revision |

Do not estimate millimetres from the screenshots. Unknown dimensions are `null` in a mechanical specification; any runnable simulation configuration must supply positive, named provisional values with their provenance. Do not substitute zero for a missing dimension. Missing required geometry prevents that configuration from being activated.

Extend model records to include full local transforms, parent frames, joint axes/origins, body ownership, shape parameters, collision participation, units and provenance. Support compound boxes initially; retain a route for conservative convex parts/meshes. Keep display-only elements explicitly identified and never mark a load-bearing housing display-only to bypass interference.

Generate typed C++ scene data, scene JSON, OBJ assets, test frame samples and manifest from `parameters.json` and the generator. Include base, all joint housings, column, boom, both carriers and arc. Compare body count, IDs, shapes, local transforms, joint frames, pair policy and effective motion settings at build/runtime boundaries. Hash the normalized complete model; matching browser files alone is insufficient.

Replacement occurs while stopped: validate new model, publish a new generation, retire all permits and stale feedback, load matching renderer geometry, then allow a fresh session. Existing generated outputs remain untouched during this documentation-only phase.

## 7. Collision avoidance architecture

### 7.1 Collision coverage

Check every relevant robot–table, robot–patient, robot–floor and non-rigid self pair. Static–static installation overlap is checked at load with explicit designed floor/support contacts. A motor stator/rotor or touching bearing can have a local intended-contact mask, but the rest of those articulated bodies must remain eligible for self-collision.

Particularly important new pairs are column/base, column/link 1, boom/column, boom/table, carrier/boom, carrier/patient, arc/column and source/table. Treat same-rigid-body primitive intersections as internal construction. Do not carry forward blanket `link2/carm` or `alignment/carm` exclusions without a local geometry justification.

Represent the C as conservative sectors or multiple convex parts. One convex hull around the whole C incorrectly fills its open space. A compound box approximation may be retained if its over-coverage is measured; finer convex sectors require enclosure verification. A thin connector or table edge must remain detectable between two otherwise clear endpoints.

### 7.2 Distance and motion certification

Use a hybrid pipeline: cached static bounds, swept AABB rejection, SAT overlap/separation for boxes, then closest-feature or bounded convex distance for unresolved pairs. Retain immutable scene/snapshot inputs and bounded worker capacity.

At each motion interval, certify:

```text
distance_lower_bound(midpoint)
    > 0.010 m + pair_uncertainty + numerical_allowance
      + maximum_body_A_displacement + maximum_body_B_displacement
```

Otherwise subdivide or deny. Equality and exhausted computation return a restriction/Unknown. The 0.5 mm current interval resolution is distinct from the nominal residual gap. Numerical distance estimates must not be assumed to be conservative merely because closest-feature enumeration is exact in real arithmetic; bound numerical error or fall back to a proven lower bound.

Re-derive movement bounds for the column's actual frame, the long upper boom, remote-centre carriers and offset imaging centre. Include intermediate joint reversals on Cartesian paths. A radius around I alone does not bound link/column movement during compensated tilt. Propagate both objects' displacement when checking self-collision.

### 7.3 Reaction and braking

Keep the 250 ms simulation reaction allowance, 150 ms permit renewal deadline and independent 1 ms monitor until measured software timing supports a reviewed revision. Include snapshot age, allowed continuation, queued commands and stop dispatch in coverage. Permission timestamps must refer to the covered trajectory timeline; solver latency cannot extend authorization.

Current maximum-speed settings produce illustrative capped horizons of 90 mm linear and 30° angular. Those are starting scalar bounds only. A compound offset mechanism requires all-body motion prediction through the latest possible stop, including axis-specific speed, acceleration and braking. Never reduce a reported initial speed by clamping it to a newly selected lower speed cap.

A smaller authorized speed can improve usable motion near obstacles only when the drive enforces the same cap and its transition is covered by the permit. If state quality is unknown, use conservative bounds or deny. Commanded simulator state must be labelled as such; do not relabel it as measured feedback.

## 8. Controller, drive, CAN and index display

Preserve the requested workflow:

1. Index joystick press sends the versioned CAN intent to the controller. Controller requests collision preflight for that direction/session.
2. Collision checks the requested start trajectory through stopping. A finite Clear permit enables drive motion; Hazard/Unknown keeps it stopped.
3. Held input reaches the drive through the controller/CAN path. The single drive owner proposes and commits only covered segments, while the collision worker independently renews or revokes permission.
4. The drive publishes accepted A1–A5 and coherent status/sequence feedback through CAN. `/state` assembles complete samples; the index canvas derives the entire new frame tree from those values and the matching model.
5. Prediction failure, hazard or timeout initiates drive stop and latches the session. Continued Hold, duplicate Start and late Clear cannot restart it.
6. Release/cancel sends controller Stop to collision and drive. Required stopped-state acknowledgements retire the session; a new Start requires fresh preflight.

Use the existing actuator identities 0x201–0x205 and coherent feedback IDs 0x301–0x307. Change protocol version explicitly if new payload fields are required. Keep any internal permit channel boundary documented; do not claim it traverses physical CAN unless that transport is implemented.

The display remains in `ui/index.html`. It must render the column at M, A3 at the top, boom to G, longitudinal C and fixed table using the shared transform chain. No input-based animation is allowed. If feedback or model agreement is stale, hold the last complete pose and inhibit new movement.

Provide side, back and top camera presets with documented viewing directions: side looks along Y, top along −Z with +X toward the table, back looks from the head toward +X. Add optional engineering markers M/C/G/I and body-pair highlighting. Display a concise restriction reason such as “source approaching table”; detailed distances, intervals and model IDs belong in expandable diagnostics.

## 9. Detailed implementation plan

The software work below is implemented for revision 5 using the provisional values in `parameters.json`. Physical dimension replacement and mechanical validation remain pending.

| Phase | Files/components | Deliverable and dependency |
| --- | --- | --- |
| 1. Capture assembly specification | Parameters/schema, drawing-reference notes, data specification | Freeze topology and view directions; separate confirmed dimensions from provisional values; select the carrier/remote-centre model |
| 2. Shared geometry generation | `tools/generate_collision_reference.py`, model parameters, generated typed scene, `cSceneRegistry` | One source for bodies, local transforms, frames and reviewed contacts; base and carriers included |
| 3. Frame registration and offsets | `cDriveCalculator`, `cBodyKinematics`, shared interfaces, kinematic docs | Separate M/G/I; explicit T_W_K; accepted I pose agrees with all five actuator angles; preserve planar solve where its assumptions apply |
| 4. C-arm/carrier construction | Shape generator and model ownership | XZ arc opening +X; connected column/boom/carriers; correct rotating/stationary housings and enclosure coverage |
| 5. Prediction and permissions | `cTrajectoryPredictor`, `cProximityBackend`, collision service, drive owner | All-body continuous sweep, joint stopping limits, numerical bounds, state age and model generation; optional speed caps only with enforced profile contract |
| 6. Runtime model agreement | Startup, CAN/status codecs, `CANMocker`, `PCAN`, controller | Compiled model matches served scene and policy; mismatch/partial startup inhibits Start; no direct collision mutation of drive state |
| 7. Integrated renderer | `ui/index.html`, offline viewer template | Same transform authority, coherent drive-only motion, side/back/top views and visible limiting pair |
| 8. Verification and reporting | C++/Python tests, actual `pcan_demo`, implementation/verification docs | Pass section 10 and existing architecture 17.5/18.12; record remaining limitations and actual evidence |

Phases 2–4 should first produce a static engineering preview and registration report. If home has forbidden interference, report the exact solids and dimensions before changing geometry. Iteration must update the declared model and revalidate all affected views; never omit a body solely to obtain a passing home fixture.

## 10. Acceptance and implementation completion

| ID | Scenario | Required result |
| --- | --- | --- |
| D5-01 | Neutral side/back/top views | Match drawing topology: base→A1→A2→column→A3→upper boom→carriers→C; C opens toward table in side view |
| D5-02 | Closed home with nonzero forward tool reach | I=H and reported X/Y=0; registered physical base, M and G differ correctly; no hidden duplicate offset |
| D5-03 | Representative positive/negative X/Y commands | Full FK/IK agrees with accepted imaging-centre pose, A2 relative convention, A3 compensation and all limits |
| D5-04 | A4/A5 alone and combined | Imaging centre follows chosen remote-centre/compensation contract; carrier remains connected; no fabricated Z correction |
| D5-05 | Source/image markers and all housing corners | Correct SID marker geometry, physical housing extents and runtime/render world-corner agreement |
| D5-06 | A5 near ±90°, outward Hold and inward restart | No overshoot or creep; stopping path respects limits; inward movement requires correct Stop/new-Start lifecycle |
| D5-07 | 9/10/11 mm static gap fixtures | Below-threshold/equality deny; 11 mm clears only when the full configured uncertainty/numerical budget permits it |
| D5-08 | Thin obstacle between clear endpoints; offset column/boom sweep | Continuous collision certification catches intervening contact and intermediate joint excursions |
| D5-09 | Intended bearing contacts and non-contact portions | Explicit local contacts do not disable checks for the rest of the assemblies |
| D5-10 | Compiled/rendered scene and policy mutation | Body IDs, transforms, masks and hashes agree; any required mismatch inhibits Start |
| D5-11 | Both angular directions at verified clear poses | LAO/RAO movement occurs when full stop path clears; true obstruction reports limiting pair; no promise of full angular range at every pose |
| D5-12 | Actual executable press/hold/release and CAN trace | No movement before preflight; consistent pose feedback drives the single index canvas; release reaches drive and collision |
| D5-13 | Predicted stop and worker/input/feedback faults | Stop occurs within modelled timing, residual gap is retained, latch resists Hold/duplicate Start/late permit, fresh Stop permits rearm |
| D5-14 | Scene replacement, clean shutdown and load failure | New generation invalidates old permits; no partially loaded robot; shutdown completes without race/hang |
| D5-15 | Runtime timing and representative workspace report | Record limiting pairs, tested poses, observed worker/stop latency, Unknown/deadline counts; grid sampling is not a continuous workspace proof |

Run the standard CMake/CTest suites, generated-data reproducibility check and Python reference tests. Extend `tests/test_runtime.py` against the actual fresh `pcan_demo`; stop an interactive instance before tests because the suite owns port 8082. If CMake is unavailable, use the C++17 sources declared in `CMakeLists.txt` and record that build method. Run concurrency/undefined-behavior checks for changed ownership and shutdown paths; past sanitizer results are not evidence for a new frame/permit implementation.

For runtime clearance, evaluate the actual limiting body pair using the current 3D transforms. Do not reuse the old source/pedestal or support/table scalar formula when the assembly has changed. Record minimum simulated clearance through the permitted stop, not just a screenshot of a separated final pose.

## 11. Assumptions, missing inputs and current delivery

The current drawings are sufficient to specify assembly topology and longitudinal C orientation. They are insufficient to determine column/boom dimensions, the detailed A4/A5 mechanism, axis signs or calibration. The plan supports a declared synthetic configuration while keeping those inputs visible; it does not claim manufacturer CAD or released physical collision protection.

A mechanical definition of the rear-to-isocentre mechanism is the main remaining design input: remote-centre/orbital motion or ordinary hinge motion with available compensation. Until resolved, represent that choice as a provisional model mode and do not mark D5-04 or final assembly completion as passed.

The C++ runtime, model parameters, generated geometry, index renderer and tests now implement this revision. The runtime scene and pair policy are generated from the same parameter source used by the browser assets. Software checks cover the provisional model; they do not resolve the missing mechanical inputs or authorize physical use.
