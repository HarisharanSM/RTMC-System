# CRAN/CAUD reversal and apparent excess clearance

Date: 2026-09-18. Diagnosed baseline: `6a28771`. Scope: diagnosis, design and simulator implementation record. The speed-aware correction changes runtime source and tests but does not change motion maxima, scene geometry or the 10 mm collision policy.

## 1. Finding

The reported reversal failure is reproducible. The main cause is a fixed worst-case angular stopping prediction, combined with a binary permit that cannot authorize slower movement. The predictor assumes 60 degrees/second even after the controller stops and resets the drive profile. Every CRAN/CAUD request therefore checks 30 degrees in the requested direction. Returning toward a previously occupied angle can be denied because the checked path continues beyond that angle into a future clearance violation.

Direction is included correctly. The 10 mm residual margin is present once in the pair test; it is not a requirement to stop only when the current gap reaches 10 mm. Motion bounds and the stopping trajectory require greater current clearance at speed. Conservative geometry and interval certification can add restriction, but neither is the primary explanation in the reproduced case.

The remedy is to authorize and enforce a speed/profile that has a certified stopping path, with explicit state and timing bounds. Reducing the prediction horizon without changing the permitted drive motion is not a valid fix.

## 2. Reproduction and evidence

The checkout was clean before documentation edits. A fresh C++17 executable was compiled from the application sources with `clang++ -O2 -std=c++17 -pthread`. It served the repository assets through `--assets` and ran the actual HTTP → simulated CAN → controller → drive → collision → CAN feedback path on port 8082. The diagnostic process was shut down after the run. These were HTTP command lifecycle checks, not browser pointer-event tests or physical device tests.

### 2.1 Actual application sequence

Initial pose: X=Y=0, A1=-180, A2=180, A3=A4=A5=0. Hold packets were sent approximately every 55 ms.

| Action | Observed result |
| --- | --- |
| Start/hold `L-up` (CRAN+) | A5 reached +16.5 degrees; Running; pose sequence 11 |
| Controller Stop | Disarmed; motion profile reset |
| Start/hold `L-down` (CAUD−) | A5 decreased, then latched at +5.7 degrees; pose sequence 19; speed field zero |
| Reported reason | `clearance cannot be certified over the prediction interval (source_housing / table_top)` |
| Stop, then fresh CAUD Start | Preflight denied again at +5.7 degrees; no further pose commit |

At +5.7 degrees the checked CAUD trajectory ends at -24.3 degrees. The original angle was 0 degrees. The source/table current-pose gap is approximately 109.37 mm at +5.7 degrees, whereas it is approximately 7.01 mm at -24.3 degrees, below the configured 10 mm residual requirement. The denial thus has a real future clearance cause within the assumed trajectory, despite generous clearance at the held pose.

### 2.2 Direct predictor experiments

An external diagnostic harness linked the unchanged calculator, compiled reference scene, transforms, proximity backend and predictor. It queried X=0, 10 and 20 cm, Y=0, A4=0 and zero world yaw at several A5 angles in both signs. The table below reports selected X=0 results. The 10 degrees/second rows change prediction settings only in the diagnostic harness; they do not validate a slower runtime implementation.

| A5 | Requested sign | Assumed maximum angular speed | Predicted travel | Result |
| --- | --- | --- | --- | --- |
| 0 degrees | CAUD− | 60 deg/s | 30 degrees | Hazard, source/table |
| 0 degrees | CRAN+ | 60 deg/s | 30 degrees | Clear |
| +5.7 degrees | CAUD− | 60 deg/s | 30 degrees | Unknown, source/table |
| +5.7 degrees | CRAN+ | 60 deg/s | 30 degrees | Clear |
| +5.7 degrees | CAUD− | 10 deg/s | 2.916667 degrees | Clear |
| +5.7 degrees | Static diagnostic | 0 deg/s | 0 degrees | Clear |
| -24.3 degrees | Static diagnostic | 0 deg/s | 0 degrees | Unknown; source/table gap 7.01 mm |

The different signs yield different verdicts at the same pose, directly contradicting a missing-direction explanation. The 10 deg/s diagnostic retains 120 deg/s² braking: `10*0.25 + 10²/(2*120) = 2.916667 degrees`. This is distinct from the A3 policy, whose 20 deg/s² braking yields a 5 degree horizon at 10 deg/s.

A separate 0.1 degree pose sweep from A5=0 to +16.5 checked all non-excluded body pairs. The minimum sampled gap was 20 mm, `link1_housing / floor`. This supports that the return corridor is geometrically available in the proxy model. A finite sample sweep is not a continuous-path proof; prospective permits must still certify the whole executed path and stop.

## 3. Assessment of the reported possibilities

| Suspected issue | Finding | Consequence |
| --- | --- | --- |
| Margin greater than 10 mm or counted twice | No duplicate fixed 10 mm addition found in the examined runtime path | Keep residual clearance distinct from stopping travel, interval motion bounds and geometry approximation |
| Direction omitted from trajectory | Not supported; `PoseAt` adds signed CRAN/LAO/X/Y travel and `AxlesAt` adds signed A3 travel | Preserve the signed trajectory; strengthen reversal velocity handling rather than remove absolute values from distance bounds |
| Prediction extends beyond the original pose | Confirmed | Continuous jog has no target at the old angle, so full-speed CAUD checks through and past it |
| Space exists but no movement starts | Reproduced | Full-speed fallback remains after Stop; no low-speed permit can be selected |

An equal-duration reverse hold also does not guarantee an exact return: acceleration ramps, shortened steps, the 50 ms cadence and receipt timing affect integrated displacement. That issue is separate from the reproduced collision latch. A request to return exactly to a saved angle would require an explicit target mode; a CAUD button currently means continuous negative jogging.

## 4. Root-cause trace

### 4.1 Full-speed fallback ignores the restarted profile

`cDriveController::SubmitCollisionRequest` always supplies maximum linear/angular speeds and `velocityMeasured=false`. `StoppingTravel` then replaces the supplied speed with the configured maximum. It cannot use the calculator's ramp state, the actually committed joint delta or a lower requested cap.

For CRAN/CAUD and LAO/RAO:

```text
reaction allowance T = 0.250 s
maximum speed V = 60 deg/s
braking magnitude B = 120 deg/s²
reaction travel = V*T = 15 degrees
braking travel = V²/(2*B) = 15 degrees
total travel = 30 degrees
q5(f) = q5_current + direction_CRAN * 30 * f, 0 <= f <= 1
```

`StopDrive` resets the calculator, but the next preflight still uses this 30 degree path. Actual simulated angular increments restart at 0.3 degrees for the first 50 ms step and ramp upward. This mismatch explains denial at rest and early return stopping. The conservative fallback is intentional for unknown physical velocity; the simulator needs an explicitly identified bounded simulation-state source instead of relabelling commanded feedback as measured velocity.

### 4.2 Binary permission cannot trade speed for clearance

`CollisionPermit` contains a verdict and expiry, but no enforced permitted speed, acceleration or segment trajectory. The drive always proposes motion using its normal constants. The collision worker immediately requests a sticky stop for either Hazard or Unknown. The system cannot try a certifiable slower continuation before latching.

A tighter usable design must connect the predictor's chosen profile with the exact segment the single drive owner will execute. A predictor-only speed reduction would authorize motion faster than the evaluated trajectory.

### 4.3 The residual gap is applied to the swept path

`CheckPair` computes:

```text
required_gap = pairMarginM + movingBodyMotionBound + otherBodyMotionBound
clear if midpoint_distance_lower_bound > required_gap
```

The two motion terms bound changes from the interval midpoint to every pose in that interval. They are not a second braking distance or a second residual margin. The predictor recursively subdivides unresolved intervals, shrinking those bounds. Removing them would miss collisions between checked poses.

`intervalMotionToleranceM=0.0005` stops refinement when the summed motion bound is at most 0.5 mm; it is not directly added as fixed 0.5 mm padding. A near-threshold clear trajectory can still receive Unknown because certification stops before resolving it. The numerical behavior should be documented and tested separately from the 10 mm geometric policy.

The parameter JSON also declares 0.01 m, but the runtime test uses `PredictionSettings::pairMarginM`; the generator does not inflate every body by that value. Those parallel declarations can drift and should become one generated policy authority.

### 4.4 Geometry may occupy more than its apparent outline

The C-arm arc uses eight oriented boxes enclosing 22.5 degree annular sectors. For inner radius 0.70 m and half-sector angle 11.25 degrees, the inner radial bound is `0.70*cos(11.25 degrees)`, about 13.45 mm inward of the nominal inner circle on the sector centreline. Rectangular corners add other shape-dependent excess. This is geometric enclosure, not margin inflation.

The index renders the same box scene as collision, so there is no evidence of a separate hidden 10 mm mesh expansion in this path. View angle, occlusion and the translucent patient fixture can make relevant occupied space difficult to judge. The reproduced limiting bodies are the source box and tabletop, so finer arc segmentation alone will not solve this example.

## 5. Additional issues and their priority

| Priority | Source finding | Proposed response |
| --- | --- | --- |
| P1 | Non-A3 `FeasibleFraction` clips the predicted stop to reachable pose limits and can label the shortened path Clear | Certify a dynamically attainable stop before a limit, consistently for every axis; do not simply shorten the geometric check |
| P1 | Signed actual velocity and acceleration are absent; direction alone does not describe residual motion during a reversal | Require established standstill before reversing, or model braking in the old direction followed by the new direction |
| P1 | Requests lack a snapshot timestamp; expiry is based on predictor entry, while the drive separately maintains a submit-time deadline | Bind permissions to snapshot time and absolute deadlines; explicitly budget state age, worker latency, dispatch and stop response |
| P1 | Starting axles and Cartesian pose are finite-checked but not fully checked for agreement at predictor entry | Validate joint limits, FK consistency, yaw and profile/state version before any candidate permission |
| P2 | Input sequence/session fields are emitted, but `InjectReceivedMessage` dispatches decoded directions without checking those fields | Implement replay/order/session validation and Stop priority; not identified as the cause of this controlled reproduction |
| P2 | The UI sends Stop and a subsequent Start asynchronously without awaiting Stop acknowledgement | Serialize motion-session transitions and reject late holds; test fast reversal and release during preflight |
| P2 | `speed_dps` reflects configured SetSpeed, not signed achieved velocity; zero speed does not establish physical standstill | Separate command cap, modeled velocity and feedback provenance in diagnostics |
| P2 | Relative-motion certification uses sums of body bounds and checks pairs even when their mutual geometry is invariant | Cache rigid/invariant-pair checks and use proven relative bounds; preserve checks against static obstacles |
| P2 | The first unresolved pair ends the search; it is not necessarily the nearest pair or earliest time of violation | Label it as the reported limiting candidate, and add explicit pose/time/gap diagnostics |
| P2 | Worker/permission timeouts use the same visible latch state as a geometry rejection | Provide distinct reason codes; the reproduced stop was geometry uncertainty, not a timeout |
| P2 | Transform constants exist separately in calculator, body kinematics and JSON; runtime tests' `x_m` helper omits nonzero A3 offset | Generate common constants and extend arbitrary-yaw parity tests and runtime coordinate reconstruction |

These are source findings and proposed work, not claims that every issue occurred in the user's session. Existing entire rigid-body pair exclusions and the provisional carrier construction require their already documented audit; loosening them is not a remedy for unwanted stops.

## 6. Architecture: permit the executable trajectory

### 6.1 Motion snapshot

The drive owner publishes an immutable snapshot containing all five accepted angles, FK-derived imaging pose and yaw, signed velocity/acceleration bounds, state-source type, snapshot timestamp, profile generation, motion kind/sign, session and sequence, plus scene/policy generation. State-source values should distinguish bounded simulator state, measured physical state and unknown state. Unknown retains conservative fallback; a fresh simulator Stop can establish modeled standstill through the simulator state transition.

Do not derive an instantaneous physical velocity solely from a CAN angle difference or the SetSpeed field. Differencing may support a simulator estimate with explicit sample-time and quantization bounds; physical feedback needs its own error bounds.

### 6.2 Segment and stop proposal

Introduce a deterministic, side-effect-free motion proposal shared by drive and prediction. A proposal contains the next permitted segment, bounded acceleration, selected speed cap, resulting profile state, and the subsequent stopping trajectory. The owner applies profile state only after accepting the matching permit.

For a constant-speed scalar illustration:

```text
v_brake = |v0| + a*T                     (use the enforced speed cap when valid)
travel = |v0|*T + 0.5*a*T² + v_brake²/(2*b)
```

If the cap is reached during T, split the integral into acceleration and capped portions. If current speed is above a newly requested cap, preserve that initial speed and certify deceleration to the cap. Never clamp an already moving speed downward before evaluating its stop. For signed reversal, split the path into old-direction braking and new-direction acceleration unless standstill is established.

The full prediction must cover the next executable segment and every stop reachable under the permitted continuation and maximum reaction delay. Define whether the 50 ms segment is already inside the reaction allowance; do not count it twice or omit it. A start-at-rest example with a=120 deg/s² and T=0.25 s has 3.75 degrees reaction travel plus 3.75 degrees braking travel (7.5 total), provided that time convention and profile bound actually apply. It is illustrative, not an independently approved replacement horizon.

For coordinated X/Y retain full five-joint kinematics and body sweeps; a scalar end-effector stopping distance alone does not bound every joint's physical braking path.

### 6.3 Speed selection before stop

The collision worker evaluates a bounded number of candidate continuation profiles, highest useful speed first. For example, an initial engineering candidate set might be normal, half, quarter and low-speed jog. Values become configuration only after acceptance. Search remains off the drive callback.

For each candidate, certify the actual current state → next segment → bounded reaction continuation → stop against all included geometry and joint/workspace limits. Select the fastest certified candidate and include it in the permit. Do not assume feasibility is monotonic in speed for arbitrary multi-joint trajectories; bisection is appropriate only where nested path envelopes have been established.

If a slower candidate is selected while running, its deceleration transition must already be covered. The previously active permission and stop envelope remain authoritative until a replacement is safely accepted. If the worker cannot establish any valid continuation before the existing deadline, stop and latch. Slowing under a current valid session is allowed; restarting after a latch still requires controller Stop and a fresh Start.

### 6.4 Permit enforcement and ownership

Extend the permit with snapshot/profile identity, motion kind and sign, exact allowed segment or equivalent bounded trajectory contract, speed and acceleration bounds, validity interval and certified terminal-stop state. The drive verifies session, sequence, age, policy generation and current profile identity before committing; a direction-only match is insufficient.

The independent monitor continues to enforce expiry and revocation. It must not become a second mutable profile owner. A revocation commands the defined stop mechanism; modeled braking integration, if introduced, belongs to the existing single drive owner or a separately defined backend contract. The existing simulator freezes accepted pose on Stop; physical standstill is outside that model.

### 6.5 Continuous jog versus exact return

The implemented simulator fix keeps CRAN/CAUD as continuous hold commands, adds progressive slowing where needed, and permits crossing the original angle when the modeled stopping envelope is clear. It does not automatically stop at zero: zero is not the target of a CAUD hold.

An optional future “Return to saved pose” command would capture a complete coherent five-axis pose and use a target-constrained deceleration profile. It must stop at the target within a declared tolerance, check the path continuously, and preserve Stop/latch behavior. This is a separate feature, not assumed part of the correction.

## 7. Clearance, numerical and display policy

Keep the residual surface requirement at 10 mm, with equality denied under the current strict policy. Define one geometry source and one margin source. Document enclosure error per proxy and an explicit numerical error allowance; do not conceal either inside a second margin. Report a current distance and a conservative swept minimum or certified lower bound separately.

Refine uncertain intervals within a bounded computation budget. For close pairs, retain the box closest-feature calculation with a reviewed numerical error treatment or use conservative separation bounds. For rotating concave structures, improve convex decomposition or add a tighter narrow phase only where measured false denials justify it. Do not replace the C-arm with a single convex hull that fills its opening, or shrink proxies without enclosure evidence.

The index should show distinct operator reasons such as “Slowing for clearance,” “Predicted clearance limit,” “Workspace limit,” “Waiting for collision permission,” and “Permission expired.” A details view should expose:

- Current pose, requested direction and signed modeled velocity/source.
- Command cap and collision-permitted speed, separately from achieved velocity.
- Predicted end/stop angle and time horizon.
- Reported body pair, current surface gap and swept gap/lower bound.
- Residual 10 mm requirement, interval bound and solver depth/budget outcome.
- Snapshot age, evaluation latency and permission expiry.

Highlight the reported pair and offer a diagnostic predicted-stop ghost. Keep actual movement driven by the coherent CAN sample; a ghost must be visibly marked as prediction. At nonzero A3 yaw, show the selected heading and preserve the documented local A4/A5 transform convention.

## 8. Implementation status and remaining plan

| Package | Files/components | Exit condition |
| --- | --- | --- |
| 1. Freeze regressions and diagnostics | `tests/test_runtime.py`, `tests/test_collision.cpp`, collision result types, `/state` | Reproduce 0 → +16.5 → +5.7 denial and expose why; preserve original evidence |
| 2. Snapshot/profile authority | `commonDrive.h`, `cDriveCalculator`, `cDriveController`, collision request types | Signed bounded simulator state and pure proposals agree with committed FK and A3 heading |
| 3. Finite trajectory permissions | `cTrajectoryPredictor`, supervisor, permit interface, drive gate | Candidate speeds are evaluated with complete transition and stop; drive cannot exceed granted motion |
| 4. Limits and reversal lifecycle | Predictor, controller, PCAN input, UI session handling | No clipped-stop Clear; ordering and old-direction residual motion handled; latch cannot auto-rearm |
| 5. Geometry/numerical efficiency | Generator, body kinematics, proximity, pair policy | Shared constants, reviewed enclosure/error budget and bounded timing; no hidden margin reduction |
| 6. Integrated display | `ui/index.html`, CAN diagnostics, `/state` | Operator sees speed limiting and actual reason; pose remains CAN-derived |
| 7. Completion evidence | Tests and verification documents | Fresh actual-executable return/collision/timeout acceptance and regression suites pass |

The simulator portion of packages 1–3 is implemented: bounded profile speed is
published with explicit modeled provenance; a finite speed set is evaluated;
the selected cap is returned in the permit and enforced by the drive calculator;
and the actual-executable reversal regression is present. Package 4 remains
partial because physical signed velocity/reversal lobes, snapshot/profile IDs,
input replay ordering and a physical stopping backend are not implemented.
Packages 5–6 remain future work. These limits prevent treating the simulator
correction as physical-system safety evidence.

## 9. Acceptance gates

| ID | Required scenario and evidence |
| --- | --- |
| REV-01 | At the reproduced pose, confirm opposite signs inspect opposite angle paths and diagnose the 30 degree baseline denial |
| REV-02 | Real executable: home → CRAN +16.5 → Stop → CAUD can reach/cross the original 0 angle under a certified profile; no teleport or collision bypass |
| REV-03 | Repeat from multiple partial angles, X/Y locations and nonzero A3/A4; compare full five-axis pose, not just displayed A5 |
| REV-04 | Hold CAUD past the original pose: slow and eventually stop before any included pair breaches residual clearance; record continuous modeled stop, not only endpoint gap |
| REV-05 | At rest, moving slowly, at maximum speed and above a new lower cap: predictions cover every permitted segment and transition |
| REV-06 | Old-direction velocity on reversal is either proven zero or its braking lobe is included before new-direction movement |
| REV-07 | Independent 9/10/11 mm static fixtures retain policy; swept thin-obstacle fixtures deny collision between clear endpoint poses |
| REV-08 | ±90 degree A5, A3 travel and X/Y boundaries produce a certified stop rather than Clear for a clipped trajectory |
| REV-09 | Late/stale/duplicate/wrong-session/profile permits and reordered input cannot move the drive; release during preflight produces no late commit |
| REV-10 | Missing Hold, worker delay and solver exhaustion stop within the validated deadline, with distinct reasons; continued Hold cannot restart a latch |
| REV-11 | Browser press/release/cancel/blur and fast reversal preserve Stop ordering; prediction ghost never replaces accepted CAN pose |
| REV-12 | Generated/compiled/rendered geometry and policy agree; curved-proxy refinement retains enclosure and the residual margin |
| REV-13 | Existing kinematic, collision, generated-data and actual `pcan_demo` acceptance gates pass; benchmark worst relevant prediction cases and run sanitizers after ownership/protocol changes |

Previously visited poses are not automatically authorized: a changed scene, insufficient current stopping distance, different A3/A4 configuration or invalid state can legitimately prevent return. The return guarantee is conditional on a certified traversable path under the supported model and profile.

## 10. Status and limits

Confirmed by fresh runtime: early CRAN/CAUD return stop and repeat-start denial. Confirmed by direct compiled predictor: signed paths, a single 10 mm residual comparison, full-speed 30 degree horizon, current versus future source/table gaps, and clearance of slower diagnostic paths. Confirmed only by sampling: the 0–16.5 degree corridor's 20 mm minimum among included pairs. Additional protocol, profile and geometry concerns are source-inspected.

Implemented and freshly verified in the simulator: modeled profile-speed input,
acceleration/deceleration-aware stopping travel, finite adaptive angular caps,
permit-carried cap enforcement, unit regressions and the real `pcan_demo`
CRAN-to-CAUD return through A5=0. No model dimensions or margins were changed.

Not implemented: measured signed physical velocity, old-direction braking lobes
for a reversal before standstill, snapshot/profile identity, exact segment
permits, replay/order hardening, richer UI diagnostics, and calibrated physical
braking evidence. Synthetic geometry and unmeasured physical braking therefore
remain the project’s existing scope limits.
