# ROS 2 verification and validation

## Latest Codespaces rerun and bounded smoke repair — 2026-09-30

The user supplied a later verifier transcript for `20260930T132638Z-2108`.
Its root CTest passed 8/8 suites, reference tests passed 14/14, ROS policy
tests passed 25/25, and colcon reported 27 tests with zero failures. Native
`ros-smoke` reached bag replay, then timed out waiting for all three telemetry
topics. `integrated-demo` reached the normal action result, whose status and
message were not printed by the failing assertion. Both native phases failed;
this run does not establish native acceptance.

The replay test launched `ros2 bag play` without a discovery interval. It now
starts playback with a three-second delay, allowing the existing subscriber
to match before the first stored sample. A timeout reports counts received
per topic, counts decoded from the bag, the player exit status and its log
path. This addresses a plausible discovery race; the supplied transcript
does not prove that discovery was the only cause. The action assertion now
reports the result status/message, recent feedback, simulator state and launch
log tail. No action policy, renewal deadline or protective-stop behavior was
changed because the transcript does not identify which condition aborted it.
The action runner now names the failed health condition in its result message
without changing the abort decision or issuing any additional command.

Local macOS checks after this edit: all 25 ROS-free tests pass, and the
ROS-free action runtime test passes against an actual `pcan_demo` executable,
including normal completion, cancellation, service interruption and watchdog
latch. Python syntax and `git diff --check` pass. Native ROS/Jazzy is unavailable on this host, so
both repaired smoke phases require a fresh Codespaces verifier run. Preserve
the new `smoke/bag_play.log`, `smoke/smoke_result.json`,
`integrated-demo/demo.log` and `integrated-demo/demo_result.json` if either
phase still fails.

## Codespaces evidence and repair — 2026-09-30

The user supplied verifier output for `20260930T022905Z-4585`, using Ubuntu,
GNU 13.3 and Python 3.12.3. It establishes these **pre-repair cloud results**:

- Root CTest: 8/8 suites passed (12.80 seconds).
- All three ROS packages built; colcon reported 23 tests, zero failures/errors.
- Reference generation and reference tests failed on last-digit numeric differences.
- Native service smoke reached telemetry discovery but failed the diagnostics
  level assertion. Jazzy represents `DiagnosticStatus.level` as an octet; the
  test incorrectly compared it with integer zero.
- The integrated demo reached joint telemetry but timed out waiting for TF at a
  fixed early timestamp. A sample predating the listener's usable history can
  never become available. This startup race is a plausible cause; the pasted
  output does not contain the publisher log/frame graph needed to exclude a
  separate TF publisher issue. Action completion/cancellation was not reached.

The pasted output does not include a source revision. Its cloud build pass
must not be attributed to the subsequently repaired revision. The original
installer/idempotency, full native smoke, replay and RViz gates remain separate.

The repair uses 12-decimal absolute serialization and `math.fsum`, normalizes
signed zero and regenerates artifacts through the generator. A documented
1e-11 m outward sector-size pad covers serialization error; it does not change
motion limits or replace physical calibration. Exact output/hash checks remain.
The smoke repair uses the ROS diagnostic constant and bounded healthy-data
readiness, and selects a recent joint sample whose TF is available at that same
timestamp. Missing TF still fails with lookup/frame details; no position
comparison tolerance, controller deadline or protective-stop rule is relaxed.

Post-repair local validation: all 8 CTest suites passed (13.70 seconds), including
the actual simulator runtime gates. After tightening TF selection to require a
sample published after the check starts, the affected CTest was rerun and all
25 ROS-free policy/action/smoke tests passed under Python 3.9 and 3.12. The
exact generator check passes under both Python versions; all 14 reference tests
pass, including all-output sine/cosine perturbations and analytic enclosure.
Smoke Python syntax and `git diff --check` also pass. Both Python versions were
run on macOS, so this is not an executed Linux portability qualification.

The repair used Luna medium for reference generation and Sol medium for smoke
checks, with primary review and final integration. No model escalation was used;
per-agent token and cost totals are unavailable.

Post-repair native ROS and Codespaces reruns remain **pending**. Local regression
results below do not establish native DDS/TF/action success. Local repair evidence
is stored under `verification-results/ros2-cloud-repair-20260930/`, including the
user-provided pre-repair log. The repaired source starts from
`788d7f61fc3fd230efcde0427a8fd1560daa91e1` plus the uncommitted repair.

## Earlier local integrated-demo result — 2026-09-30

The integrated demo passes **8/8 local CTest suites** in 13.96 seconds, including
21 policy/action tests, URDF-to-C++ transform comparison and action execution
against the actual simulator. The generated-reference check (18 files) and all
12 reference tests also pass. Python/XML parsing, Bash syntax and diff checks
pass. Local evidence is under `verification-results/ros2-integrated-demo-20260930/`.
Native ROS 2/Jazzy and Codespaces acceptance remain unexecuted.

## Historical fixture-fix result — 2026-09-29

The optional ROS bridge and Codespaces setup are implemented. Local C++,
HTTP-adapter, policy and browser checks have run. **Native ROS 2 and Codespaces
acceptance remain unexecuted.** This is a simulation-only result, not physical
motion or collision-protection qualification.

The optimized local CTest run at that stage passed **all 5 suites** (11.15 seconds).
Kinematics passes 28/28 scenarios and 66/66 checks. Twenty consecutive collision
suite runs also pass; reference-scene prediction ranges from 3.859 to 10.317 ms
(median 4.771 ms), with the original 50 ms assertion unchanged. Logs are stored
locally under `verification-results/ros2-fix-20260929/` (ignored build evidence).
These host-specific measurements do not establish cloud or hard-real-time timing.

The earlier integration run passed 4/5 suites. User-supplied Codespaces logs
also showed KIN-22 failing and one intermittent 55.924 ms collision prediction.
KIN-22's old (75, 0) diagonal reaches A1's +10 degree joint limit before the
current Y=100 cm envelope. Its replacement starts at (25, 75), reaches the Y
boundary first and checks both coordinates remain stationary. The old diagonal
is retained in the long-hold joint-limit test. No production motion limits or
geometry were changed. KIN-23 now asserts exact saturation and reports the
configured limit rather than the obsolete 25 cm text.

Single-config builds now default to RelWithDebInfo when no type is specified;
the Codespaces verifier explicitly selects it. Explicit Debug remains supported
and was separately checked. An optimized build addresses avoidable unoptimized
prediction cost; the updated suite still needs a Codespaces rerun.

Environment: macOS arm64, AppleClang 21.0.0, Python 3.9, CMake 4.4.3 installed
in a temporary tool environment. Validation uses base revision
`4797a4070142f177cde6147e5f8e1b3f02a579e9`, plus the uncommitted test/build/docs
fix and integrated ROS demo changes. Historical integration evidence below used `38e8f7d2b804410086ac5367877dfe29dd4523b2`
plus integration changes. No ROS installation or container runtime was available
locally. GitHub Codespaces listing returned HTTP 403 indicating missing
`codespace` credential scope; no cloud environment was started or modified.

## Integrated demo update — 2026-09-30

The next increment adds topics/services/actions/URDF in one headless launch,
with optional RViz, a bounded `JogFor` action, a direct action client and an
owned-process native demo smoke. The bridge retains a single executor and the
C++ drive remains the sole motion owner. No production geometry, drive limits
or watchdog thresholds were changed.

The description test compiles the existing C++ drive and collision kinematics
and compares the URDF transforms for five representative poses, including
independent A3 and A4/A5 rotations. The package's visible C silhouette is a
schematic proxy, not a replacement for the generated collision scene.

Native Jazzy action generation, DDS execution, integrated launch, TF publication
and RViz are **not executed on this macOS host**. The Codespaces verifier now
runs `integrated-demo` after the existing native smoke, retaining separate
logs/results. A local pure action-state-machine check must not be presented as
an executed ROS ActionServer check. The new guide is
[integrated-demo.md](integrated-demo.md).

Implementation used one Luna agent at medium effort for the description, then
one Sol agent at medium effort for the action. Primary review corrected model
frame/visual details and requested action ownership, stop confirmation and
uncertain-command tests before running the complete suite. No model escalation
was used. Per-agent token/cost usage is unavailable; account-level quota
percentages are not task token measurements.

## Requirement-to-evidence traceability

| ID | Requirement / acceptance criterion | Evidence | Status |
| --- | --- | --- | --- |
| ROS-V01 | Simulator still builds without ROS | Root CMake configure/build | PASS locally |
| ROS-V02 | Preserve existing kinematics, collision and actual runtime gates | CTest `kinematics`, `collision_avoidance`, `runtime_avoidance` | PASS locally after fixture correction |
| ROS-V03 | Preserve generated geometry and provenance | Exact generator/hash checks; numeric portability regressions | Cloud baseline FAIL; repaired local checks PASS; cloud rerun pending |
| ROS-V04 | Wrong-source Start/Hold/Stop cannot change input sequence or session | CTest `command_source`, both modes and actual CAN-derived motion | PASS |
| ROS-V05 | Timestamp, sequence, client/direction ownership and enablement checks | 12 pure-Python policy/transport tests | PASS |
| ROS-V06 | Invalid/stale pose, counter regression and disconnection require acknowledgement | Policy tests, CTest `ros2_adapter_runtime` | PASS at policy/HTTP boundary |
| ROS-V07 | No synthetic Hold or Stop on failure; C++ watchdog stops; latch requires explicit rearm | CTest `ros2_adapter_runtime` against real executable and actual bridge HTTP transport | PASS |
| ROS-V08 | Browser monitors ROS-owned runtime and cannot command it | In-app browser inspected at localhost:8082; all ten jog buttons and Stop disabled; five angles and CAN freshness displayed | PASS locally |
| ROS-V09 | Installer fails before mutation on unsupported host | Bash syntax and actual macOS rejection (`missing /etc/os-release`) | PASS for guard only |
| ROS-V10 | Valid Python/package metadata and devcontainer configuration | Python syntax, package XML build types/license, JSON parsing, `git diff --check` | PASS; not a ROS build |
| ROS-V11 | Fresh Ubuntu installation and installer rerun both succeed | `setup_ros2_codespaces.sh` in a new Codespace, then rerun | NOT EXECUTED |
| ROS-V12 | ROS interfaces generate and Python package builds/tests through colcon | User cloud verifier: 3 packages, 23 tests, zero errors/failures | PASS for pre-repair cloud run; repaired version pending |
| ROS-V13 | Real DDS discovery, service dispatch and radians agree with simulator | Native `rtmc_ros2 smoke` | Cloud FAIL at diagnostics assertion; repair rerun pending |
| ROS-V14 | Moving bridge termination stops drive; restart cannot rearm; simulator reconnect requires Stop | Native smoke failure stages | NOT EXECUTED |
| ROS-V15 | Record/decode/replay telemetry and compare stored values, with control offline | Native smoke SQLite3 bag stages | NOT EXECUTED |
| ROS-V16 | Browser displays matching motion through Codespaces forwarding | Manual procedure below | NOT EXECUTED |
| ROS-V18 | URDF agrees with C++ drive and eight collision frames | CTest `ros2_description_fk`, five poses | PASS locally |
| ROS-V19 | Finite action, ownership, failure and cancellation policy | CTest `ros2_policy`, 21 combined policy/action tests | PASS locally |
| ROS-V20 | Action producer moves/stops actual simulator and preserves watchdog latch | CTest `ros2_action_runtime` | PASS at policy/HTTP boundary |
| ROS-V21 | Native actions, integrated launch, timestamped TF and service interruption | `demo_smoke`, verifier `integrated-demo` phase | Cloud FAIL at TF wait before action checks; repair rerun pending |
| ROS-V22 | RViz model displays correctly | Optional `use_rviz:=true` desktop launch | NOT EXECUTED |
| ROS-V17 | Independent review of command/expiry/concurrency behavior | Planned separate reviewer | NOT EXECUTED: agent usage limit reached |

For the original ROS bridge integration, the independent reviewer was not launched after the two implementation agents
hit the account usage limit. Primary integration review corrected package build
metadata, source arbitration, HTTP latency/freshness accounting, wall-clock
enforcement and the smoke test's initial movement and recording checks. This
does not substitute for an independent review or a native ROS run.

During that original integration, one initial `command_source` run failed its ROS-mode movement assertion. An
immediate run, five consecutive repeats and the final full CTest run passed.
No root cause was established for the initial failure; cloud timing stability
remains an acceptance item. Runtime timing checks intentionally fail rather
than retry motion or weaken watchdog thresholds.

## Run the cloud acceptance gate

Use the Ubuntu 24.04 devcontainer and the [run guide](README.md). Stop any
interactive simulator/bridge first. From the repository root:

```bash
bash scripts/setup_ros2_codespaces.sh
# A second run is the installer idempotency check.
bash scripts/setup_ros2_codespaces.sh
bash scripts/verify_ros2_codespaces.sh
```

Save both installer outputs separately when qualifying a fresh Codespace.
The verifier records host/tool versions, Git state, SHA-256 hashes of tracked
and untracked source files, phase results, CTest output, colcon output, ROS
process logs and bag artifacts under `~/rtmc-ros2-evidence/`. It continues
independent phases after failure and returns nonzero if any phase fails.
The KIN-22 fixture correction passes locally; rerun this gate on the updated
source to establish cloud acceptance. The collision timing threshold remains
50 ms.

The native smoke owns its simulator, bridge, recorder and player, refuses an
existing listener/service, terminates only its processes, and records a failed
or passed status in `smoke_result.json`. A 120-second outer deadline prevents
an indefinitely stalled cloud check. It uses valid X motion from home: A3
rotation at exact home is prohibited by the existing workspace envelope.

## Manual application validation in Codespaces

1. Launch `pcan_demo --command-source ros2` and the bridge with commands enabled,
   following the guide. Keep port 8082 Private and open its forwarded URL.
2. Confirm the browser says ROS 2 controls motion and disables all jog controls
   and Stop. Check that the initial five angles agree with `/rtmc/joint_states`
   after conversion from degrees to radians.
3. Run `ros2 run rtmc_ros2 jog R-up --duration 0.5` from another prepared terminal.
   Observe CAN pose sequence advancement and matching browser/ROS movement,
   followed by Disarmed and zero commanded speed.
4. Inspect `/rtmc/diagnostics` and native smoke logs for lost-input and bridge
   termination checks. Confirm a protective stop is never acknowledged merely
   by reconnecting. Use `ros2 run rtmc_ros2 jog --stop` for deliberate acknowledgement.
5. Replay the smoke's telemetry bag with the live bridge and simulator stopped.
   Confirm replayed values match the recorded data; replay does not drive motion.

Record Codespace image, exact source hashes, middleware/version, observed
browser behavior, commands and failures alongside the generated evidence.
ROS/V&V completion requires these native and cloud checks; local syntax and
HTTP-policy tests alone do not establish it. New sanitizer, physical feedback,
braking calibration, networked DDS and hard-real-time qualification were not
performed by this change.
