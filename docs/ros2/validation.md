# ROS 2 verification and validation

## Result as of 2026-09-29

The optional ROS bridge and Codespaces setup are implemented. Local C++,
HTTP-adapter, policy and browser checks have run. **Native ROS 2 and Codespaces
acceptance remain unexecuted.** This is a simulation-only result, not physical
motion or collision-protection qualification.

The final local CTest run passes 4 of 5 suites. The existing `kinematics` suite
still fails KIN-22: 27/28 scenarios and 63/65 checks pass. KIN-22 expects a
25 cm Y limit, whereas the current model allows 100 cm; it reports Y=61.3874 cm
and different X values at ticks 50 and 100. The kinematics implementation and
its tests were not changed by this integration. The failure also occurred in
the initial build and is not suppressed by the verification script.

Environment: macOS arm64, AppleClang 21.0.0, Python 3.9, CMake 4.4.3 installed
in a temporary tool environment. The base Git revision was
`38e8f7d2b804410086ac5367877dfe29dd4523b2`, plus the uncommitted integration
changes. No ROS installation or container runtime was available locally.
GitHub Codespaces listing returned HTTP 403 indicating missing `codespace`
credential scope; no cloud environment was started or modified.

## Requirement-to-evidence traceability

| ID | Requirement / acceptance criterion | Evidence | Status |
| --- | --- | --- | --- |
| ROS-V01 | Simulator still builds without ROS | Root CMake configure/build | PASS locally |
| ROS-V02 | Preserve existing kinematics, collision and actual runtime gates | CTest `kinematics`, `collision_avoidance`, `runtime_avoidance` | KIN-22 FAIL; other two PASS |
| ROS-V03 | Preserve generated geometry and provenance | Generator `--check` (18 files), reference tests (12 tests) | PASS |
| ROS-V04 | Wrong-source Start/Hold/Stop cannot change input sequence or session | CTest `command_source`, both modes and actual CAN-derived motion | PASS |
| ROS-V05 | Timestamp, sequence, client/direction ownership and enablement checks | 12 pure-Python policy/transport tests | PASS |
| ROS-V06 | Invalid/stale pose, counter regression and disconnection require acknowledgement | Policy tests, CTest `ros2_adapter_runtime` | PASS at policy/HTTP boundary |
| ROS-V07 | No synthetic Hold or Stop on failure; C++ watchdog stops; latch requires explicit rearm | CTest `ros2_adapter_runtime` against real executable and actual bridge HTTP transport | PASS |
| ROS-V08 | Browser monitors ROS-owned runtime and cannot command it | In-app browser inspected at localhost:8082; all ten jog buttons and Stop disabled; five angles and CAN freshness displayed | PASS locally |
| ROS-V09 | Installer fails before mutation on unsupported host | Bash syntax and actual macOS rejection (`missing /etc/os-release`) | PASS for guard only |
| ROS-V10 | Valid Python/package metadata and devcontainer configuration | Python syntax, package XML build types/license, JSON parsing, `git diff --check` | PASS; not a ROS build |
| ROS-V11 | Fresh Ubuntu installation and installer rerun both succeed | `setup_ros2_codespaces.sh` in a new Codespace, then rerun | NOT EXECUTED |
| ROS-V12 | ROS interfaces generate and Python package builds/tests through colcon | Verifier `colcon-build`, `colcon-test`, `colcon-results` | NOT EXECUTED |
| ROS-V13 | Real DDS discovery, service dispatch and radians agree with simulator | Native `rtmc_ros2 smoke` | NOT EXECUTED |
| ROS-V14 | Moving bridge termination stops drive; restart cannot rearm; simulator reconnect requires Stop | Native smoke failure stages | NOT EXECUTED |
| ROS-V15 | Record/decode/replay telemetry and compare stored values, with control offline | Native smoke SQLite3 bag stages | NOT EXECUTED |
| ROS-V16 | Browser displays matching motion through Codespaces forwarding | Manual procedure below | NOT EXECUTED |
| ROS-V17 | Independent review of command/expiry/concurrency behavior | Planned separate reviewer | NOT EXECUTED: agent usage limit reached |

The independent reviewer was not launched after the two implementation agents
hit the account usage limit. Primary integration review corrected package build
metadata, source arbitration, HTTP latency/freshness accounting, wall-clock
enforcement and the smoke test's initial movement and recording checks. This
does not substitute for an independent review or a native ROS run.

One initial `command_source` run failed its ROS-mode movement assertion. An
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
independent phases after failure and returns nonzero if any phase fails. The
known KIN-22 failure therefore prevents an overall pass even if ROS checks
pass. Resolve it in a separately scoped kinematics change; do not silently
change model dimensions or weaken the assertion here.

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
