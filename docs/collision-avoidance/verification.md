# Design and implementation verification

## 2026-09-18 reversal correction verification

A fresh optimized C++17 application build reproduced the reported failure:
CRAN from 0 to +16.5 degrees, controller Stop, then CAUD latched at +5.7 degrees
with `source_housing / table_top` uncertainty. Stop/new-Start denied again
without movement. The run used actual HTTP/CAN/controller/drive/collision
integration; it did not exercise browser pointer events. The process shut down
after the diagnostic run.

Direct compiled predictor queries confirmed that CAUD at +5.7 checks a 30 degree
path ending at -24.3. Current pair gap: 109.37 mm; endpoint gap: 7.01 mm. At the
same initial pose, the opposite sign is Clear. A diagnostic 10 deg/s predictor
cap with the existing 120 deg/s² braking gives a 2.916667 degree path and Clear;
this is not evidence of an implemented lower-speed drive/permit contract.

A 0.1 degree sample sweep of A5=0 to +16.5 found minimum included-pair clearance
20 mm, link1 housing/floor. Sampling is diagnostic, not continuous certification.

The implemented simulator correction was then verified by direct C++17 builds:

- `collision_tests` passes modeled-standstill prediction, reduced-speed permit
  selection, drive-side cap enforcement, bounded cap reduction and all prior
  collision checks.
- `tests/test_runtime.py --binary pcan_demo` passes the actual
  HTTP/CAN/controller/drive/collision sequence: CRAN reaches at least +16
  degrees, Stop creates a fresh session, and CAUD crosses A5=0 without a false
  latch. Existing deadline, A3, 3D collision stop, residual-clearance and
  reverse-away checks also pass.
- The generated reference-data check is unchanged by this correction because
  no scene dimensions, transforms or margins changed.

The standalone kinematic suite still has the checkout's unrelated historical
KIN-22 expectation of a +25 cm Y envelope while the current geometry authority
defines +100 cm; that mismatch is not caused by the collision correction and
was not altered here. Full findings and remaining acceptance requirements are
in [reversal-clearance-analysis.md](reversal-clearance-analysis.md).

## Historical implementation checks

Date: 2026-09-17. Scope: C++ simulation collision module, five-axis drive
integration, reproducible synthetic geometry, integrated joystick/C-arm runtime and
fail-closed startup checks. No hardware test or physical safety release is
claimed.

## Completed checks

| Check | Result | Meaning |
| --- | --- | --- |
| `python3 tools/generate_collision_reference.py --check` | 14 generated files reproduce exactly | Parameters, generator, viewer template and output manifest agree |
| `python3 tests/test_collision_reference.py` | 11 tests passed | SI units, IDs, frames, head origin, five-axis alignment, FK, rigid transforms, rotational sweep, reference gap/SID, enclosure, OBJ topology and hashes checked |
| Fresh C++ drive kinematics suite | 24/24 scenarios; 56/56 checks passed | Existing planar behavior and five-axis rate/travel mapping remain covered |
| Fresh C++ collision suite | 38 checks passed | OBB geometry, versioned/legacy direction decoding, stopping prediction, head pivot, A3 compensation, worker, preflight, renewal and latch behavior checked |
| Fresh full simulator link | Passed with stub-related unused-parameter warnings | Drive, CAN, HTTP, controller and collision modules link together under C++17 with threads |
| Fresh address/undefined-behavior runtime | 21 checks passed | Full `pcan_demo` command, model agreement, feedback, collision and HTTP path completed without a finding |
| Fresh ThreadSanitizer runtime | 21 checks passed | Full application passed after removal of concurrent shared-stream writes; no reported race in CAN heartbeat/assembly, worker, monitor or HTTP path |
| Real `pcan_demo` runtime acceptance | 21 checks passed | Integrated canvas/model hashes, versioned UI CAN/session lifecycle, coherent drive CAN feedback, movement, watchdog, predictive stop, latch, Stop and safe reverse verified |
| Observed synthetic pedestal stop | 0.167 m source-to-pedestal clearance at X=1.143 m | Positive clearance remained when the predictive collision latch stopped commanded motion |
| Fail-closed startup acceptance | Passed | Missing assets and occupied port return failure; SIGTERM performs supervised clean shutdown |
| Revision-4 reference artifacts | 12 tests passed; 14 files reproduce | Head-side support, A5 limits, 10 mm policy, frames, enclosure and hashes agree |
| Revision-4 kinematics | 25/25 scenarios; 58/58 checks passed | Existing kinematics plus exact CRAN +90° and CAUD −90° saturation |
| Revision-4 collision suite | Passed | 11 mm closest-feature distance, 9/10/11 mm policy, speed-capped horizons, head-side support and bilateral home LAO/RAO preflight |
| Revision-4 real runtime | 22 checks passed | Actual executable preserved CAN feedback, watchdog, predictive stop/latch, safe reverse and clean SIGTERM |
| Observed revision-4 stop | 0.0966 m support-to-table gap at X=0.373 m | Corrected head-side column becomes the first limiting pair; stop retains more than the required 10 mm residual gap |
| Integrated display inspection | Passed | Canvas visibly places the support on the head side and shows coherent A1–A5 drive CAN feedback; no iframe |
| Revision-5 generated model | 18 files reproduce; 12 tests passed | Eight-frame drawing topology, XZ C-arm, registered head origin, carrier geometry, OBJ closure, hashes and typed scene data agree |
| Revision-5 collision suite | 42 checks passed | Generated scene/pair policy, 3D prediction, 10 mm rule, bilateral LAO/RAO, asynchronous permits and stop latch pass |
| Revision-5 real runtime | 24 checks passed | Actual executable serves the model, uses drive CAN feedback, reports the limiting pair, stops predictively, latches and safely rearms |
| Observed revision-5 stop | 0.0819 m `carm_sector_05`/`table_top` current-pose gap | The reported limiting pair retained more than the 10 mm residual gap when commanded motion stopped |
| Revision-6 A3 kinematics | 28/28 scenarios; 65/65 checks passed | Home denial, pure A3 fixed-A1/A2 arc and retained-heading X/Y behavior are executable regressions |
| Revision-6 collision suite | Passed | Ten-direction revision-4 decoding, independent A3 path, complete-stop home denial and interior clear preflight pass |
| Revision-6 real runtime | Passed | Actual A3 UI/CAN hold, coherent feedback, retained-heading translation, opposite-jog restore, predictive stop/latch and clean SIGTERM pass |
| Observed revision-6 stop | 0.0836 m `carm_sector_05`/`table_top` current-pose gap | The normal 3D stop remained active after the A3 workflow and retained more than the 10 mm residual gap |

The live dashboard was fetched from the running application and inspected in the
browser. It contains the C-arm canvas, patient-head reference, all five axle
values, controls and feedback status in one document, with no iframe. Runtime
acceptance proves its pose source is a complete drive-originated CAN sample and
that generated parameters/scene data are served to the integrated renderer.

CMake was not on PATH in this session. The unchanged kinematics test target was built directly with the available C++ compiler using the same three sources as `CMakeLists.txt`:

```sh
c++ -std=c++17 -Wall -Wextra -Iincludes -Idrive/include \
  -Icollision/include \
  tests/test_kinematics.cpp drive/src/cDriveController.cpp \
  drive/src/cDriveCalculator.cpp -o /tmp/rtmc-reference-check.IlbziY/rtmc_tests
/tmp/rtmc-reference-check.IlbziY/rtmc_tests
```

The 2026-09-14 completion run used a fresh temporary directory and rebuilt the
kinematics suite, collision suite, and `pcan_demo` directly from the current
sources. It then ran `tests/test_runtime.py` against that fresh application
binary. The temporary binaries are not project artifacts. Normal project
validation remains the CMake/CTest procedure in the root README.

Fresh address/undefined-behavior and thread-sanitized full application
executables were compiled from the current sources and each passed all 21
runtime checks. The Apple AddressSanitizer runtime does not provide leak
detection on this platform, so leak checking remains a release-toolchain task.

## What is not established

- A physical no-contact stopping guarantee: the worker operates on synthetic geometry and configured simulation braking bounds.
- Target-hardware real-time timing, physical CAN delivery, actuator commit semantics or independent hardware watchdog behavior. The in-process CAN path and host timing test cover the current simulation implementation only.
- Physical source/detector/arm/table enclosure accuracy, A3/A4/A5 mechanical transforms, patient coverage or calibration uncertainty.
- Guaranteed braking deceleration, jerk/response bounds, measured standstill, load effects or hardware safety acceptance.
- Collision-free home/extended/oblique poses. The OBJ snapshots are visual fixtures, and simplified joint/support contacts require reviewed masks.

The architecture's hardware-dependent `AVOID` cases and release gates remain acceptance criteria. Passing the C++ and artifact suites does not satisfy physical release gates G1–G5.
