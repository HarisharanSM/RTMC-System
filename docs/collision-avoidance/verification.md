# Design and implementation verification

Date: 2026-09-15. Scope: C++ simulation collision module, five-axis drive
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
