# Design and implementation verification

Date: 2026-09-14. Scope: C++ simulation collision module, five-axis drive
integration, reproducible synthetic geometry, live joystick/viewer runtime and
fail-closed startup checks. No hardware test or physical safety release is
claimed.

## Completed checks

| Check | Result | Meaning |
| --- | --- | --- |
| `python3 tools/generate_collision_reference.py --check` | 14 generated files reproduce exactly | Parameters, generator, viewer template and output manifest agree |
| `python3 tests/test_collision_reference.py` | 11 tests passed | SI units, IDs, frames, head origin, five-axis alignment, FK, rigid transforms, rotational sweep, reference gap/SID, enclosure, OBJ topology and hashes checked |
| Fresh C++ drive kinematics suite | 24/24 scenarios; 56/56 checks passed | Existing planar behavior and five-axis rate/travel mapping remain covered |
| Fresh C++ collision suite | 36 checks passed | OBB geometry, stopping prediction, invalid input/settings, head pivot, A3 compensation, A5 corner sweep, worker, sticky stop, preflight, renewal and latch behavior checked |
| Fresh full simulator link | Passed with stub-related unused-parameter warnings | Drive, CAN, HTTP, controller and collision modules link together under C++17 with threads |
| Fresh address/undefined-behavior sanitizer run | 36 checks passed | No address or undefined-behavior finding in the collision and integrated-drive suite; macOS leak detection is unavailable and was not claimed |
| Fresh ThreadSanitizer run | 36 checks passed | No reported data race in the worker, stop monitor, mailbox or integrated drive tests |
| Real `pcan_demo` runtime acceptance | Passed | Actual HTTP to CAN to drive to worker path moved, renewed permits, stopped predictively, held its latch, acknowledged Stop and allowed safe reverse |
| Observed synthetic pedestal stop | 0.167 m source-to-pedestal clearance at X=1.143 m | Positive clearance remained when the predictive collision latch stopped commanded motion |
| Fail-closed startup acceptance | Passed | Missing assets and occupied port return failure; SIGTERM performs supervised clean shutdown |

The live dashboard and generated viewer were fetched from the running application.
The automated test confirms that the joystick page embeds `/viewer?live=1`, the
viewer contains the patient-head marker, and telemetry reports five axles. Visual
rendering and pointer-cancellation behavior remain manual browser checks.

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

Fresh address/undefined-behavior and thread-sanitized collision executables were
also compiled from the current sources and passed all 36 checks. The Apple
AddressSanitizer runtime reports that leak detection is unsupported on this
platform, so leak checking remains a release-toolchain task.

## What is not established

- A physical no-contact stopping guarantee: the worker operates on synthetic geometry and configured simulation braking bounds.
- Target-hardware real-time timing, CAN delivery/commit semantics or independent watchdog behavior. The bounded SPSC mailbox and host timing test cover the current simulation implementation only.
- Physical source/detector/arm/table enclosure accuracy, A3/A4/A5 mechanical transforms, patient coverage or calibration uncertainty.
- Guaranteed braking deceleration, jerk/response bounds, measured standstill, load effects or hardware safety acceptance.
- Collision-free home/extended/oblique poses. The OBJ snapshots are visual fixtures, and simplified joint/support contacts require reviewed masks.

The architecture's hardware-dependent `AVOID` cases and release gates remain acceptance criteria. Passing the C++ and artifact suites does not satisfy physical release gates G1–G5.
