# Design and reference-data verification

Date: 2026-09-14. Scope: C++ simulation collision module, drive integration, reproducible synthetic geometry, viewer and artifact checks. No hardware test or physical safety release is claimed.

## Completed checks

| Check | Result | Meaning |
| --- | --- | --- |
| `python3 tools/generate_collision_reference.py --check` | 14 generated files reproduce exactly | Parameters, generator, viewer template and output manifest agree |
| `python3 tests/test_collision_reference.py` | 10 tests passed | SI units, IDs, frames, home/extension FK, relative elbow convention, rigid transforms, pure rotational surface motion, reference gap/SID, sector enclosure, OBJ topology and hashes checked |
| Existing C++ drive kinematics suite | 24/24 scenarios; 56/56 checks passed | Existing behavior remains covered; no collision-supervision coverage implied |
| C++ collision suite | 30/30 checks passed | OBB geometry, stopping-path prediction, invalid inputs/settings, reference-scene timing, asynchronous worker, sticky stop and integrated preflight/renewal/latch behavior checked |
| Address/undefined-behavior sanitizer build | 30/30 checks passed | No sanitizer finding in the collision suite |
| Thread sanitizer build | 30/30 checks passed | No reported data race in worker, monitor, mailbox or drive integration tests |
| Full simulator link | Passed with existing stub-related warnings | Drive, CAN, controller and collision modules link together under C++17 with threads |
| Browser visual inspection | Model renders with table, segmented C-arm, arm links and patient fixture | Layout and visible geometry checked |
| Browser Oblique preset | A1=-60°, A2=100°, A3=25°, A4=-15°; EOF display approximately (0.891,-0.007) m | Preset updates the articulated view and reported EOF |

Browser checks used a loopback static server serving only the generated directory.
Direct `file://` navigation was blocked by the automated browser's URL policy,
so direct-file opening was not browser-verified. The generated viewer embeds its
data and has no external dependencies; users can open the HTML in their own browser.

CMake was not on PATH in this session. The unchanged kinematics test target was built directly with the available C++ compiler using the same three sources as `CMakeLists.txt`:

```sh
c++ -std=c++17 -Wall -Wextra -Iincludes -Idrive/include \
  -Icollision/include \
  tests/test_kinematics.cpp drive/src/cDriveController.cpp \
  drive/src/cDriveCalculator.cpp -o /tmp/rtmc-reference-check.IlbziY/rtmc_tests
/tmp/rtmc-reference-check.IlbziY/rtmc_tests
```

The temporary path above records this run, not a required project directory. The collision suite, application, address/undefined-behavior sanitizer build and thread sanitizer build were likewise compiled directly as C++17 binaries. Normal project validation remains the CMake/CTest procedure in the root README.

## What is not established

- A physical no-contact stopping guarantee: the worker operates on synthetic geometry and configured simulation braking bounds.
- Target-hardware real-time timing, CAN delivery/commit semantics or independent watchdog behavior. The bounded SPSC mailbox and host timing test cover the current simulation implementation only.
- Physical source/detector/arm/table enclosure accuracy, A3/A4 mechanical transforms, patient coverage or calibration uncertainty.
- Guaranteed braking deceleration, jerk/response bounds, measured standstill, load effects or hardware safety acceptance.
- Collision-free home/extended/oblique poses. The OBJ snapshots are visual fixtures, and simplified joint/support contacts require reviewed masks.

The architecture's hardware-dependent `AVOID` cases and release gates remain acceptance criteria. Passing the C++ and artifact suites does not satisfy physical release gates G1–G5.
