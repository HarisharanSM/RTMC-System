# RTMC-System working guidance

## Project structure and behavior

- C++17 motion-control simulator: input/HTTP in `CANMocker/`, simulated CAN in `PCAN/`, orchestration in `SystemController/`, motion in `drive/`, shared interfaces in `includes/`.
- `drive/include/cDriveCalculator.h` is the current geometry authority: 75/100 cm links, base=(-25,0) cm, A2 relative to A1, closed EOF=(0,0) aligned to the initial patient head. A3 is independently jogged; world heading is A1+A2+A3 and is retained during X/Y. A4=LAO and A5=CRAN. Legacy positions use cm and angles use degrees.
- Read `docs/kinematics-model.md` and `docs/kinematics-bdd.md` before changing drive behavior. Preserve accepted-pose/angle agreement and the existing kinematic tests.
- `collision/` implements simulation-only predictive avoidance following `docs/collision-avoidance/architecture.md`; current coverage and limitations are in `docs/collision-avoidance/implementation.md`. Do not describe it as providing released physical collision protection.
- Revision-5 collision geometry uses a K-to-world X translation of -1.15 m, then distinct `Column`, `Boom`, `A4Carrier`, `A5Carrier` and `CArm` frames. The neutral C is in XZ and opens toward +X. These synthetic dimensions and the remote-centre carrier are provisional.

## Build and tests

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
python3 tools/generate_collision_reference.py --check
python3 tests/test_collision_reference.py
```

The CMake suite includes `collision_tests` and Python-backed `runtime_avoidance` (owns port 8082; stop any interactive demo before tests); link threaded targets through `Threads::Threads`.

If CMake is unavailable, the test sources are listed in `CMakeLists.txt` and can be compiled directly with a C++17 compiler. Keep temporary binaries outside source directories.

## Reference assets

- Edit `data/collision/reference/parameters.json`, `tools/generate_collision_reference.py` or `tools/collision_reference_viewer.html`, then regenerate with `python3 tools/generate_collision_reference.py`.
- The generator also emits `generated/scene_data.inc`, which is compiled by `cSceneRegistry`; body geometry and pair exclusions must not be duplicated manually in C++.
- Do not hand-edit `data/collision/reference/generated/`; hashes and reproducibility checks cover these outputs. OBJ and scene geometry use meters and Z up; `_deg` fields explicitly use degrees.
- Models are original synthetic proxies, not Siemens CAD. Preserve provenance, unknown calibration/braking fields and `simulation_only` status. A boolean metadata change is not hardware release evidence.
- Keep source citations and assumption tables current when dimensions or transforms change. The planned collision model uses SI internally and a legacy unit adapter.

## Conventions and constraints

- Follow existing `cClass`/`iInterface` C++ naming and module boundaries. Prefer explicit units and deterministic, side-effect-free kinematics.
- Do not introduce concurrent mutation of `cDriveCalculator` or drive state. Future avoidance integration must respect the documented single drive owner and finite permit contract.
- Preserve unrelated user changes. Do not replace existing motion limits, protocol semantics or model dimensions with manufacturer-reference values without a requested, documented change.

## Integrated five-axis runtime

- `pcan_demo` serves one joystick/C-arm canvas page, `GET /state`, model JSON,
  and ordered `POST /command` on loopback port 8082. The generated viewer is an
  offline engineering artifact; the runtime page has no iframe.
- UI commands use revision-4 CAN frames with ten explicit directions, sequence and session.
  Accepted drive poses return through coherent fixed-point CAN feedback IDs
  0x301–0x307 before `/state` and the canvas can observe them.
- Telemetry is simulated commanded feedback, not physical encoder feedback.
  Five actuator target fields remain A1/A2/A3/A4/A5 on IDs 0x201–0x205.
- The independent 1 ms stop monitor enforces 150 ms permission renewal expiry;
  the predictor uses a 250 ms simulation reaction allowance.
- Changing frames or drive/avoidance behavior requires architecture sections
  17.5 and 18.12, including `tests/test_runtime.py` against the actual
  executable. A static viewer or library-only test is insufficient.
