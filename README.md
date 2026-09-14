# RTMC-System

**Real-Time Motion Control System Simulator for Precision Imaging Devices**

RTMC-System is a C++17 simulator for a five-axis motion-control stack used in precision imaging equipment (the axis naming — `LAO/RAO`, `CRAN/CAUD`, `X/Y` — mirrors the positioning conventions of C-arm / angiography-style imaging gantries). It models the full signal path from operator input to actuator command — joystick input → CAN bus transport → drive kinematics — without requiring any physical PCAN hardware or motor hardware. A browser-based dual-joystick dashboard drives the pipeline over HTTP, standing in for a real hardware joystick and CAN interface.

## How it works

```
Browser dashboard (ui/index.html)
        │  HTTP GET (button press)
        ▼
CANMocker  — HTTP-to-CAN bridge, listens on :8082
        │  builds a TPCANMsg frame, injects it into the bus
        ▼
PCAN layer — cPCANController / cCANDriveHandler
        │  decodes frame → joystickSignal, dispatches to subscribers
        ▼
SystemController — cControlManager
        │  routes joystick/start/stop signals to the drive
        ▼
Drive layer — cDrive / cDriveController / cDriveCalculator
        │  integrates motion, applies kinematics & limits
        ▼
PCAN layer — SetSpeed() / SetPosition()
        (logged to stdout in place of real CAN writes)
```

The dashboard displays the backend's simulated commanded pose. Everything below the dashboard is simulated: the "PCAN" layer mimics the real PCANBasic SDK's types and constants but stubs the actual hardware calls with console logging, so the whole pipeline runs on a plain Linux machine with no CAN adapter attached.

## Architecture

The system is built around motion and collision interfaces (`includes/iSystemController.h`, `iPCANController.h`, `iDrive.h`, `iCollisionSupervisor.h`) that decouple orchestration from implementation, so the simulated CAN/drive layers could be swapped for real hardware without touching the composition logic in `main.cpp`.

| Module | Responsibility |
|---|---|
| `includes/` | Shared interfaces (`iSystemController`, `iPCANController`, `iDrive`), simulated PCANBasic types (`PCANTypes.h`), and common data structures (`commonDrive.h`: `drivePosition`, `joystickSignal`, `AxelPostion`) |
| `PCAN/` | Simulated CAN transport layer — `cPCANSender`/`cPCANReceiver` (stubbed I/O), `cPCANController` (bus orchestrator with a pub/sub subscription registry keyed by CAN message ID), `cCANDriveHandler` (decodes raw CAN frames into `joystickSignal`) |
| `SystemController/` | `cControlManager` — the composition root; wires the CAN controller and drive together and routes `DRIVE_MSG`/`START_DRIVE_MSG`/`STOP_DRIVE_MSG` to the appropriate drive calls |
| `CANMocker/` | `cCANMocker` — a minimal HTTP server (raw POSIX sockets, port 8082) that translates GET requests from the web UI into `TPCANMsg` CAN frames and injects them into the bus |
| `drive/` | Motion subsystem — `cDrive` (façade), `cDriveController` (stateful engine: e-stop, error handling, current position), `cDriveCalculator` (2-link planar-arm forward/inverse kinematics, reach and joint-limit gating, trapezoidal velocity profiling) — see [docs/kinematics-model.md](docs/kinematics-model.md) |
| `ui/` | `index.html` — a self-contained dual-joystick dashboard (left pad: LAO/RAO & CRAN/CAUD, right pad: X/Y) that polls the CAN mocker over HTTP while a button is held |

## Building

Requirements: a C++17 compiler, CMake ≥ 3.10, and a POSIX/Linux environment (the CAN mocker uses raw BSD sockets).

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

This produces `pcan_demo`, the existing `rtmc_tests` kinematics suite, and the
`collision_tests` predictive-avoidance suite.

## Tests

```bash
cd build && ctest --output-on-failure     # or: ./build/rtmc_tests
```

`rtmc_tests` is a self-contained BDD suite covering the drive kinematics — home
pose, IK/FK round-trip, reach and joint limits, motion profiling, position/angle
consistency and the safety interlocks. Each scenario prints its Given/When/Then
and maps to an ID in [docs/kinematics-bdd.md](docs/kinematics-bdd.md); the
design rationale is in [docs/kinematics-model.md](docs/kinematics-model.md).
`collision_tests` covers predictive stopping paths, static and self-collision
geometry, asynchronous supervision, permit renewal and the protective-stop latch.

## Running

```bash
./build/pcan_demo
```

On startup, `pcan_demo`:
1. Initializes the simulated PCAN controller and drive subsystem via `cControlManager`.
2. Starts `cCANMocker`, which listens on **port 8082** for joystick commands from the UI.
3. Serves one integrated joystick/C-arm canvas and live telemetry from the same process on port 8082.

Then open `http://localhost:8082` in a browser and use the on-screen joystick buttons to drive the simulated motion pipeline; system activity is logged to stdout in place of real CAN traffic.

The asset root is recorded by CMake. If relocating the executable, use
`./pcan_demo --assets /absolute/path/to/RTMC-System`. The listener is loopback-only.
The runtime page renders generated scene geometry directly in `index.html`; it
does not embed a second viewer. The generated viewer file remains available for
offline inspection. No separate Python server is needed.

X/Y are fixed offsets from the initial patient-head position. A1/A2 retain the
existing geometry; A3 automatically cancels their heading, A4 is LAO and A5 is
CRAN. The console shows all five drive-originated CAN feedback angles, feedback
age and avoidance state. UI press/hold/release commands also enter through the
versioned simulated CAN codec.

## Status

This is a simulator/prototype: there is no dependency on the real PCANBasic SDK or physical motor hardware, and several pieces (e.g. `iSystemController::ProcessUiCommand`) are stubs. It's intended for exercising and demonstrating the motion-control architecture end-to-end in software.

## Predictive collision avoidance design and 3D reference

The [collision avoidance architecture](docs/collision-avoidance/architecture.md)
specifies asynchronous preflight, a parallel collision worker, finite movement
permissions, whole-body stopping envelopes and a protective-stop latch that
requires controller Stop before a new Start. The C++ simulation implementation
and its current limits are recorded in the [implementation status](docs/collision-avoidance/implementation.md).

The [3D data specification](docs/collision-avoidance/data-specification.md)
documents a pheno-inspired C-arm adapted to this repository's 75/100 cm planar
arm, plus a fixed patient table. [Editable parameters](data/collision/reference/parameters.json),
frame-local and assembled OBJ models, scene JSON, and an
[offline interactive viewer](data/collision/reference/generated/viewer.html)
are provided. Open the viewer HTML in a browser to inspect poses.

The simulator now runs predictive collision avoidance. All new housing/table
dimensions, stopping values and 3D joint-frame conventions are explicitly
synthetic; the implementation and reference bundle cannot authorize physical motion.

```sh
python3 tools/generate_collision_reference.py --check
python3 tests/test_collision_reference.py
```

See the [verification record](docs/collision-avoidance/verification.md) for completed
checks and the distinction from future runtime/hardware acceptance tests.

## Design implementation completion

Run CTest with port 8082 free. The `runtime_avoidance` test (requires Python 3)
launches the actual `pcan_demo`, verifies repeated permitted movement, stops on
approach to the fixed pedestal with positive clearance, verifies the latched
restart block and Stop acknowledgement, and tests lost-input watchdog stopping.

```sh
python3 tests/test_runtime.py --binary build/pcan_demo
```

The required completion checklists are [architecture section 17.5](docs/collision-avoidance/architecture.md#175-mandatory-design-implementation-completion-gate)
and [the integrated workflow gate](docs/collision-avoidance/architecture.md#1812-mandatory-workflow-acceptance-gate).
The offline viewer and unit tests alone do not satisfy runtime acceptance.
