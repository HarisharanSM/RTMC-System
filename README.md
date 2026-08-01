# RTMC-System

**Real-Time Motion Control System Simulator for Precision Imaging Devices**

RTMC-System is a C++17 simulator for a two-axis motion-control stack used in precision imaging equipment (the axis naming — `LAO/RAO`, `CRAN/CAUD`, `X/Y` — mirrors the positioning conventions of C-arm / angiography-style imaging gantries). It models the full signal path from operator input to actuator command — joystick input → CAN bus transport → drive kinematics — without requiring any physical PCAN hardware or motor hardware. A browser-based dual-joystick dashboard drives the pipeline over HTTP, standing in for a real hardware joystick and CAN interface.

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

Everything below the dashboard is simulated: the "PCAN" layer mimics the real PCANBasic SDK's types and constants but stubs the actual hardware calls with console logging, so the whole pipeline runs on a plain Linux machine with no CAN adapter attached.

## Architecture

The system is built around three interfaces (`includes/iSystemController.h`, `iPCANController.h`, `iDrive.h`) that decouple orchestration from implementation, so the simulated CAN/drive layers could be swapped for real hardware without touching the composition logic in `main.cpp`.

| Module | Responsibility |
|---|---|
| `includes/` | Shared interfaces (`iSystemController`, `iPCANController`, `iDrive`), simulated PCANBasic types (`PCANTypes.h`), and common data structures (`commonDrive.h`: `drivePosition`, `joystickSignal`, `AxelPostion`) |
| `PCAN/` | Simulated CAN transport layer — `cPCANSender`/`cPCANReceiver` (stubbed I/O), `cPCANController` (bus orchestrator with a pub/sub subscription registry keyed by CAN message ID), `cCANDriveHandler` (decodes raw CAN frames into `joystickSignal`) |
| `SystemController/` | `cControlManager` — the composition root; wires the CAN controller and drive together and routes `DRIVE_MSG`/`START_DRIVE_MSG`/`STOP_DRIVE_MSG` to the appropriate drive calls |
| `CANMocker/` | `cCANMocker` — a minimal HTTP server (raw POSIX sockets, port 8082) that translates GET requests from the web UI into `TPCANMsg` CAN frames and injects them into the bus |
| `drive/` | Motion subsystem — `cDrive` (façade), `cDriveController` (stateful engine: e-stop, error handling, current position), `cDriveCalculator` (trapezoidal velocity-profile motion integration, workspace limit clamping, and 2-link planar-arm inverse kinematics) |
| `ui/` | `index.html` — a self-contained dual-joystick dashboard (left pad: LAO/RAO & CRAN/CAUD, right pad: X/Y) that polls the CAN mocker over HTTP while a button is held |

## Building

Requirements: a C++17 compiler, CMake ≥ 3.10, and a POSIX/Linux environment (the CAN mocker uses raw BSD sockets).

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

This produces the `pcan_demo` executable.

## Running

```bash
./build/pcan_demo
```

On startup, `pcan_demo`:
1. Initializes the simulated PCAN controller and drive subsystem via `cControlManager`.
2. Starts `cCANMocker`, which listens on **port 8082** for joystick commands from the UI.
3. Launches `python3 -m http.server 8000` to serve `ui/index.html`, so `python3` must be on `PATH`.

Then open `http://localhost:8000` in a browser and use the on-screen joystick buttons to drive the simulated motion pipeline; system activity is logged to stdout in place of real CAN traffic.

> **Note:** the UI serving path in `main.cpp` and the UI's backend URL derivation (it rewrites `-8000.` to `-8082.` in the page URL) are currently hardcoded for a specific devcontainer/Codespaces-style setup. Running outside that environment may require adjusting the static file path in `main.cpp` and the mocker URL logic in `ui/index.html`.

## Status

This is a simulator/prototype: there is no dependency on the real PCANBasic SDK or physical motor hardware, and several pieces (e.g. `iSystemController::ProcessUiCommand`) are stubs. It's intended for exercising and demonstrating the motion-control architecture end-to-end in software.
