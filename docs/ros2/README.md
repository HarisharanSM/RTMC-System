# Optional ROS 2 Jazzy adapter in Codespaces

This integration runs the C++ motion-control **simulator** with a headless ROS 2
Jazzy bridge on Ubuntu 24.04. The C++ drive remains the only motion owner.
`/rtmc/joint_states` contains simulated commanded CAN feedback, not physical
encoder measurements; collision avoidance is simulation-only. No RViz or desktop
environment is needed.

## Application use cases

| Use case | What ROS adds to RTMC | Initial implementation |
| --- | --- | --- |
| Engineering monitoring | Standard subscriptions for five-axis CAN feedback and stop reasons | JointState, raw state and diagnostics topics |
| Reproducing observations | Telemetry recording for comparing runs without commanding motion | Telemetry-only rosbag record, decoding and replay validation |
| Scripted integration testing | A typed interface for another ROS process to request bounded jogging | Jog service with timestamps, sequence checks and explicit Stop acknowledgement |
| Component discovery | ROS tools can discover and inspect the simulator adapter in the same Codespace | Node, topic and service discovery |

The bridge wraps the current HTTP/CAN/drive path. The [integrated demo](integrated-demo.md) adds a bounded jog action, URDF/TF
and an optional RViz configuration. Trajectory planning, `ros2_control`, physical
hardware and remote DDS networking remain future use cases.

## Start a Codespace

Open this repository in a GitHub Codespace. Its single Ubuntu 24.04
[devcontainer](../../.devcontainer/devcontainer.json) runs
[`setup_ros2_codespaces.sh`](../../scripts/setup_ros2_codespaces.sh) at creation.
The installer requires `amd64` or `arm64` and passwordless `sudo`; it installs
the official ROS apt-source package, Jazzy ROS base, message and service
generation, Python bridge dependencies, rosbag2 with SQLite3 storage, colcon,
and C++/Python build tools. It is safe to rerun after a partial setup:

```bash
bash scripts/setup_ros2_codespaces.sh
```

The repository setup follows the official
[Jazzy Ubuntu deb instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and the ROS project's
[apt-source package](https://github.com/ros-infrastructure/ros-apt-source).

Open each terminal in the Codespace and prepare the local ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

The Codespace sets those discovery variables by default. They limit ROS peer
discovery to the container. The simulator's HTTP listener is also loopback-only.
Codespaces forwarded ports are private by default; verify port 8082 is **Private**
in the Ports panel. The devcontainer forwards the port but does not override an
existing user's visibility setting. These are environment settings, not
an authentication boundary. Keep the simulator, bridge, and ROS CLI in the same
Codespace. ROS graph discovery (`ros2 node list`, `ros2 topic list`) only shows
which ROS endpoints exist; it does not prove that a jog was accepted or that the
C++ drive authorized motion.

## Build and observe

From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
colcon --log-base "$HOME/.local/share/rtmc-ros2/log" build \
  --base-paths ros2/src \
  --build-base "$HOME/.local/share/rtmc-ros2/build" \
  --install-base "$HOME/.local/share/rtmc-ros2/install"
source "$HOME/.local/share/rtmc-ros2/install/setup.bash"
```

In one terminal, start the simulator in its default browser-control mode:

```bash
./build/pcan_demo
```

In another terminal, after sourcing Jazzy and the colcon `setup.bash`, launch the
bridge with commands **disabled**:

```bash
ros2 launch rtmc_ros2 bridge.launch.py base_url:=http://127.0.0.1:8082
```

Keep that terminal running. In a third prepared terminal:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 topic echo /rtmc/diagnostics
```

The bridge publishes `/rtmc/joint_states` (`sensor_msgs/JointState`, A1–A5 in
radians), `/rtmc/state` (the unmodified HTTP JSON), and `/rtmc/diagnostics`
(`diagnostic_msgs/DiagnosticArray`). Joint states appear only while coherent
five-axis feedback is valid and at most 250 ms old. A stationary pose may repeat
its sequence because the simulator refreshes feedback. ROS timestamps describe
observation time, not physical acquisition time.

## Opt in to ROS commands

Stop the browser-mode simulator. Start it with the explicit command source:

```bash
./build/pcan_demo --command-source ros2
```

Stop the telemetry bridge too (only one bridge process may hold the local
connection), then start the bridge with command handling enabled:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/.local/share/rtmc-ros2/install/setup.bash"
ros2 launch rtmc_ros2 bridge.launch.py \
  base_url:=http://127.0.0.1:8082 enable_commands:=true
```

The browser continues to show live feedback, with jog controls disabled. The
HTTP `/state` field `command_source` reports the selected source. A browser
command in ROS mode receives HTTP 409. ROS command handling is an explicit opt-in
on both processes; ROS graph discovery alone does not enable control.

In a separate prepared terminal, request a half-second X jog:

```bash
ros2 run rtmc_ros2 jog R-up --duration 0.5
```

This explicitly requests **Stop / acknowledge, Start, 20 Hz Hold, Stop**. It
stops sending input on a rejected/uncertain result; it does not automatically
acknowledge a failure. The initial Stop can acknowledge an existing protective
latch, so inspect `/rtmc/diagnostics` before running it. A3 cannot rotate at
exact home because of the existing workspace envelope; translate into the
interior before trying an A3 jog. To acknowledge only, without starting motion:

```bash
ros2 run rtmc_ros2 jog --stop
```

From a third prepared terminal, run the bounded example jog:

```bash
ros2 run rtmc_ros2 jog A3-right --duration 0.5
```

The example sends an explicit Stop acknowledgement, Start, 20 Hz Holds, and
Stop. It operates the simulator only.

Use the `rtmc_interfaces/srv/Jog` service at `/rtmc/jog` (or the package's `jog`
example). A request carries a wall-clock `stamp`, `client_id`, strictly
increasing `sequence`, `command` (`start`, `hold`, `stop`), and `direction`.
Directions are `R-up`, `R-down`, `R-left`, `R-right`, `L-up`, `L-down`,
`L-left`, `L-right`, `A3-left`, and `A3-right`. Keep one client ID and increase
its sequence for every request. While jogging, send a Hold for the same
direction at 20 Hz. Stop explicitly before a new Start, including after a
watchdog stop, reconnect, stale feedback, or uncertain request. A service
acceptance says the input was dispatched; it is not proof of authorized motion.
Check `/rtmc/state`, `/rtmc/diagnostics`, and the simulator's `/state` to observe
the result. The C++ 150 ms permission-renewal watchdog remains authoritative.

Client sequence history is bounded to 32 unique IDs for one bridge lifetime.
The service example uses a new ID per invocation; each accepted action also uses a new ID. If that limit is reached, stop the
bridge, confirm the drive has stopped, then restart it and explicitly acknowledge
before jogging. The bridge never evicts old sequence history to admit delayed
requests. It rejects simulated-clock operation (`use_sim_time=true`); command
timestamps and expiry use wall/monotonic time.

## Record and replay telemetry

With a running bridge, record only the three telemetry topics:

```bash
ros2 bag record -s sqlite3 -o /tmp/rtmc-telemetry \
  /rtmc/joint_states /rtmc/state /rtmc/diagnostics
```

Stop the recording with Ctrl-C. For replay, stop the live bridge and do not
enable ROS commands. Replay publishes telemetry; it cannot reproduce drive
motion or validate the command path:

```bash
ros2 bag play /tmp/rtmc-telemetry
```

## Verify

Stop any interactive simulator first: runtime tests and the ROS smoke test own
loopback port 8082 and refuse an unrelated listener. Run:

```bash
bash scripts/verify_ros2_codespaces.sh
```

The verifier keeps phase logs, colcon results, and smoke artifacts under
`~/rtmc-ros2-evidence/<UTC timestamp>-<process ID>` (or
`RTMC_VERIFY_OUTPUT_DIR`). It runs CMake/CTest, the generated-reference checks,
bridge policy tests, colcon build/test, and a ROS smoke that starts only its own
simulator and bridge. It continues independent checks after a failure and exits
nonzero with a failed-phase summary. A pre-existing CTest failure is reported
as such; the script never turns it into a pass. The smoke covers discovery,
motion, failure handling, and bag record/replay without using RViz.

The contract and exact safety boundary are in
[`interface-contract.md`](interface-contract.md). Cloud results should be
recorded separately in `validation.md`; a local macOS syntax check does not
qualify a Codespace run.
