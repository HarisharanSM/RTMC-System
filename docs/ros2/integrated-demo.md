# Integrated ROS 2 simulator demo

This demo connects topics, the Jog service, a bounded jog action and a URDF/TF
model to the same RTMC simulator. It is simulation-only: joint states describe
commanded CAN feedback, and the model uses provisional synthetic geometry.

## What each ROS interface demonstrates

| Interface | Purpose | Evidence to observe |
| --- | --- | --- |
| `/rtmc/joint_states` | Five joint angles in radians | A1 through A5, with no invented velocity or effort |
| `/rtmc/state` | Complete runtime state in its named legacy units | Coherent CAN sequence, lifecycle and command source |
| `/rtmc/diagnostics` | Connection, freshness and stop state | Explicit acknowledgement requirement after failures |
| `/rtmc/jog` service | One explicit Start, Hold or Stop per request | Input acceptance followed by actual runtime feedback |
| Bounded jog action | A finite direction/duration request with feedback and cancellation | Goal acceptance, progress and terminal result |
| URDF and TF | Link poses computed from joint feedback | Imaging-centre pose agrees with C++ forward kinematics |

Service acceptance means input was dispatched. It does not establish that the
C++ drive granted motion. Likewise, a completed duration does not promise a
particular displacement: limits or collision constraints can restrict motion.
Always observe the returned feedback and the runtime state.

## Build and launch in Codespaces

From the repository root, use the Ubuntu 24.04 environment described in the
[setup guide](README.md). Rerun the dependency installer for the new TF packages:

```bash
bash scripts/setup_ros2_codespaces.sh
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel 2
colcon --log-base "$HOME/.local/share/rtmc-ros2/log" build \
  --base-paths ros2/src \
  --build-base "$HOME/.local/share/rtmc-ros2/build" \
  --install-base "$HOME/.local/share/rtmc-ros2/install"
source "$HOME/.local/share/rtmc-ros2/install/setup.bash"
ros2 launch rtmc_ros2 demo.launch.py \
  simulator_binary:="$PWD/build/pcan_demo" assets:="$PWD" enable_commands:=true
```

The launch starts the simulator, bridge and robot state publisher. Open the
private forwarded port 8082 to watch the existing browser visualization. ROS
owns commands; browser jog controls are disabled. Stop the launch with Ctrl-C.
The demo refuses an occupied port and shuts down its children when the simulator
or bridge exits. It never stops an unrelated listener.

Prepare a second terminal with the same ROS environment and workspace overlay.
Discover and inspect the interfaces:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list -t
ros2 topic echo /rtmc/joint_states --once
ros2 topic echo /rtmc/diagnostics --once
ros2 run tf2_ros tf2_echo rtmc_patient imaging_center
```

Stop `tf2_echo` with Ctrl-C before the next commands. Demonstrate the existing
service client, which explicitly acknowledges, starts, sends Holds and stops:

```bash
ros2 run rtmc_ros2 jog R-up --duration 0.5
```

Inspect diagnostics before explicitly acknowledging for the action example.
Unlike the service example, the action client never sends an initial Stop:

```bash
ros2 run rtmc_ros2 jog --stop
ros2 run rtmc_ros2 jog_for R-up --duration 0.5
ros2 run rtmc_ros2 jog_for R-up --duration 3 --cancel-after 0.4
```

These short X moves are selected for the home pose; A3 is prohibited at exact
home. To demonstrate input loss, use the automated smoke below, which checks
the watchdog and records failure/recovery assertions without requiring timed
manual terminal actions.

For a desktop with a display, install `ros-jazzy-rviz2` and add `use_rviz:=true`
to the launch. RViz is optional and is not required by the headless Codespaces
installer. Its model is a schematic visual proxy; the browser continues to show
the existing runtime scene. RViz transforms retained after a disconnect are
last-known poses, so use diagnostics to establish whether feedback is live.

## Automated demonstration and evidence

Stop the interactive launch first; the smoke owns port 8082 and its processes:

```bash
ros2 run rtmc_ros2 demo_smoke \
  --binary "$PWD/build/pcan_demo" --assets "$PWD" \
  --evidence-dir /tmp/rtmc-integrated-demo
```

Choose an empty evidence directory for each run. The smoke records `demo.log`
and `demo_result.json`, including staged results and timestamped TF comparisons.
It tests service movement, action completion and cancellation, competing goals,
service Stop interruption, and input-loss acknowledgement. The full acceptance
script also retains the existing bridge termination and telemetry bag checks:

```bash
bash scripts/verify_ros2_codespaces.sh
```

## Control rules

Start the simulator in ROS command mode and explicitly enable bridge commands.
The browser becomes a live monitor. A user must explicitly acknowledge startup
or a failure with Stop before requesting motion. Neither an action goal nor a
reconnection acknowledges a protective latch.

Only one motion owner may be active. A competing Start or Hold cannot take over
an action. An explicit service Stop remains available to interrupt motion.
Normal action completion or cancellation stops its own healthy session. On
transport uncertainty, stale feedback or a protective stop, it stops generating
input and reports failure without issuing an acknowledgement. The existing C++
watchdog handles missing input. No uncertain command is retried.

## Validation scope

The native integrated smoke must run on ROS 2 Jazzy after building all packages.
It must exercise discovery, service motion, action completion/cancellation,
competing requests, lost input and TF feedback. Run actual-runtime tests
serially because they own port 8082. The cloud verifier retains each phase's
status and returns failure if any phase fails.

Local XML, C++ transform and policy tests are complementary checks, not proof
that ROS interface generation, DDS discovery or RViz has executed successfully.
See [validation.md](validation.md) for the recorded scope and results.
