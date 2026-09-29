# ROS 2 integration contract (revision 1)

This optional adapter exposes the existing C++ simulator to ROS 2 Jazzy on
Ubuntu 24.04. It does not provide physical encoder feedback or physical collision
protection. The C++ drive remains the sole motion owner; ROS does not compute
joint targets, issue collision permits, or change generated geometry.

## Runtime boundary

`pcan_demo --command-source browser|ros2` selects one command source at startup
(default `browser`). The HTTP server reports `command_source` in `/state`.
Existing requests without `source` mean `browser`. The bridge sends
`POST /command?source=ros2&btn=<direction>&cmd=start|stop` and uses an empty
`cmd` for Hold. A request from the other source gets HTTP 409 with
`accepted:false`, without allocating a session or injecting a CAN frame. All
commands, including Stop, obey the selected source. This is input arbitration,
not authentication. The listener remains loopback-only. In ROS mode the browser
shows live feedback but disables its jog controls with an explanatory message.

The ten direction names are `R-up`, `R-down`, `R-left`, `R-right`, `L-up`,
`L-down`, `L-left`, `L-right`, `A3-left`, `A3-right`. An HTTP acceptance means
input was dispatched, not that motion was authorized. CAN protocol revision 4,
input/session sequencing, independent stop monitoring, collision preflight and
protective-stop acknowledgement remain authoritative.

## ROS packages and interfaces

The workspace is `ros2/`, with packages under `ros2/src/`:

- `rtmc_interfaces`: `srv/Jog.srv` with request fields
  `builtin_interfaces/Time stamp`, `string client_id`, `uint64 sequence`,
  `string command` (`start`, `hold`, `stop`), and `string direction`.
  Response: `bool accepted`, `string message`, `uint32 input_sequence`,
  `uint32 session`.
- `rtmc_ros2`: Python adapter, `bridge` executable, `jog` example executable,
  `bridge.launch.py`, pure-Python policy tests and ROS runtime validation.
- `/rtmc/joint_states`: `sensor_msgs/msg/JointState`, names `A1` through `A5`,
  radians converted from coherent `axles_deg`; leave velocity/effort empty
  because the runtime does not provide per-joint measured values.
- `/rtmc/state`: `std_msgs/msg/String` containing the unchanged HTTP JSON,
  including explicit legacy unit field names and simulation provenance.
- `/rtmc/diagnostics`: `diagnostic_msgs/msg/DiagnosticArray`, connection,
  feedback freshness, lifecycle and protective-stop reason.
- `/rtmc/jog`: `rtmc_interfaces/srv/Jog` (volatile service requests).

## Bridge policy

Use one single-threaded executor and bounded HTTP requests to loopback; no
background Hold generation and no automatic HTTP command retries. A single
bridge process must own the control connection (use a local process lock).
Telemetry-only operation is permitted in either runtime mode; control requires
an explicit `enable_commands:=true` and runtime `command_source=ros2`.

Publish joint states only for valid, finite, coherent five-axis CAN feedback
with age at most 250 ms. Publish diagnostics for invalid/stale/disconnected
feedback; do not make old positions appear fresh. Repeated pose sequence is
valid for a stationary pose because the runtime refreshes CAN feedback.
ROS timestamps describe observation time, not physical acquisition time.

Commands have strictly increasing sequence numbers per client and wall-clock
timestamps no older than 150 ms (at most 50 ms into the future). Use wall and
monotonic clocks for input validity, regardless of `/clock`; reject simulated
time for commanding. Bound remembered clients and reject excess clients rather
than evicting sequence history. Retain sequence history across Stop. Only the
client that started a jog may Hold it, and Hold must retain the same direction.
Stop may be issued by any valid client as an explicit acknowledgement. Require
Stop before a new Start, including after reconnect, a rejected/uncertain HTTP
command, stale feedback, bridge restart, or protective stop. Never send Stop
automatically on expiry or disconnect: that would acknowledge the protective
latch. Stop remains available when pose feedback is stale if transport works.

Each accepted Hold forwards exactly one command. Clients should send at 20 Hz;
loss of input leaves the existing C++ 150 ms renewal watchdog to stop motion.
The bridge also expires its local active input after 150 ms, refusing late
Holds until explicit Stop and fresh Start. Network/cloud scheduling may cause
nuisance stops; these are simulation checks, not hard real-time guarantees.

## Verification boundary

`tests/test_command_source.py` checks C++/browser source arbitration.
`tests/test_ros2_adapter_runtime.py` exercises the actual bridge HTTP transport
and policy against the simulator without ROS middleware. Pure policy tests are
under `ros2/src/rtmc_ros2/test`. The native ROS smoke test and
`scripts/verify_ros2_codespaces.sh` check the full Jazzy integration.
Runtime tests own port 8082 and must never command an unrelated listener.

Cloud validation must distinguish a fresh installation, installer rerun,
workspace build, discovery, motion, failure handling, browser observation and
bag recording/replay. Replay telemetry only, with commands disabled. Existing
kinematics/collision/runtime and generated-reference checks remain mandatory;
record pre-existing failures independently rather than weakening assertions.
