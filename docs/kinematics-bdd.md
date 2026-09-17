# Use Cases and BDD Scenarios — Drive Kinematics

Executable form: `tests/test_kinematics.cpp`. Scenario IDs below match the IDs
printed by the test binary.

Fixture constants: `L1 = 75 cm`, `L2 = 100 cm`, `BASE = (-25, 0) cm`,
`tick = 50 ms`, `MAX_LINEAR_SPEED = 20 cm/s`, `MAX_ANGULAR_SPEED = 60 deg/s`,
`MAX_JOINT_SPEED = 60 deg/s`, `JOINT_ACCEL = 120 deg/s²`,
`A1 ∈ [-180, 10]°`, `A2 ∈ [0, 180]°`, `A3, A4 ∈ [-180, 180]°`,
`A5 ∈ [-90, 90]°`.

---

## UC-1 — Home pose is the fully-closed configuration

**As** a service engineer **I want** world (0,0) to correspond to A1 and A2 fully
closed **so that** the command frame is anchored to a physically identifiable
pose.

### KIN-01 — Home maps to the closed pose
- **Given** the arm is commanded to world position (0, 0)
- **When** inverse kinematics is solved
- **Then** the status is `Ok`
- **And** A2 = 180° ± 1e-6 (fully closed)
- **And** A1 = −180° ± 1e-6

### KIN-02 — Full extension is the far end of travel
- **Given** the arm is commanded to world position (150, 0)
- **When** inverse kinematics is solved
- **Then** the status is `Ok`
- **And** A1 = 0° ± 1e-6 **and** A2 = 0° ± 1e-6

---

## UC-2 — Forward and inverse kinematics agree

**As** a control engineer **I want** a single angle convention **so that** the
stored Cartesian position and the angles sent to the bus describe the same pose.

### KIN-03 — IK/FK round-trip
- **Given** any reachable target from the set
  {(10,0), (40,10), (75,-15), (120,20), (149,0), (60,-25), (100,25)}
- **When** IK is solved and the resulting angles are fed back through FK
- **Then** the recovered position matches the target within 1e-9 cm on X and Y

### KIN-04 — A2 is documented as relative to link 1
- **Given** angles A1 = −180°, A2 = 180°
- **When** forward kinematics is evaluated
- **Then** the end effector is at (0, 0) ± 1e-9
  *(the absolute-angle interpretation would instead yield (−100, 0))*

---

## UC-3 — Reach limits, not a hardcoded box, bound the motion

**As** an operator **I want** the arm to stop where it physically cannot go
further **so that** the reported position never diverges from the real pose.

### KIN-05 — Beyond maximum reach is rejected, not silently retargeted
- **Given** a target at (200, 0), which is 225 cm from the base
- **When** inverse kinematics is solved
- **Then** the status is `OutOfReach`
- **And** no rescaled position is written back to the caller's target

### KIN-06 — Inside the folded dead zone is rejected
- **Given** a target at (−10, 0), which is 15 cm from the base
- **When** inverse kinematics is solved
- **Then** the status is `TooClose`

### KIN-07 — Joint limits are enforced independently of reach
- **Given** a target at (62.5, 151.55), which is within reach but requires
  A1 ≈ +60°
- **When** inverse kinematics is solved
- **Then** the status is `JointLimit`

### KIN-08 — Declared envelope is consistent with reach
- **Given** the envelope corners (0,0), (150,0) and the Y extremes at mid travel
- **When** each is solved
- **Then** every corner on the X axis is `Ok`
- **And** the maximum reachable X at Y = 0 is 150 ± 0.01 cm

---

## UC-4 — Commanded position never diverges from the axle solution

**As** a safety reviewer **I want** the stored position to always be the pose the
axles actually hold **so that** reversing direction responds immediately.

### KIN-09 — Held button drives to the limit and stops there
- **Given** the arm starts at home (0, 0) with the drive started
- **When** "+X" is held for 400 ticks (20 s)
- **Then** the final X is ≤ 150 exactly, with no epsilon overshoot
- **And** FK of the last commanded angles equals the stored position within
  1e-6 cm
- **And** the final status reports a constraint rather than `Ok`

### KIN-10 — Reversing at the limit moves on the very next tick
- **Given** the arm has been driven against the +X limit
- **When** "−X" is applied
- **Then** the stored X strictly decreases on the very first tick (no dead zone)
- **And** more than 1 cm is recovered within 10 ticks (0.5 s), against the ~3.9 s
  dead zone the silent-retarget bug produced

### KIN-11 — Every committed pose is valid
- **Given** a 400-tick "+X" run followed by a 400-tick "−X" run
- **When** each committed pose is re-validated
- **Then** every one solves `Ok` and lies inside the envelope, the reach annulus
  and all joint limits
- **And** no coordinate is ever NaN
- **And** on every tick the stored position equals FK of the commanded angles
  within 1e-6 cm *(this diverged by up to 79.9 cm before the fix)*

### KIN-22 — A diagonal command keeps its direction and does not creep
- **Given** the arm at (75, 0) with "+X" and "+Y" held together
- **When** 100 ticks are commanded, long past the point Y saturates
- **Then** while both axes are free the move stays on the commanded 45° line
- **And** Y saturates exactly at +25 cm
- **And** once Y saturates the move halts rather than sliding along the wall —
  X is identical at tick 50 and tick 100

### KIN-23 — The Y envelope limit is enforced exactly
- **Given** the arm at (75, 0) with "+Y" held for 200 ticks
- **Then** Y never exceeds +25 cm
- **And** X is untouched by pure-Y motion

### KIN-24 — A saturated axis does not drift over a long hold
- **Given** the arm held against the +X limit, and against a saturated diagonal
- **When** 20 000 further ticks are commanded on each (~17 minutes of held button)
- **Then** the position is bit-identical to the moment it saturated
- **And** all 40 000 soak poses stay valid and finite
  *(regression guard: the first cut of this fix crept 1e-6 cm per tick without
  bound, because the validation tolerance was reachable afresh every tick)*

---

## UC-5 — Motion respects the velocity and acceleration profile

**As** a control engineer **I want** joint motion bounded by the profile **so
that** commanded speeds are physically achievable.

### KIN-12 — Per-tick joint step never exceeds the profile budget
- **Given** the drive is started at a mid-workspace pose (100, 0)
- **When** "+X" is held for 100 ticks
- **Then** on every tick, max(|ΔA1|, |ΔA2|, |ΔA3|, |ΔA4|, |ΔA5|) ≤ profile speed × dt
  + 1e-6

### KIN-13 — Speed ramps rather than stepping to maximum
- **Given** a freshly reset motion profile
- **When** ten consecutive ticks are commanded
- **Then** the profile speed increases monotonically
- **And** it reaches exactly `MAX_JOINT_SPEED` after `MAX_JOINT_SPEED /
  JOINT_ACCEL` seconds (10 ticks) and never exceeds it

### KIN-14 — Stop resets the profile
- **Given** the profile has ramped to maximum
- **When** `StopDrive` is issued
- **Then** the profile speed returns to 0
- **And** the next motion starts from the bottom of the ramp again

### KIN-15 — The limited path is never faster than the unlimited request
- **Given** any single tick from a held-button run
- **When** the achieved Cartesian step is measured
- **Then** it is ≤ `MAX_LINEAR_SPEED × dt` + 1e-9
  *(regression guard for the old branch, which was 8.7× faster than the
  unconstrained path)*

---

## UC-6 — Angular axes move in their own units

### KIN-16 — LAO/CRAN move at the angular speed, not the linear one
- **Given** the profile is fully ramped at a mid-workspace pose
- **When** "L-up" (CRAN+) is held for one tick
- **Then** ΔCRAN = `MAX_ANGULAR_SPEED × dt` = 3.0° ± 1e-6
- **And** X and Y are unchanged

### KIN-17 — Angular axes clamp at their travel limits
- **Given** LAO is driven positive for 200 ticks
- **Then** LAO saturates at +180° and never exceeds it

### KIN-25 — CRAN/CAUD clamp at ±90°
- **Given** CRAN or CAUD is held beyond its configured travel
- **Then** CRAN saturates exactly at +90° and CAUD at −90°
- **And** neither direction wraps, overshoots or renews numerical creep

---

## UC-7 — Numerical robustness

### KIN-18 — No NaN at the exact reach boundaries
- **Given** targets placed exactly at `|L2 − L1|` and at `L1 + L2` from the base,
  and at 1e-12 either side of each
- **When** IK is solved
- **Then** no returned angle is NaN
- **And** the boundary targets themselves report `Ok`

### KIN-19 — Degenerate joystick input is inert
- **Given** an all-zero joystick signal
- **When** a tick is processed
- **Then** the position is unchanged and the profile is reset

---

## UC-8 — Safety interlocks still dominate

### KIN-20 — Emergency stop blocks motion and zeroes speed
- **Given** the drive is running
- **When** `SetEmgStop` is called and "+X" is then held for 10 ticks
- **Then** the position does not change
- **And** the last speed written to the bus is 0

### KIN-21 — An active fault blocks motion until cleared
- **Given** `SetError(7)` has been called
- **When** "+X" is held for 10 ticks
- **Then** the position does not change
- **And** after `SetError(0)` the arm moves again

## Five-axis patient-head additions

The existing 24 scenarios remain regression requirements. Additional executable
checks in `tests/test_collision.cpp` verify A3 compensation over multiple X/Y
poses, five-axis IK/FK, LAO=A4 and CRAN=A5, stationary head pivot under combined
tilt, and nonzero box-corner sweep under pure A5 rotation. Artifact tests verify
that the head center and home imaging center coincide and that compensated
translations preserve the C-arm orientation.

The running application must also pass `tests/test_runtime.py`: live telemetry,
permit renewal, independent timeout stop, predictive pedestal stop with positive
clearance, latch persistence and Stop/new-Start recovery. This is part of design
implementation completion, not an optional viewer demonstration.

## UC-9 — Independent A3 carrier yaw

### KIN-26 — Exact-home A3 motion is workspace-blocked
- **Given** the closed home pose at X=Y=0 and heading zero
- **When** either A3 direction is requested
- **Then** the move reports `OutsideEnvelope`
- **And** no pose or axle change is committed

### KIN-27 — Pure A3 holds the planar arm
- **Given** a valid interior pose
- **When** A3 is held
- **Then** A1/A2/A4/A5 remain fixed and only A3 changes
- **And** the imaging centre follows the 115 cm carrier arc
- **And** reported heading equals A1+A2+A3

### KIN-28 — Translation retains selected heading
- **Given** a nonzero heading selected by an A3 jog
- **When** X or Y is commanded
- **Then** the imaging centre translates in patient coordinates
- **And** A1/A2 are solved while A3 changes as required to preserve heading
