# Kinematic Model — Problem Statement and Solution

## 1. Design intent

The positioner is a 2-link planar arm rotating about a common Z axis:

- **A1** — base axle.
- **A2** — mounted on A1's hand end; its own hand end carries the C-arm.
- The combination of A1 and A2 yields the end-effector (EOF) position **(X, Y)**.
- **A3** — Z-axis yaw compensation at the link-2 endpoint, parallel to the A1/A2 axes: `A3=-(A1+A2)`.
- **A4 / A5** — LAO/RAO and CRAN/CAUD, about the moving imaging center.

Revision 2 fixes the patient head at `(0,0,1.20)` m. X/Y are always offsets from
that initial head frame; +X points toward the feet, Y is transverse, Z is up.
A1/A2 planar geometry and the closed-home numeric coordinates are unchanged.
A3 removes link heading from the support and C-arm orientation, so changing X/Y
never rotates the requested translation axes. LAO/CRAN keep the current imaging
center fixed; at X=Y=0 this is the head. See the detailed five-axis contract and
runtime completion gate in [the avoidance architecture](collision-avoidance/architecture.md#17-five-axis-head-reference-and-live-runtime-completion).

Every 50 ms, while a UI button is held, the backend receives an input. X or Y is
incremented in the direction commanded, **bounded by what A1 and A2 can actually
reach given their rotation and link-size limits**. World **(X, Y) = (0, 0) is
defined as the fully-closed pose** of A1 and A2, with physical limits included.

## 2. Problems found in the previous implementation

The IK formula itself was the correct textbook 2R solution (it round-trips
through forward kinematics to ~1e-14). The model *around* it did not implement
the intent above.

### P1 — (0, 0) was not the closed pose

Fully closed means A2 = 180° and an end-effector reach of `|L2 − L1|` = 25 cm.
The base was at `(50, 50)`, which is 70.71 cm from the world origin, so:

```
IK(0,0) -> A1 = -221.6 deg, A2 = 135.1 deg     (closed requires A2 = 180)
```

Home was a mid-fold pose, and A1 fell outside any physical axle range.

### P2 — Reachability did not limit motion; a hardcoded box did

Motion was clamped to a fixed envelope `X∈[0,300], Y∈[-25,25]`, while
reachability was handled separately *inside* IK by silently rescaling the target
onto the reach annulus. The two disagreed:

- only **72.3 %** of the commanded box was reachable;
- max reachable X at Y = 25 was **223.2 cm** while the box allowed **300**;
- the rescale was never written back, so the commanded pose and the pose the
  axles actually reach diverged by up to **79.9 cm**.

Because the rescale is radial, a pure-X command also produced Y motion. And
because `m_CurrentPosition` kept counting past the physical limit, reversing
direction at the end of travel produced roughly **3.9 s of dead joystick** while
the stored position walked back into the reachable region.

### P3 — A1 and A2 had no joint limits at all

Only A3/A4 were clamped. Across the commanded box A1 spanned
**[−270°, −5.7°]** and was written to the bus unclamped.

### P4 — The speed limiter used a second, contradictory kinematic model

`CalculateNextPosition` computed `L1*cos(t) + L2*cos(2t)`, which assumes A2 is
*absolute* and equal to `2·A1`, whereas the IK returns A2 *relative* to link 1.
That expression is also an absolute position (~174.8 cm) that was multiplied by
`deltaTime` and added as an increment, so the "limiting" branch produced
**174.8 cm/s versus 20 cm/s** for the unlimited branch — 8.7× faster than the
path it was supposed to constrain. `fVpeak = sqrt(ACC_RATE * fThetaTotal)` also
passed the *threshold* rather than the actual move distance, making it the
constant 60.0 every time, and applied the triangular-profile formula in the
branch taken for long (trapezoidal) moves.

### P5 — NaN risk at the reach boundary

`sqrt(1 - temp*temp)` had no clamp of `temp` to [−1, 1]. The rescale code set the
distance *exactly* to a reach boundary where `temp` is exactly ±1; one float ULP
past that yields NaN, which propagates into the CAN position frames.

### P6 — Unit and precision mixing

`updatedPos.X += signal.x` added a unitless ±1 to a **cm** quantity while
`updatedPos.LAO += signal.LAO` added the same ±1 to a **degree** quantity. X
therefore moved at 20 cm/s and LAO at 20 deg/s, neither related to the
`MAX_SPEED = 60` actually sent over `SetSpeed`. Positions were `double` while all
intermediate kinematic maths was `float`.

## 3. Solution

### S1 — Home the command frame on the closed pose

The base offset is *derived* from the requirement rather than guessed. At the
closed pose (A2 = 180°) the end effector sits at `|L2 − L1|` from the base along
`A1 + 180°`. Choosing `A1_home = 180°` so that the arm extends toward +X gives:

```
BASE = (-(L2 - L1), 0) = (-25, 0)
```

Verified: `IK(0,0)` → **A1 = −180.000000°, A2 = 180.000000°**, and FK maps that
back to exactly `(0, 0)`. Full extension (A1 = 0°, A2 = 0°) lands at **X = 150**.
The base sits behind the whole travel range, so the arm never has to fold through
its own base.

### S2 — Reachability and joint limits are the authoritative gate

`CalculateInverseKinematics` no longer rescales anything. It returns an
`eKinematicStatus` and never silently retargets:

| status | meaning |
|---|---|
| `Ok` | pose is reachable and within all joint limits |
| `OutOfReach` | target further than `L1 + L2` |
| `TooClose` | target inside the folded dead-zone `|L2 − L1|` |
| `JointLimit` | solution exists but A1/A2/A3/A4 exceed axle travel |
| `OutsideEnvelope` | target outside the declared safety envelope |
| `RateLimited` | step shortened to respect the joint speed profile |

`CalculateNextPosition` computes the requested step, then finds by bisection the
largest fraction of that step which satisfies **envelope ∧ reach ∧ joint limits ∧
joint rate**. It returns both the accepted pose and its axle solution, so the
caller commits a pose that is guaranteed consistent with the angles being sent to
the bus. `m_CurrentPosition` can no longer diverge from the physical arm.

### S3 — Explicit joint limits

Assumed machine limits, exposed as named constants for replacement with real
values:

```
A1 in [-180, +10] deg      A2 in [0, 180] deg
A3 in [-180, +180] deg     A4 in [-180, +180] deg     A5 in [-180, +180] deg
```

A1 = −180° and A2 = 180° are the closed pose; A1 = A2 = 0° is full extension.
The `+10°` on A1 covers the upper-corner geometry (measured maximum 7.712°).

### S4 — One consistent kinematic model, one motion profile

The contradictory FK expression is gone. `CalculateForwardKinematics` is the
exact inverse of the IK and uses the same convention (**A2 is relative to link
1**), which is asserted by a round-trip test.

Motion is time-integrated from explicit, dimensioned speeds:

```
dX, dY      = signal * MAX_LINEAR_SPEED_CMPS  * dt      (cm)
dLAO, dCRAN = signal * MAX_ANGULAR_SPEED_DPS  * dt      (deg)
```

A trapezoidal profile ramps the per-axle joint speed from 0 to
`MAX_JOINT_SPEED_DPS` at `JOINT_ACCEL_DPSS`, giving a per-tick joint budget of
`speed * dt`. The profile resets on stop, error and emergency stop, so
`JOINT_ACCEL_DPSS` is now actually meaningful.

### S5 — Numerical robustness, and tolerance that cannot cause motion

`temp` is clamped to [−1, 1] before `sqrt` and all kinematic maths is `double`.

Limit comparisons need a tolerance, because the home pose sits exactly on two
joint limits and the inner reach circle at once and would otherwise fail to
validate. But that tolerance is applied asymmetrically:

- the pose the system **already holds** is validated with `GEOM_EPS`;
- a **candidate step** is validated with zero tolerance.

The distinction matters. The first implementation of this fix used `GEOM_EPS`
for both, and the test suite caught the consequence: with an axis pinned at a
limit, each tick found `GEOM_EPS` of headroom, the commit clamp snapped the
pinned axis back onto the exact limit, and the *next* tick found the same
headroom again. Holding a diagonal against the Y limit made X creep by exactly
1e-6 cm per tick without bound. Slack may accept a pose; it must never authorise
a step. `KIN-24` soaks 40 000 ticks against saturated limits as a regression
guard.

### S6 — Coordinated motion at a limit

When a multi-axis command drives one axis into a limit, the whole move stops
rather than continuing along the remaining free axes. Direction is preserved, so
the arm never silently changes the commanded heading to slide along a wall —
more predictable behaviour for a positioner working near a patient. `KIN-22`
pins this down. (The current UI cannot produce a multi-axis command anyway: the
CAN mocker emits a one-hot button bitmask.)

### S7 — Consequence worth knowing

Near the fold singularity, Cartesian motion is expensive in joint terms: at
(0, 0), **1 cm of X travel demands 59.1° of joint rotation**, against a 3°/tick
budget. The rate limiter therefore permits only a very small step at home and
opens up as the arm unfolds. This is physically correct behaviour for a folded
2R arm rather than a defect, but it means the system creeps out of the home pose.
A dedicated park/unfold routine that moves in joint space is the usual remedy and
is not implemented here.
