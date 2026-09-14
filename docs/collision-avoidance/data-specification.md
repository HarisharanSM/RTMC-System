# Static 3D reference dataset

Revision 2, 2026-09-14. Read with [the avoidance architecture](architecture.md).

## Purpose and limitations

This is original synthetic geometry inspired by the general appearance and clearance of ARTIS pheno, adapted to RTMC's five-axis patient-head model. It is not Siemens CAD, a reverse-engineered product, a certified collision envelope or a measured fixed patient table. All physical measurement uncertainties are unknown. The entire bundle is `simulation_only` with `hardware_authorization=false`.

The reference can support scene loading, transform development, broad/narrow-phase integration, regression fixtures and visualization. It cannot prove real-machine clearance. Preview poses have not been certified collision-free; intended joint contacts and support contacts exist, and no reviewed adjacency masks are supplied.

## Files and use

| File under `data/collision/reference/` | Purpose |
| --- | --- |
| `parameters.json` | Editable dimensions, provenance, joint limits and named poses |
| `generated/scene.json` | Frame-local box primitives, IDs, rigid-body grouping and pair policy |
| `generated/frames/world.obj` | Fixed table, base, floor and optional patient test envelope, in world coordinates |
| `generated/frames/link1.obj` | Link 1 and elbow housing in link-1 coordinates |
| `generated/frames/link2.obj` | Link 2 in elbow coordinates |
| `generated/frames/eof_support.obj` | Rear support in EOF support coordinates; rotated by the A3 alignment joint |
| `generated/frames/carm.obj` | Compound C-arm sectors, detector and source in isocenter coordinates |
| `generated/poses/{home,extended,oblique}.json` | Joint angles and world transforms for reproducible snapshots |
| `generated/poses/{home,extended,oblique}.obj` | Assembled world-coordinate snapshots for CAD viewers |
| `generated/viewer.html` | Offline interactive model viewer; orbit/zoom, pose presets and five axle readouts; A3 is automatic |
| `generated/manifest.json` | Source and generated asset SHA-256 integrity records |

OBJ files use meters and Z up, documented in comments because OBJ has no enforced units. Each named object is a closed, outward-triangulated box. A compound assembly has overlapping boxes and is not one manifold solid. Use `scene.json` primitives for collision queries. Do not convert the C-arm into one convex hull: that closes its free opening.

From the repository root:

```sh
python3 tools/generate_collision_reference.py
python3 tools/generate_collision_reference.py --check
python3 tests/test_collision_reference.py
```

Generation requires Python's standard library only and does not modify drive code. Open `generated/viewer.html` directly in a browser; it embeds the scene and needs no network or server. A local static server is optional. The source viewer template is `tools/collision_reference_viewer.html`.

## Coordinate and transform definitions

Use column vectors; matrices are row-major JSON arrays. `p_world = T_world_frame * p_frame`. Angles in the scene and transforms are radians; parameter keys ending `_deg` and preview joint values are explicitly degrees. Dimensions ending `_m` are meters.

World X/Y preserve the existing planar command frame and world Z points up. At closed pose, A1=-180°, A2=180°, EOF X/Y=(0,0). Robot base X/Y=(-0.25,0). The joint-2 angle is relative to link 1. Fixed table's longitudinal axis is world X. The prototype has no Z-translation axis.

With `B=(-0.25,0)`, lengths L1=0.75 and L2=1.00, and q in radians:

```text
E = B + [L1*cos(q1), L1*sin(q1)]
F = E + [L2*cos(q1+q2), L2*sin(q1+q2)]
T_W_link1   = Trans(B.x, B.y, 0.43) * Rz(q1)
T_W_link2   = Trans(E.x, E.y, 0.68) * Rz(q1+q2)
q3 = -(q1+q2)
T_W_support = Trans(F.x, F.y, 0.68) * Rz(q1+q2+q3)
T_W_carm    = Trans(F.x, F.y, 1.20) * Rz(q1+q2+q3) * Ry(q4) * Rx(q5)
```

The last two rotation axes and all Z offsets are assumptions. Do not infer actual LAO/CRAN sign, order or pivot from this model. The support is carried by A3 and has rigid-body ID `alignment`; the C-arm rotates about the imaging center through A4/A5. A3 has an axis parallel to A1/A2, through the link-2 endpoint, not the base's spatial center. Mount/bearing detail is omitted. Measurements must replace that interface before physically meaningful self-collision checks.

Each box has `center_m`, positive full `size_m` and `rotation_x_rad` relative to its frame. Its local transform is `Trans(center) * Rx(rotation_x_rad)`. Each body has a stable unique ID, rigid-body ID, mobility and `collision_enabled=true`.

## Dimension and provenance register

| Item | Value | Provenance |
| --- | --- | --- |
| Link center-to-center lengths | 0.75 m / 1.00 m | RTMC geometry constants |
| Base planar position | (-0.25,0) m | Derived from RTMC closed-pose convention |
| A1 / A2 travel | [-180,10]° / [0,180]° | Existing assumed software limits, not measured stops |
| A3 / A4 / A5 travel | [-180,180]° each | Existing assumed software limits |
| EOF command envelope | X=[0,1.5], Y=[-0.25,0.25] m | RTMC software envelope |
| Reference SID | 1.30 m | Published ARTIS pheno maximum; marker-to-marker distance in synthetic C-arm |
| Reference usable clearance | 0.955 m | Published ARTIS pheno value; matched by synthetic housing faces |
| Link 1 / link 2 center heights | 0.43 / 0.68 m | Synthetic, avoids representing folded links as coplanar solids |
| Isocenter height | 1.20 m | Synthetic fixed height |
| Base enclosure | 0.55 × 0.60 × 0.35 m | Synthetic |
| Link 1 / link 2 cross-sections | 0.20 × 0.16 / 0.16 × 0.14 m | Synthetic rectangular proxies |
| C-arm arc | Inner radius 0.70 m; outer 0.84 m; X-depth 0.22 m | Synthetic half-annulus, 24 conservative box sectors |
| Detector housing | 0.48 × 0.42 × 0.245 m | Synthetic exterior; not detector active field |
| Source housing | 0.38 × 0.40 × 0.285 m | Synthetic exterior |
| Fixed table top | 2.20 × 0.55 × 0.10 m; center (0.90,0,0.90) m | Synthetic installation and table |
| Table pedestal | 0.40 × 0.45 × 0.75 m at X=1.70 m | Synthetic |
| Table base | 0.90 × 0.80 × 0.10 m | Synthetic |
| Mattress | 2.00 × 0.53 × 0.05 m | Synthetic |
| Patient test box | Torso 1.68 × 0.50 × 0.30 m centered at (0.96,0,1.15); head 0.24 m cube at (0,0,1.20) | Artificial test volume; not human clearance coverage |
| Floor extent | 6.00 × 5.00 × 0.10 m, top at Z=0 | Synthetic visual/obstacle fixture; no complete room model |
| Pair clearance margin | 0.020 m | Illustrative simulation value only |

Reference source: [Siemens ARTIS pheno technical specifications](https://www.siemens-healthineers.com/angio/artis-interventional-angiography-systems/artis-pheno), consulted 2026-09-14. This source does not provide the synthetic housing or table dimensions in this dataset. The manufacturer's multi-tilt table is not modeled; the user-requested fixed table has no degrees of freedom.

Source focal marker is at local Z=-0.65 m and detector image marker at +0.65 m. They are metadata points, not collision surfaces. Source top is -0.62+0.285/2=-0.4775 m; detector bottom is 0.60-0.245/2=+0.4775 m, giving 0.955 m nominal face separation. These relationships only establish consistency of the synthetic model.

## Conservative C-arm representation

The analytic reference is an annular sector in local Y/Z, 90° to 270°, extruded along X. Its opening faces +Y. For a segment midpoint angle theta and angular half-width h:

```text
radial_min = inner_radius * cos(h)
radial_max = outer_radius
tangent_min/max = +/-outer_radius * sin(h)
```

These extrema define an oriented box rotated by theta about X. It encloses the full analytic sector rather than a chord that cuts through it. Neighboring boxes may overlap. Unit tests sample radial/angular/depth points as a regression check; the formula provides the enclosure argument. This proves enclosure of the synthetic half-annulus only. No bound relating it to a physical product is known.

## Collision pair policy

All collision-enabled robot bodies must be tested against table, patient exclusion volume, floor and other scene obstacles; robot inter-body pairs remain enabled. Skip internal primitives with the same rigid-body ID. The source, detector and arc sectors share the C-arm rigid-body ID; support is `alignment`, and link 2 is `link2`.

The scene metadata prescribes reviewed local contact masks. The current simulator still excludes entire synthetic interface pairs: link1/link2, link2/alignment, alignment/carm and legacy link2/carm. These exclusions are visible implementation limits and cannot establish physical self-collision protection. Intended bearing connections and permanent floor supports need reviewed local allowed-contact masks. Joint housings in the dataset are simplified and may produce expected overlaps. A consumer must not silently disable all adjacent pairs to make a preview appear clear. Static support contacts should be distinguished from forbidden assembly overlaps at scene validation.

The runtime pair margin must account for both bodies' geometry error, registration, encoder/tracking bounds, interpolation/numerical errors and any deformation reserve. Timing motion belongs in the reachable tube; avoid counting it twice in a static margin. Unknown uncertainty cannot be interpreted as zero. Mesh vertices do not already contain the illustrative 20 mm margin; apply it once per pair according to the runtime contract.

## Asset replacement and dynamic migration

1. Obtain measured CAD or approved conservative solids for the actual robot/table/accessories. Record license, revision, serial/configuration applicability, measurement source and frame definitions.
2. Register geometry to calibrated joint frames, preserving RTMC A2-relative convention. Resolve A3/A4/A5 offsets, axis order, signs and mechanical limits.
3. Verify mesh units, positive volume, no degenerate surfaces, enclosure error and simplification bounds. Supply collision solids separately from decorative meshes.
4. Record measured tracking, calibration, load/deflection and stopping uncertainties. Replace the unvalidated margin and missing brake profile.
5. Review local allowed-contact regions and obstacle coverage, including cables and drapes. Validate known configurations and motion paths against physical measurements.
6. Build a new immutable scene generation and integrity manifest. Keep simulation-only status until the hardware release gates in the architecture pass; flipping a JSON boolean is not approval.

Future tracked obstacles reuse geometry and IDs but obtain a dynamic motion source, uncertainty model and freshness deadline. Change the scene atomically while stopped in version 1. A table that can move manually is not static merely because it has no commanded joints.

## Verification scope

`tests/test_collision_reference.py` checks reference FK, rotated transforms, basic schema/IDs/units, generated hashes, OBJ topology and conservative arc enclosure. The viewer uses the same parameter conventions for visualization. Neither generator nor tests implement GJK, continuous collision, stopping supervision or physical clearance validation. The runtime completion gate in architecture section 17 is exercised by `tests/test_runtime.py`; the remaining hardware AVOID tests remain future acceptance.

## Patient head and live display

The fixed reference head center is H=(0,0,1.20) m. The torso begins at X=0.12 m
and extends toward +X. The head is a distinct collision-enabled static body.
Home F=(0,0) therefore places the imaging pivot at the head center rather than
at an arbitrary table-side offset. Both references remain visible in the viewer
after translation: the head stays fixed and the imaging center moves.

The generated viewer remains an offline inspection artifact by default. When
served by `pcan_demo` at `/viewer?live=1`, it disables manual poses and renders
backend `axles_deg=[A1,A2,A3,A4,A5]` from `GET /state`. The joystick console
embeds that view. Loss of telemetry retains the last pose and displays
Disconnected; it never extrapolates motion or labels cached data as current.
