# Static 3D reference dataset

Revision 5, 2026-09-16. Read with [the avoidance architecture](architecture.md).

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
| `generated/frames/column.obj` | Column and A3 stator geometry at the physical link-2 endpoint |
| `generated/frames/boom.obj` | Compensated A3 rotor, upper boom and A4 bearing geometry |
| `generated/frames/a4_carrier.obj` | Synthetic remote-centre carrier and A5 bearing geometry |
| `generated/frames/a5_carrier.obj` | Reserved downstream carrier frame |
| `generated/frames/carm.obj` | Compound C-arm sectors, detector and source in isocenter coordinates |
| `generated/scene_data.inc` | Generated typed C++ body and pair-policy records used by the runtime |
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

World X/Y report imaging-centre displacement from the initial patient head and world Z points up. The legacy planar calculation frame K retains base (-0.25,0), but `T_W_K` translates it by (-1.15,0) m so the physical column can remain headward while the imaging centre is at X/Y=(0,0). The joint-2 angle is relative to link 1. Fixed table's longitudinal axis is world X. The prototype has no Z-translation axis.

With `B=(-0.25,0)`, lengths L1=0.75 and L2=1.00, and q in radians:

```text
E_K = B_K + [L1*cos(q1), L1*sin(q1)]
M_K = E_K + [L2*cos(q1+q2), L2*sin(q1+q2)]
E_W = E_K + [-1.15,0]
M_W = M_K + [-1.15,0]
T_W_link1   = Trans(B_K.x-1.15, B_K.y, 0.10) * Rz(q1)
T_W_link2   = Trans(E_W.x, E_W.y, 0.28) * Rz(q1+q2)
q3 = -(q1+q2)
T_W_column  = Trans(M_W.x, M_W.y, 0.37) * Rz(q1+q2)
T_W_boom    = Trans(M_W.x, M_W.y, 1.20) * Rz(q1+q2+q3)
I_W         = T_W_boom * [1.15,0,0]
T_W_carm    = Trans(I_W) * Rz(q1+q2+q3) * Rx(q4) * Ry(q5)
```

The last two rotation axes, carrier construction and all Z offsets are assumptions. Do not infer actual LAO/CRAN sign, order or pivot from this model. The column follows A1+A2; A3 compensates the boom heading; the synthetic carrier rotates the C-arm about the imaging centre. Measurements must replace that interface before physically meaningful self-collision checks.

Each box has `center_m`, positive full `size_m` and X/Y/Z local rotations relative to its frame. Its local transform is `Trans(center) * Rx * Ry * Rz`. Each body has a stable unique ID, rigid-body ID, mobility and `collision_enabled=true`.

## Dimension and provenance register

| Item | Value | Provenance |
| --- | --- | --- |
| Link center-to-center lengths | 0.75 m / 1.00 m | RTMC geometry constants |
| Base planar position | K=(-0.25,0) m; world=(-1.40,0) m | Existing solve plus explicit -1.15 m registration |
| A1 / A2 travel | [-180,10]° / [0,180]° | Existing assumed software limits, not measured stops |
| A3 / A4 / A5 travel | [-180,180]° / [-180,180]° / [-90,90]° | Assumed software limits; A5 revised for CRAN/CAUD |
| EOF command envelope | X=[0,1.5], Y=[-0.25,0.25] m | RTMC software envelope |
| Reference SID | 1.30 m | Published ARTIS pheno maximum; marker-to-marker distance in synthetic C-arm |
| Reference usable clearance | 0.955 m | Published ARTIS pheno value; matched by synthetic housing faces |
| Link 1 / link 2 center heights | 0.10 / 0.28 m | Synthetic under-table construction supporting the head-side mount |
| Isocenter height | 1.20 m | Synthetic fixed height |
| Column / upper boom | 0.18 m square × 0.83 m; 0.31 m × 0.18 m square | Synthetic drawing-based proxies |
| Base enclosure | 0.55 × 0.60 × 0.25 m | Synthetic |
| Link 1 / link 2 cross-sections | 0.20 × 0.16 / 0.16 × 0.14 m | Synthetic rectangular proxies |
| C-arm arc | Inner radius 0.70 m; outer 0.84 m; Y-depth 0.22 m | Synthetic XZ half-annulus, 8 conservative box sectors |
| Detector housing | 0.48 × 0.42 × 0.245 m | Synthetic exterior; not detector active field |
| Source housing | 0.38 × 0.40 × 0.285 m | Synthetic exterior |
| Fixed table top | 2.20 × 0.55 × 0.10 m; center (0.90,0,0.90) m | Synthetic installation and table |
| Table pedestal | 0.40 × 0.45 × 0.75 m at X=1.70 m | Synthetic |
| Table base | 0.90 × 0.80 × 0.10 m | Synthetic |
| Mattress | 2.00 × 0.53 × 0.05 m | Synthetic |
| Patient test box | Torso 1.68 × 0.50 × 0.30 m centered at (0.96,0,1.15); head 0.24 m cube at (0,0,1.20) | Artificial test volume; not human clearance coverage |
| Floor extent | 6.00 × 5.00 × 0.10 m, top at Z=0 | Synthetic visual/obstacle fixture; no complete room model |
| Pair clearance margin | 0.010 m | One nominal full-speed linear step; illustrative simulation value only |

Reference source: [Siemens ARTIS pheno technical specifications](https://www.siemens-healthineers.com/angio/artis-interventional-angiography-systems/artis-pheno), consulted 2026-09-14. This source does not provide the synthetic housing or table dimensions in this dataset. The manufacturer's multi-tilt table is not modeled; the user-requested fixed table has no degrees of freedom.

Source focal marker is at local Z=-0.65 m and detector image marker at +0.65 m. They are metadata points, not collision surfaces. Source top is -0.62+0.285/2=-0.4775 m; detector bottom is 0.60-0.245/2=+0.4775 m, giving 0.955 m nominal face separation. These relationships only establish consistency of the synthetic model.

## Conservative C-arm representation

The analytic reference is an annular sector in local X/Z, 90° to 270°, extruded along Y. Its opening faces +X toward the table. For a segment midpoint angle theta and angular half-width h:

```text
radial_min = inner_radius * cos(h)
radial_max = outer_radius
tangent_min/max = +/-outer_radius * sin(h)
```

These extrema define an oriented box rotated by -theta about Y. It encloses the full analytic sector rather than a chord that cuts through it. Neighboring boxes may overlap. Unit tests sample radial/angular/depth points as a regression check; the formula provides the enclosure argument. This proves enclosure of the synthetic half-annulus only. No bound relating it to a physical product is known.

## Collision pair policy

All collision-enabled robot bodies must be tested against table, patient exclusion volume, floor and other scene obstacles; robot inter-body pairs remain enabled. Skip internal primitives with the same rigid-body ID. Eight explicit synthetic bearing/interface pair exclusions are generated into both scene metadata and typed C++ data. They are an implementation limitation, not physical self-collision evidence. Measured geometry still requires reviewed local allowed-contact masks.

The runtime pair margin must account for both bodies' geometry error, registration, encoder/tracking bounds, interpolation/numerical errors and any deformation reserve. Timing motion belongs in the reachable tube; avoid counting it twice in a static margin. Unknown uncertainty cannot be interpreted as zero. Mesh vertices do not already contain the illustrative 10 mm margin; apply it once per pair according to the runtime contract.

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
Home I=(0,0) therefore places the imaging pivot at the head center rather than
at an arbitrary table-side offset. Both references remain visible in the viewer
after translation: the head stays fixed and the imaging center moves.

The generated viewer remains an offline inspection artifact. The running
`pcan_demo` loads `parameters.json` and generated `scene.json` into the canvas
inside `ui/index.html`; there is no iframe or separate live viewer. The canvas
renders only complete drive-originated CAN feedback
`axles_deg=[A1,A2,A3,A4,A5]` from `GET /state`. Loss or stale feedback retains
the last complete pose, inhibits Start and never extrapolates motion.
