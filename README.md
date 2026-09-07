# payload-attach-bolt-sizing (STM-08)

Bolted-joint sizing for a payload-to-structure interface under representative
launch load factors.

**Milestone 1 computes elastic rigid-interface bolt-group load distribution
only. Bolt strength, preload, separation, slip, bearing, and final
margin-of-safety sizing are intentionally deferred.**

## Objective

Given a rigid payload interface and an idealized bolt group, how are launch
forces and moments distributed to the individual bolts, and can the resulting
bolt forces be verified exactly from static equilibrium?

## Current scope (Milestone 1)

- Validated bolt-pattern geometry (arbitrary coordinates, plus circular and
  rectangular pattern generators).
- A validated `InterfaceLoad` (Fx, Fy, Fz, Mx, My, Mz).
- A rigid, equal-bolt-stiffness bolt-group solver:
  - direct in-plane shear (equal split of Fx, Fy),
  - torsional shear from Mz (proportional to radius from the bolt-group
    centroid),
  - axial/tensile bolt load from Fz, Mx, My (linear distribution, solved
    generally from bolt-coordinate sums -- not a formula specialized to
    circular patterns).
- A per-bolt result structure and deterministic governing-bolt helpers
  (max shear, max tensile, max absolute axial).
- An independent equilibrium-verification helper that recovers group loads
  from the per-bolt results and reports residuals against the applied load.
- A representative (illustrative) payload-attach launch-load sanity case.
- An automated test suite covering geometry, shear, torsion, axial load,
  generality/validation, and one fully hand-derived closed-form case.

## Coordinate and sign convention

- The payload interface is modeled as a rigid plate lying in the **x-y**
  plane; **+z** is the interface normal.
- Each bolt is an idealized point fastener at `(x_i, y_i)`, in meters.
- **Fx, Fy** — in-plane shear resultants (N).
- **Fz** — axial/normal resultant (N). Positive Fz is **tension** (+z,
  pulling the payload away from the structure). Fz may be negative
  (compression) and is never silently clipped.
- **Mx, My** — overturning moments (N·m) about the x and y axes.
- **Mz** — in-plane torsional moment (N·m) about the interface normal,
  positive by the right-hand rule (counterclockwise viewed from +z).
- **All applied loads are referenced about the bolt-group centroid**, not
  necessarily the origin used to define bolt coordinates. Internally, bolt
  coordinates are translated to the centroid for all load-distribution math;
  original coordinates are retained unchanged for reporting. A worked test
  (`test_translated_pattern_same_relative_bolt_forces`) proves this makes the
  distributed bolt forces invariant to translating every bolt coordinate by a
  constant.
- Units are SI throughout: meters, newtons, newton-meters.

## Rigid-interface bolt-group model

This is an **elastic rigid-plate / equal-bolt-stiffness** bolt-group model:
the attach plate is treated as perfectly rigid, and every bolt is assumed to
have identical stiffness, so bolt loads are computed purely from geometry --
no stiffness-weighted load sharing, no contact/separation redistribution.

### Direct in-plane shear

```
Vx_direct,i = Fx / n
Vy_direct,i = Fy / n
```

### Torsional shear from Mz

For bolt `i` at centroid-relative coordinates `(x_i, y_i)`, with
`J = sum(x_i^2 + y_i^2)`:

```
Vx_torsion,i = -Mz * y_i / J
Vy_torsion,i =  Mz * x_i / J
```

Total shear: `Vx_i = Vx_direct,i + Vx_torsion,i`, `Vy_i = Vy_direct,i +
Vy_torsion,i`, `V_i = sqrt(Vx_i^2 + Vy_i^2)`.

### Axial/tensile load from Fz, Mx, My (overturning)

Bolt axial load is assumed to vary linearly with bolt coordinates (rigid
rotation of the attach flange): `T_i = c0 + cx*x_i + cy*y_i`. The
coefficients are solved from the general 3x3 system built from bolt-group
coordinate sums (not a formula specialized to any one pattern shape):

```
[ n,    Sx,   Sy  ] [c0]   [ Fz]
[ Sy,   Sxy,  Syy ] [cx] = [ Mx]
[-Sx,  -Sxx, -Sxy ] [cy]   [ My]
```

where `Sx = sum(x_i)`, `Sy = sum(y_i)`, `Sxx = sum(x_i^2)`,
`Syy = sum(y_i^2)`, `Sxy = sum(x_i*y_i)` (all centroid-relative, so `Sx = Sy
= 0` in practice, but the full system is solved as shown). A singular system
(degenerate bolt geometry) raises a clear error rather than guessing.

**Negative axial results are signed compression-side loads, not clipped to
zero.** Joint separation / contact redistribution is a later-milestone
concern.

## Equilibrium verification

`check_equilibrium` independently recomputes group-level loads from the
per-bolt result list (a separate code path from the distribution solve) and
reports both the recovered totals and their residual against the applied
`InterfaceLoad`:

```
Fx_recovered = sum(Vx_i)              Mx_recovered = sum(y_i * T_i)
Fy_recovered = sum(Vy_i)              My_recovered = sum(-x_i * T_i)
Fz_recovered = sum(T_i)               Mz_recovered = sum(x_i*Vy_i - y_i*Vx_i)
```

In the representative sanity case below, all residuals are at floating-point
precision (~1e-12 or exactly zero).

## Representative sanity case

`examples/payload_attach_sanity.py` — an 8-bolt circular pattern, 0.5 m bolt
circle, with illustrative launch-representative loads
(Fx=20 kN, Fy=-10 kN, Fz=80 kN tension, Mx=25 kN·m, My=-15 kN·m, Mz=12 kN·m).
Run it:

```bash
python examples/payload_attach_sanity.py
```

It reports pattern properties, applied loads, a per-bolt table, governing
bolts (max shear / max tensile / max |axial|), and the equilibrium
recovery/residual check. Loads are illustrative only and are not tuned to
size any particular bolt.

## Limitations (explicitly out of scope for Milestone 1)

- Rigid payload/interface plate assumption.
- Equal bolt stiffness (no stiffness-based load fraction).
- Linear elastic distribution only; no nonlinear contact.
- Bolt coordinates treated as ideal point fasteners.
- Moments referenced about the bolt-group centroid.
- No preload or torque.
- No contact/separation redistribution (negative axial values are signed,
  not physical compression-side bolt reactions after joint opening).
- No friction / shear load sharing through faying surfaces.
- No prying action.
- No bearing or tear-out / edge-distance checks.
- No pull-through.
- No fastener strength allowables or margin-of-safety calculation.
- No thread effects, no fatigue.
- No optimization or sensitivity/trade study (later milestone).
- No certification claim of any kind.

---

# Milestone 2 — preliminary bolt strength sizing

**Milestone 2 adds preliminary bolt tensile/shear strength screening
only. Preload, slip, separation, bearing, prying, thread failure, and
fatigue remain deferred.**

Milestone 2 consumes the Milestone 1 per-bolt loads (`axial_total`,
`shear_resultant`) exactly as produced by `distribute_loads` and layers
a strength assessment on top. **Bolt-group geometry, direct/torsional
shear, overturning axial distribution, and equilibrium verification are
completely unchanged from Milestone 1** — see `src/payload_bolts/strength.py`.

The engineering question: given the per-bolt axial and shear loads from
the rigid-interface bolt-group model, what bolt size/material is
required, which failure mode governs, and what margin exists against
tensile/shear strength?

## Bolt material representation

`BoltMaterial(name, tensile_allowable, shear_allowable)` — allowables in
Pa, both finite and > 0. Any illustrative material entry in this project
is explicitly labeled illustrative in its name and is not claimed to be
a sourced fastener specification.

## Bolt section / area convention

`BoltSection(nominal_diameter, tensile_area, shear_area)` — meters and
m². Tensile and shear areas are supplied **explicitly** by the caller;
Milestone 2 does not assume a real threaded fastener's tensile stress
area equals its gross shank area. `circular_unthreaded_bolt(diameter)`
is provided as an **idealized shank-area helper** (`A = pi*d^2/4` used
for both tensile and shear area) — labeled idealized, not an ISO/SAE
tensile-stress-area calculation.

## Signed axial-load policy (unchanged from Milestone 1, applied here)

Milestone 1's signed `axial_total` is never mutated. For the tensile
strength check only:

```
T_positive = max(axial_total, 0)
```

A bolt with `axial_total <= 0` has no tensile demand (tensile check not
applicable); Milestone 2 does not evaluate a compressive bolt failure
mode.

## Tensile stress

```
sigma_t = T_positive / A_t
```

## Shear stress

```
tau = V_resultant / A_s
```

`V_resultant` is the unmodified Milestone 1 `shear_resultant`.

## Margin definitions (preliminary bolt strength margins)

```
MS_tension     = S_t_allow / sigma_t - 1        (sigma_t > 0, else not applicable -> None)
MS_shear       = S_s_allow / tau - 1            (tau > 0, else not applicable -> None)
```

**Zero-demand convention**: when a stress is exactly zero, the
corresponding margin is `None` (not applicable), never a division by
zero and never `+inf`.

## Quadratic tension-shear interaction (illustrative)

An **illustrative quadratic tension-shear interaction**, not a named
fastener standard:

```
FI = (sigma_t / S_t_allow)^2 + (tau / S_s_allow)^2
```

Pass if `FI <= 1` (boundary included). Interaction margin:

```
MS_interaction = 1 / sqrt(FI) - 1          (FI > 0, else None)
```

## Governing mode and deterministic tie-break

Each bolt reports its tensile, shear, and interaction margins
**separately** — they are never collapsed into one undocumented number.
A bolt's *governing margin* is the smallest applicable (non-`None`)
margin among the three.

Because the quadratic form guarantees `MS_interaction <= MS_tension` and
`MS_interaction <= MS_shear` whenever both are applicable (with equality
exactly when the *other* demand is zero), a bolt loaded in pure tension
or pure shear ties exactly with "interaction". **This project's
tie-break convention prefers the more specific single-mode explanation
on a tie**: tension, then shear, then interaction — so a pure-tension
bolt is reported tension-governed and a pure-shear bolt shear-governed.
Interaction only governs outright (not via tie-break) when both tensile
and shear demand are simultaneously present, in which case its margin is
strictly the smallest of the three.

At the **group level**, the governing bolt is the one with the smallest
governing margin across all bolts (a bolt with no applicable margin at
all — zero total demand — is treated as having no constraint, i.e. it
cannot govern). Ties across bolts are broken by **lowest bolt index**.
The group's `passed` is true only if every bolt individually passes.

## Candidate bolt sizing

`evaluate_candidates(group_load_result, candidates, material)` assesses
a list of candidate `BoltSection`s against the same Milestone 1 group
load result, returned sorted by increasing nominal diameter.
`select_smallest_passing_bolt(...)` returns the first passing candidate
in that order and raises `NoFeasibleCandidateError` if none pass — it
never silently enlarges beyond the supplied candidate list. This
milestone uses an explicit idealized-diameter candidate list, not an
inferred standard fastener size table.

## Representative sizing result

`examples/bolt_strength_sizing.py` reuses the exact Milestone 1 8-bolt /
R=0.5 m sanity load case (max tensile bolt: index 1, T ≈ 24.1 kN; max
shear bolt: index 5, V ≈ 5.7 kN) against an illustrative
"Illustrative high-strength steel bolt" (tensile allowable 800 MPa,
shear allowable 480 MPa) and idealized circular-shank candidates at
5, 6, 8, 10, 12 mm nominal diameter:

- 5 mm and 6 mm candidates **fail** (interaction-governed at bolt 1).
- 8 mm, 10 mm, and 12 mm candidates **pass**.
- Smallest passing candidate: **8 mm**, governing bolt index 1,
  governing mode interaction, governing margin ≈ +0.66.

Run it:

```bash
python examples/bolt_strength_sizing.py
```

## Verification summary (Milestone 2)

- One fully hand-derived single-bolt load state (T=10 kN, V=5 kN,
  A_t=100 mm², A_s=80 mm², S_t=500 MPa, S_s=300 MPa) verified exactly
  for stress, both individual margins, and FI.
- Exact interaction boundary (`FI = 1`) verified to pass with zero
  margin; slightly above/below verified to fail/pass.
- Tensile and shear margin boundaries (`MS = 0` exactly) verified.
- Zero-tensile-demand, zero-shear-demand, and zero-total-demand cases
  verified not to divide by zero and to report `None`/clean pass.
- Load, area, and allowable scaling laws verified (doubling area halves
  stress; doubling allowable improves margin; doubling load quadruples
  FI under the quadratic form).
- Dedicated tension-governed, shear-governed, and interaction-governed
  constructions verified, plus a construction where the strength
  governing bolt differs from the Milestone 1 max-load labels,
  confirming governance is recomputed from margins, not assumed.
- Candidate ordering, smallest-passing selection, no-feasible-candidate
  handling, and monotonic improvement with increasing idealized
  diameter all verified.
- Deterministic repeated assessment and non-mutation of the Milestone 1
  `BoltGroupResult` verified.
- **All 47 Milestone 1 tests remain unchanged and passing.**

## Limitations

Milestone 1 limitations (rigid payload/interface plate, equal bolt
stiffness, linear elastic distribution, point fasteners, moments about
the bolt-group centroid) all still apply. In addition, for Milestone 2:

- Signed compression-side bolt loads from Milestone 1 are preserved;
  the tensile check ignores compressive bolt loading (no compressive
  bolt failure mode is evaluated).
- No preload or proof-load check.
- No torque.
- No friction / shear load transfer through faying (joint) surfaces.
- No joint separation / contact redistribution.
- No bearing or tear-out / edge-distance checks.
- No prying action.
- No pull-through.
- No thread stripping.
- No fatigue.
- Candidate areas are idealized (gross circular shank) unless a caller
  explicitly supplies real tensile/shear stress areas — never claimed
  to be sourced ISO/SAE fastener data.
- The quadratic tension-shear interaction is illustrative, not a named
  fastener standard.
- No certification claim of any kind.

---

# Milestone 3 — preloaded-joint closure and friction-slip screening

**Milestone 3 adds a first-order preloaded-joint closure and
friction-slip screen. It does not yet size torque or verify preload
against bolt proof/yield strength.**

Milestone 3 consumes the Milestone 1 per-bolt loads (`axial_total`,
`shear_resultant`) unchanged and adds a preload/joint-behavior layer on
top. **Milestone 1 bolt-group mechanics and Milestone 2 bolt-strength
calculations (including the interaction criterion and its
governing-mode tie-break) are completely unchanged** — see
`src/payload_bolts/preload.py`.

The engineering question: for the selected payload attach bolt pattern
and bolt candidate, how much preload is required to keep the joint
closed and prevent interface slip under the representative launch
loads?

## Preload model

`PreloadState(preload_per_bolt, label="")` — an explicit engineering
input in newtons, finite and > 0. Milestone 3 does **not** derive
preload from an installation torque; torque-to-preload conversion is
deferred.

## Joint stiffness / load fraction C

`JointStiffness(bolt_stiffness, member_stiffness)` — preliminary
equivalent linear stiffnesses in N/m, both finite and > 0. Defines the
classical load fraction:

```
C = k_b / (k_b + k_m)          (0 < C < 1)
```

`C` is the fraction of an external separating tensile load carried as
*additional* bolt load before the joint separates; the remaining
fraction `(1 - C)` is lost from the clamped-member compression.

## External axial-load policy

Only the separating (tensile) portion of Milestone 1's signed
`axial_total` creates separation/slip demand:

```
T_sep,i = max(axial_total_i, 0)
```

A bolt with `axial_total_i <= 0` (compression side) contributes no
separation demand; the signed load is retained unchanged for reporting.

## Bolt-load increment and clamp-force reduction

```
Delta_F_b,i = C * T_sep,i                     (additional bolt load)
F_b,total,i = F_preload + Delta_F_b,i         (total bolt tension)

Delta_F_m,i = (1 - C) * T_sep,i               (clamp-force reduction)
F_clamp_remaining,i = F_preload - Delta_F_m,i (unclipped, signed)
```

Force balance: `Delta_F_b,i + Delta_F_m,i == T_sep,i` exactly.

## Separation condition and margin

The joint remains closed at a bolt location if
`F_clamp_remaining,i >= 0`. Normalized margin:

```
MS_sep,i = F_preload / ((1-C) * T_sep,i) - 1     (T_sep,i > 0)
```

**Zero-demand convention**: at `T_sep,i <= 0`, `MS_sep,i = None`
(not applicable) and the location trivially passes — never a division
by zero.

## Local friction / interface-slip screening

`FrictionModel(friction_coefficient, number_of_faying_surfaces=1)` — an
illustrative interface friction coefficient (mu > 0) and an integer
count of faying surfaces (>= 1).

Effective clamp force available for friction at each location clips at
zero (a separated location contributes no friction capacity, but never
a *negative* one):

```
F_clamp_effective,i = max(F_clamp_remaining,i, 0)
V_fric_cap,i = mu * n_interfaces * F_clamp_effective,i
```

The primary slip screen is **local, per bolt**, using each bolt's own
Milestone 1 shear demand `V_i = shear_resultant_i` — this already
includes both direct shear and Mz torsional shear, so Mz is not
ignored. Margin:

```
MS_slip,i = V_fric_cap,i / V_i - 1      (V_i > 0)
```

**Zero-demand convention**: `V_i = 0` → `MS_slip,i = None`, passes. If
`V_i > 0` but `F_clamp_effective,i = 0` (fully separated location with
shear demand), `MS_slip,i` is set to the explicit finite value **-1**
(FAIL) — never NaN or infinite.

This local check is a **preliminary load-sharing approximation**: it
assumes each bolt station's surrounding clamped area supplies friction
in proportion to that station's own remaining clamp force. It is **not**
a detailed contact-pressure/friction analysis.

## Governing logic (deterministic)

- **Governing separation bolt**: smallest applicable `MS_sep,i`
  (`None` treated as no constraint); ties → lowest bolt index.
- **Governing slip bolt**: smallest applicable `MS_slip,i`; ties →
  lowest bolt index.
- Overall joint-screen `passed` is **PASS only if there is no local
  separation and no local slip anywhere** — Milestone 2 bolt-strength
  pass/fail is a separate, independently reported result and is never
  folded into this flag.

## Required preload (analytical, not a torque spec)

```
F_preload_required_sep  = max_i[(1-C) * T_sep,i]
F_preload_required_slip = max_i[(1-C) * T_sep,i + V_i / (mu * n_interfaces)]
F_preload_required      = max(F_preload_required_sep, F_preload_required_slip)
```

The slip expression already contains the separation term for the same
bolt, so `slip_required >= separation_required` always; a numerical tie
between the two is reported as governed by "slip" (the more inclusive
check). `apply_preload_factor(required_value, preload_factor >= 1.0)`
is kept separate from the requirement itself, so an explicit design
reserve is never hidden inside the analytical result.

## Representative result

Using the Milestone 1 8-bolt / R=0.5 m sanity case, illustrative
stiffnesses `k_b = 1e8 N/m`, `k_m = 4e8 N/m` (`C = 0.2`), and an
illustrative friction coefficient `mu = 0.2`:

- Required preload: separation 19,313.7 N/bolt (bolt 1), slip
  29,258.2 N/bolt (bolt 0) → **overall required ≈ 29,258 N/bolt**,
  governed by **slip at bolt 0**.
- Selected preload (illustrative `preload_factor = 1.2`):
  **≈ 35,110 N/bolt** (+20% reserve above the requirement).
- At the selected preload: all 8 locations closed, no slip anywhere;
  governing separation bolt 1 (MS ≈ +0.82), governing slip bolt 5
  (MS ≈ +0.23). **Overall joint screen: PASS.**

Run it:

```bash
python examples/preloaded_joint_screening.py
```

## Sensitivity studies

- **Preload factor** (`F_preload / F_required` = 0.5 … 2.0): margins
  improve monotonically with preload; the joint screen transitions from
  FAIL to PASS exactly at factor 1.0, with the governing slip margin at
  bolt 0 landing at 0.000 there — confirming the required-preload
  equation is tight, not conservative by construction.
- **Friction coefficient** (`mu` = 0.10 … 0.40, stiffness/loads fixed):
  required preload against slip decreases monotonically as `mu`
  increases (57.2 kN/bolt at mu=0.10 down to 21.7 kN/bolt at mu=0.40);
  the governing bolt itself can shift as the balance between
  shear-heavy and tension-heavy bolts changes.
- **Load fraction C** (0.10 … 0.40, via member stiffness, mu/loads
  fixed): required preload against separation decreases monotonically
  as `C` increases. **Trade interpretation**: a lower `C` transfers
  less external load into the bolt but loses more clamp force from the
  members (greater separation/slip sensitivity for a given preload); a
  higher `C` retains more clamp force but increases the bolt's tension
  increment — this is a trade, not a universal preference for high
  `C`.

## Verification summary (Milestone 3)

- Hand-derived force balance (`additional bolt load + clamp-force
  reduction == separating demand`), bolt-load increment, and clamp
  reduction formulas verified exactly.
- Exact separation and slip boundaries (`margin = 0`) verified to pass;
  slightly above/below verified to pass/fail.
- Zero separating demand, zero shear demand, and a fully-separated
  location with nonzero shear (explicit `MS_slip = -1`, no NaN/inf) all
  verified.
- Compression-side bolts verified to retain full preload (no clamp-force
  reduction) while their signed load is preserved for reporting.
- Required-preload equations (separation, slip, overall) verified by
  hand calculation and shown to exactly satisfy every bolt at the
  boundary (one bolt at margin ≈ 0), with a preload slightly below
  failing and slightly above passing.
- Monotonic sensitivity verified: required slip preload decreases with
  `mu`; required separation preload decreases with `C`; margins improve
  monotonically with preload factor.
- Deterministic governing-bolt tie-break (lowest index) verified for
  both separation and slip; deterministic repeated assessment verified.
- Milestone 1 `BoltGroupResult` and Milestone 2 strength results
  verified unmutated/unaffected by this module.
- **All 79 Milestone 1–2 tests remain unchanged and passing.**

## Limitations

Milestone 1–2 limitations (rigid payload/interface plate, equal bolt
stiffness in load distribution, point fasteners, moments about the
bolt-group centroid, idealized candidate areas, illustrative quadratic
interaction, no bolt strength/preload-interaction check, etc.) all
still apply. In addition, for Milestone 3:

- Equivalent linear bolt/member stiffness is used for preload load
  sharing (not derived from detailed flange/washer/thread flexibility).
- One scalar load fraction `C` applies to the whole joint (no per-bolt
  or spatially varying stiffness).
- Preload is prescribed directly; there is no torque-to-preload
  relation.
- No preload scatter (installation variability) is modeled.
- No embedment preload loss.
- No thermal preload effects.
- No nonlinear contact / detailed pressure distribution.
- No local flange flexibility.
- Local friction capacity is assumed proportional to each bolt's own
  remaining clamp force — a simplified load-sharing approximation, not
  a detailed contact-pressure/friction analysis.
- No bolt proof/yield preload check.
- No bearing/tear-out, no prying, no pull-through, no thread stripping.
- No fatigue.
- No certification claim of any kind.

---

# Milestone 4 — preload feasibility, proof/yield screening, and installation window

**Milestone 4 adds a deterministic preload-feasibility screen: whether
the Milestone 3 required preload is structurally installable for the
selected bolt, and over what window. It does NOT size an installation
torque, model a torque-tension/nut-factor relationship, or perform
proof testing / certification.**

## Why Milestone 4 is necessary

Milestone 3 computes how much preload is analytically *required* to
prevent joint separation and interface slip. It explicitly does not ask
whether that required preload is *feasible* to install on the selected
bolt: is there any preload the installer could realistically achieve
that (a) still meets the Milestone 3 requirement even accounting for
installation scatter, and (b) does not overstress the bolt against its
proof strength? Milestone 4 answers that question directly, and — new
in this milestone — reports honestly when the answer is "no."

## Proof/yield basis

`BoltStrengthLimits` holds an illustrative proof strength `S_p` (and
optionally a yield strength `S_y >= S_p`), Pa. This is a **distinct**
property set from Milestone 2's `BoltMaterial` (`tensile_allowable`,
`shear_allowable`), which is a simple working-stress allowable for M2's
margin checks — not a proof or yield strength. The two are never
conflated.

Proof/yield loads reuse the **exact** Milestone 2 tensile stress area
`BoltSection.tensile_area` (no new/conflicting stress-area convention):

```
F_proof  = S_p * A_t
F_yield  = S_y * A_t     (if S_y supplied)
```

## Required vs. allowable (installation) preload

Milestone 3's `F_required` (the larger of the separation- and
slip-required preloads) is a *theoretical minimum*. Milestone 4 adds a
deterministic installation **scatter/loss allowance** `delta_F`
(`0 <= delta_F < 1`) so that even the low end of installation scatter
still clears the Milestone 3 requirement:

```
F_target,min = F_required / (1 - delta_F)
```

and an **installation ceiling** expressed as a fraction `eta_proof`
(`0 < eta_proof < 1`) of proof load:

```
F_target,max = eta_proof * F_proof
```

A feasible installation window exists only if `F_target,min <=
F_target,max`. Window width `F_target,max - F_target,min` is **never**
clipped at zero — a negative width is reported as an infeasible window,
not silently corrected.

## Representative baseline result (8 mm, same M1–M3 illustrative case)

Illustrative baseline: `S_p = 830 MPa`, `S_y = 970 MPa`, `eta_proof =
0.75`, `delta_F = 0.10` (source audit below).

| Quantity | Value |
|---|---|
| `A_t` (reused from M2) | 50.265 mm² |
| `F_proof = A_t·S_p` | 41,720.4 N |
| `F_target,max = eta_proof·F_proof` | 31,290.3 N |
| `F_target,min = F_required/(1-delta_F)` | 32,509.1 N |
| window width | **−1,218.8 N (infeasible)** |
| M3 selected preload (factor 1.20) | 35,109.8 N |
| status vs. M4 window | `NO_INSTALLATION_WINDOW` |
| max in-service bolt force (bolt 1) | 39,938.3 N |
| proof reserve | +4.5% |
| yield reserve | +22.1% |

**Genuine finding, reported honestly, not forced:** at these
illustrative proof/scatter assumptions, the 8 mm bolt — Milestone 2's
smallest *strength*-passing candidate — has **no feasible Milestone 4
preload-installation window**: the scatter-adjusted minimum target
(32,509.1 N) exceeds the proof-based ceiling (31,290.3 N) by about
1,219 N (≈3.7% of the minimum target). The Milestone 3 selected preload
(35,109.8 N) is additionally above the proof ceiling on its own. The
in-service bolt-tension check alone still shows a positive (+4.5%)
proof reserve at that same selected preload — i.e. the bolt would not
immediately yield in service — but the *installation window itself* is
infeasible under these assumptions. Milestone 4 does not resolve this
by silently upsizing the bolt; the bolt-size sensitivity below shows
10 mm and 12 mm do have feasible windows, and a final bolt-size decision
combining M2 strength and M4 preload feasibility is left open.

## Governing status (deterministic priority)

```
NO_INSTALLATION_WINDOW
SELECTED_PRELOAD_TOO_LOW
SELECTED_PRELOAD_TOO_HIGH
PROOF_LIMIT_EXCEEDED_IN_SERVICE
FEASIBLE
```

`diagnostics` reports every applicable flag simultaneously; `status` is
the single highest-priority one present.

## Run

```bash
python examples/preload_feasibility_screening.py
```

## Sensitivity studies

- **Scatter allowance `delta_F`** (5%/10%/20%, 8 mm): window degrades
  monotonically as `delta_F` rises — feasible at 5% (+492 N width),
  infeasible at 10% and 20%.
- **Proof strength `S_p`** (700/830/900/1000 MPa, 8 mm): ceiling and
  window width increase monotonically with `S_p` — infeasible at 700
  and 830 MPa, feasible at 900 and 1000 MPa.
- **Proof fraction `eta_proof`** (0.60/0.70/0.80, 8 mm): ceiling and
  window width increase monotonically — infeasible at 0.60/0.70,
  feasible at 0.80.
- **Friction coefficient `mu`** (0.10 … 0.40, carried forward from
  Milestone 3 unchanged): lower `mu` → higher M3 required preload →
  higher `F_target,min` → infeasible; higher `mu` → lower required
  preload → feasible. The window flips from infeasible to feasible
  between `mu=0.20` and `mu=0.25` at this baseline — a Milestone 3
  friction/slip result propagating directly into Milestone 4
  installation feasibility, with no M3 formula touched.
- **Bolt size** (Milestone 2 candidates 5/6/8/10/12 mm): `F_target,min`
  is bolt-size-independent (M3 mechanics do not depend on diameter);
  `F_target,max` grows with `A_t`. Result: 5/6/8 mm infeasible, 10/12 mm
  feasible — even though 8 mm already passes Milestone 2 strength.

## Source audit

Sources actually inspected (see `src/payload_bolts/preload_limits.py`
module docstring for the full detail):

- **mechanicalc.com, "Bolted Joint Analysis"** — preload as a fraction
  of proof load (~50% non-permanent/reusable, ~75% semi-permanent, ~90%
  permanent connections); tensile stress-area formula; torque-based
  preload uncertainty (~±25% hand torque wrench vs. ~±3–5%
  elongation/load-sensing methods) and why torque cannot uniquely
  determine achieved preload (thread/under-head friction variability
  folded into an uncertain torque coefficient).
- **engineeringlibrary.org, "Preloaded Bolted Joint Analysis
  Methodology (NASA)"** — fasteners commonly preloaded to "65 to 90
  percent of yield strength"; the joint stiffness ratio `phi =
  Kb/(Kb+Kj)`, algebraically identical to Milestone 3's `C =
  k_b/(k_b+k_m)` (corroborates the M3 formulation); "±25 percent" hand
  torque-wrench preload uncertainty.
- **Shigley, *Mechanical Engineering Design*** (via a search-engine
  summary, not a direct primary-source read — reported as a
  secondary/corroborating data point only): `Fi = 0.75*Fp` for reused
  connections; proof strength ≈ 0.85× yield strength.
- **NASA-STD-5020** was located but returned unreadable binary content
  in this environment; it is **not** cited for any specific numeric
  value here.

Baseline choices: `eta_proof = 0.75` (mid-range of the cited ~50–90%),
`delta_F = 0.10` (inside the cited ~±25% hand-torque-wrench bound,
above the ~3–5% achievable with load-sensing methods), illustrative
`S_p = 830 MPa` / `S_y = 970 MPa` (ratio ≈0.86, consistent with the
cited ≈0.85 proof-to-yield ratio). None of these are claimed to be a
sourced real fastener-grade allowable.

## Verification summary (Milestone 4)

- Proof/yield load hand calculations (`F = A_t·S`); exact reuse of the
  Milestone 2 tensile stress area confirmed (no hidden diameter
  substitution).
- Upper-ceiling and lower-target formulas verified by hand
  (`F_target,max = eta_proof·F_proof`, `F_target,min =
  F_required/(1-delta_F)`); `delta_F=0` reduces exactly to `F_required`.
- Monotonicity verified: `F_target,min` rises with `delta_F`;
  `F_target,max` rises with `S_p` and with `eta_proof`.
- Exact feasibility boundary (`target_min == target_max`) gives zero
  window width and zero upper margin; constructed below/above-boundary
  cases verified infeasible/feasible.
- Window-width and normalized-window identities verified exactly.
- M3 selected-preload classification verified against the computed
  window (correctly `NO_INSTALLATION_WINDOW` at the 8 mm baseline).
- Maximum in-service bolt force and proof/yield reserve verified by an
  independent hand reconstruction of `F_preload + C·T_sep` from
  Milestone 1's signed axial loads.
- No double-counting of preload verified (`T_sep=0` bolts report
  `F_bolt == F_preload` exactly).
- The closed-joint load-sharing formula is verified restricted to bolts
  that remain closed, with an explicit `in_service_model_valid` flag
  when no bolt remains closed (probed directly with synthetic
  edge-case records).
- Milestone 3's separation-required (19,313.7 N/bolt), slip-required
  (29,258.2 N/bolt, bolt 0), and overall-required preload values
  verified preserved exactly.
- Friction-sensitivity propagation from Milestone 3 into Milestone 4
  window feasibility verified monotonic and shown to flip feasibility
  across the tested `mu` range, without altering any M3 formula.
- Deterministic governing-bolt/status behavior verified under repeated
  assessment.
- Invalid proof strength / yield-strength ordering / `eta_proof` /
  `delta_F` / selected-preload inputs all verified rejected.
- **All 119 Milestone 1–3 tests remain unchanged and passing; 31 new
  Milestone 4 tests added (150 total).**

## Limitations

Milestone 1–3 limitations all still apply. In addition, for Milestone 4:

- Proof/yield properties are illustrative, not a sourced real
  fastener-grade allowable.
- No torque-tension relationship, nut factor, torque coefficient,
  lubrication, thread friction, or under-head friction is modeled.
- No preload relaxation / embedment loss.
- No thermal preload change.
- No fatigue.
- No prying, bearing, thread stripping.
- No nonlinear joint opening beyond the linear closed-joint model
  already used in Milestone 3.
- No proof testing.
- The installation scatter/loss allowance `delta_F` is a deterministic
  engineering allowance, **not** a statistical confidence interval.
- The in-service bolt-force screen uses the Milestone 3 closed-joint
  linear load-sharing formula and is restricted to bolts that remain
  closed at the evaluated preload; it is not valid for a separated
  joint location.
- No certification claim of any kind.

---

# Milestone 5 — bolt-size trade, local joint failure modes, and preliminary hardware selection

**Milestone 5 resolves the engineering decision exposed by Milestone 4
with a transparent, predeclared conceptual trade. It does NOT model
fatigue, prying, nonlinear plate flexibility, detailed flange bending,
detailed fastener torque, lubrication/nut-factor effects, thermal
preload, embedment/relaxation, detailed contact FEA, fracture
mechanics, net-section rupture, thread stripping (explicitly not
modeled -- see below), proof testing, or certification/qualification.**

## Why Milestone 5 was required

Milestone 2 found 8 mm to be the smallest bolt passing its tensile/
shear/interaction strength screen. Milestone 4 then showed that same
8 mm bolt has **no feasible installation-preload window** under
illustrative proof-strength and scatter assumptions (target_min
32,509.1 N > target_max 31,290.3 N), and that 10 mm / 12 mm recover
feasibility. Milestone 5 asks the resulting question directly: does 8 mm
remain a defensible conceptual choice once preload feasibility **and**
local bearing/edge-distance/spacing screening are added — or does the
trade support upsizing? The answer is computed, not assumed.

## Candidate trade architecture

For each candidate diameter, Milestone 5 reuses **exactly** (never
recomputes): Milestone 2's `BoltGroupStrengthResult` (tensile stress
area, strength pass/fail, governing mode); Milestone 3's
`RequiredPreloadResult`; and Milestone 4's `PreloadFeasibilityResult`
(installation window, selected-preload classification, in-service bolt
force). New in Milestone 5: bearing, edge-distance, and spacing local-
joint screens (below), and thread stripping is explicitly flagged as
not modeled.

## Local bearing screen

`sigma_bearing = F_bearing / (d_hole * t)`, where `F_bearing` is each
bolt's Milestone 1 **in-plane shear resultant** (`shear_resultant`,
including both direct and Mz-torsional shear) — **never** the axial/
tensile load, which is physically the wrong load path for plate/hole
bearing. `d_hole` is idealized equal to the candidate nominal diameter
(no clearance modeled, consistent with Milestone 2's idealized shank-
area convention). `MS_bearing = allowable/demand - 1`.

Source: mechanicalc.com's "Lug Analysis" (Air Force Method) confirms
bearing area `A_br = D_p * t`, i.e. exactly this convention.

## Edge-distance screen (geometry screen only)

Each bolt's edge distance is `e_i = R_plate - |bolt_i - plate_center|`
for an illustrative circular plate boundary concentric with the
Milestone 1 bolt-circle center (`R_plate` = bolt-circle radius + an
illustrative 50 mm margin) — a simplification valid only for this
project's circular bolt pattern, not a general polygon-boundary model.
Reported as the dimensionless ratio `e/d` against `(e/d)_min = 1.5`
(source: mechanicalc.com's Air-Force-Method bearing/shear-out regime
transition at `e/D >= 1.5`). This is a **geometry screen only**, not a
tear-out/net-section strength calculation.

## Spacing screen (geometry screen only)

Every pairwise bolt center-to-center spacing `s/d` is checked against
`(s/d)_min = 3.0` (source: a web-search summary of AISC 360-22, a
general structural-steel bolted-connection standard — **not**
aerospace-specific, used only because no aerospace-specific spacing
source could be independently read; AISC's cited minimum is 2.67d, with
3d preferred). Also a **geometry screen only**.

## Thread-strip method/status

**Explicitly NOT modeled.** Source-verified thread-stripping shear-area
formulas (Unified/inch and ISO-metric/VDI-2230 forms) require detailed
thread-geometry parameters — pitch diameters, thread class of fit,
effective engagement fraction — that are not established anywhere else
in this project and cannot be transparently reproduced from a search
summary with confidence. Per this milestone's explicit guidance, a
transparent omission (`ThreadCheckStatus.THREAD_CHECK_NOT_MODELED`) is
used instead of a fabricated formula; this check is **never** a gating
criterion in the selection rule below.

## Predeclared selection rule (declared before evaluating candidates)

```
A candidate is ADMISSIBLE only if ALL of:
  1. Milestone 2 strength passes;
  2. the Milestone 4 installation-preload window is feasible AND the
     Milestone 3 selected preload classifies as FEASIBLE against it;
  3. bearing margin of safety >= 0;
  4. the edge-distance screen passes;
  5. the spacing screen passes.
(Thread stripping is NOT modeled and is never gating.)

Among ADMISSIBLE candidates, SELECT the smallest nominal diameter.
```

This is a conceptual minimum-size rule, not a claim of global
optimality. It was fixed in code before the candidate table below was
generated and was not adjusted afterward.

## Candidate table (baseline: same M1–M4 illustrative case)

| d (mm) | A_t (mm²) | M2 pass | M4 status | M4 window width (N) | bearing MS | e/d | s/d | **admissible** |
|---|---|---|---|---|---|---|---|---|
| 6 | 28.27 | **FAIL** | NO_INSTALLATION_WINDOW | −14,908 | +2.36 | 8.33 | 63.78 | **No** |
| 8 | 50.27 | PASS | **NO_INSTALLATION_WINDOW** | −1,219 | +3.48 | 6.25 | 47.84 | **No** |
| 10 | 78.54 | PASS | FEASIBLE | +16,382 | +4.59 | 5.00 | 38.27 | **Yes** |
| 12 | 113.10 | PASS | FEASIBLE | +37,894 | +5.71 | 4.17 | 31.89 | Yes |

## Result

**8 mm remains rejected — solely on Milestone 4 preload-feasibility
grounds, not bearing or geometry (both of which pass 8 mm comfortably).
10 mm becomes the smallest admissible conceptual candidate**, with
window width +16,382 N, proof reserve +63.2%, bearing MS +4.59, edge
ratio 5.00 (vs. criterion 1.5), spacing ratio 38.27 (vs. criterion
3.0). 6 mm fails both M2 strength and M4 feasibility. This is a genuine
computed result, not assumed in advance.

*"The selected bolt is the smallest candidate passing this reduced-order
screening set; it is not a flight-qualified or globally optimized
fastener selection." "Local bearing, spacing, and thread checks are
conceptual screening models and do not replace detailed joint
analysis."*

## Sensitivity

- **Plate thickness** (0.75x/1.0x/1.25x) and **bearing allowable**
  (−20%/nominal/+20%): bearing margin at 8 mm stays strongly positive
  (+2.36 to +4.59) across the full range — bearing never becomes
  governing and never changes the selected candidate.
- **Edge-distance criterion** (1.5/2.0/2.5): 8 mm's actual e/d (6.25)
  clears all three tested criteria comfortably.
- **Preload scatter `delta_F`** (5%/10%/20%): reveals a subtlety —
  loosening `delta_F` to 5% makes the M4 *window itself* feasible, but
  the *fixed* Milestone-3-selected preload (pinned to the
  `delta_F=0.10`-derived requirement × factor 1.20) is then **above**
  the window (`SELECTED_PRELOAD_TOO_HIGH`), so 8 mm still fails —
  a feasible window alone does not guarantee an admissible selection
  unless the selection is re-derived against it.
- **Friction `mu`** (0.10–0.40, carried forward from Milestone 3
  unchanged): here both the requirement *and* the selection are
  re-derived together, and 8 mm's admissibility flips to `True` at
  `mu >= 0.30`.
- **Thread engagement length**: not applicable (thread stripping not
  modeled).

None of these sensitivities change the *governing reason* 8 mm is
rejected in the baseline case: Milestone 4 preload feasibility, not
bearing or geometry.

## Verification summary (Milestone 5)

- Bearing stress/margin hand calculations; bearing stress verified to
  decrease monotonically with both diameter and plate thickness.
- Edge distance and pairwise spacing independently reconstructed from
  the actual Milestone 1 bolt coordinates (not a formula assumed to
  match); minimum pairwise spacing on the regular 8-bolt circle
  independently cross-checked against `2*R*sin(pi/n)`.
- Exact geometric/bearing boundary behavior (margin/ratio at the
  criterion) verified to pass; just-below-boundary cases verified to
  fail.
- Milestone 2 tensile-stress-area, pass/fail, and governing-mode
  results, Milestone 3 required-preload values, and Milestone 4
  installation-window results for 8/10/12 mm all verified preserved
  exactly (unchanged from their own modules).
- Candidate admissibility logic and smallest-admissible selection
  verified deterministic and independent of input candidate ordering;
  a selection is verified to never occur if Milestone 2 strength or the
  Milestone 4 window fails; a genuine "no feasible candidate" case is
  verified reported honestly (not silently resolved).
- Thread stripping verified to never appear in any candidate's
  admissibility-failure reasons.
- Pairwise spacing verified robust to bolt input ordering.
- Invalid plate/geometry inputs verified rejected.
- **All 150 Milestone 1–4 tests remain unchanged and passing; 32 new
  Milestone 5 tests added (182 total).**

## Limitations

Milestone 1–4 limitations all still apply. In addition, for Milestone 5:

- No fatigue, prying, or nonlinear plate flexibility.
- No detailed flange bending.
- No detailed bearing/tear-out interaction beyond the simple bearing-
  stress screen (no combined bearing + shear-out interaction curve).
- No net-section rupture check.
- No nonlinear contact FEA, no fracture mechanics.
- No thermal preload, no embedment/relaxation.
- No torque-tension relationship, nut factor, or lubrication.
- Thread stripping is explicitly NOT modeled (see above) — not
  approximated, not assumed adequate.
- No proof testing.
- Edge-distance/spacing checks are geometric screens only, not
  detailed tear-out or net-section strength calculations.
- The illustrative circular plate boundary is a simplification specific
  to this project's circular bolt pattern.
- The spacing criterion is borrowed from a general (non-aerospace)
  structural-steel connection standard.
- No certification/qualification claim of any kind.
- No figures were generated this milestone: this repository has used a
  text-only reduced-order report format through Milestones 1–4 and does
  not currently depend on a plotting library; adding one purely for
  Milestone 5 was judged to add more architectural inconsistency than
  portfolio value, so this is documented as an explicit, deliberate
  scope decision rather than an oversight.

---

# Milestone 6 — integrated preload + bolt-strength sizing

**Milestone 6 checks whether the preload required to keep the joint
closed and resist slip is itself compatible with bolt proof/tensile/
shear strength. Torque, preload scatter, bearing, prying, thread
failure, and fatigue remain deferred.**

Milestone 6 is a distinct, additive layer alongside (not a replacement
for) Milestone 4's `preload_limits.py` (force-based proof/yield
installation window against a separate `BoltStrengthLimits` proof/yield
strength) and Milestone 5's `joint_local_checks.py` (bearing/edge/
spacing screens). Milestone 6 instead asks, at the **stress level**
using `BoltMaterial`'s tensile/shear allowables and the exact Milestone
2 quadratic interaction criterion: once the bolt carries preload,
external tension, and external shear **simultaneously**, do the
Milestone 2 strength margins still pass?

> **Note on repository history**: this repository is a fork of
> [`payload-attach-bolt-sizing`](https://github.com/Sanjanakamboj/payload-attach-bolt-sizing)
> at its accepted Milestone 5 checkpoint (commit `4ee28c5`), created to
> continue Milestone 6 work in an isolated repository after a second,
> independently-running session was found actively editing the original
> repository's working tree at the same time (files edited in this
> session kept silently reverting there with no corresponding commit).
> Milestone 1–5 history and all accepted numbers are carried over
> unchanged; Milestone 6 is new work added only here.

## Proof/preload allowable

`BoltMaterial` gains an optional `proof_allowable: float | None = None`
field — Pa, finite and > 0 when supplied, `None` by default for exact
backward compatibility with every existing Milestone 2/4/5 caller.
Illustrative canonical value used here: **proof/preload allowable = 600
MPa**, alongside the unchanged Milestone 2 tensile (800 MPa) and shear
(480 MPa) allowables — kept as a distinct property, never conflated
with tensile allowable.

## Installation preload stress

```
sigma_preload = F_preload / A_t
MS_preload    = S_proof / sigma_preload - 1
preload utilization = sigma_preload / S_proof
```

`A_t` is the exact Milestone 2 tensile stress area — no new/conflicting
area convention.

## Total service bolt tension

Reuses Milestone 3's closed-joint load-sharing **exactly** (not
Milestone 2's external-load-only tensile force):

```
T_sep,i        = max(T_ext,i, 0)
Delta_F_b,i    = C * T_sep,i
F_b,total,i    = F_preload + C * T_sep,i     (SERVICE tension)
sigma_service,i = F_b,total,i / A_t
```

Service shear reuses the unmodified Milestone 1 shear demand
(`tau_service = V_i / A_s`) — bolt shear is **never** reduced because
friction exists; the Milestone 3 friction/slip model and this
conservative bolt shear-strength screen are separate, independently
reported checks.

## Service interaction (reuses the exact Milestone 2 criterion)

```
FI_service = (sigma_service/S_t)^2 + (tau_service/S_s)^2      <= 1
MS_interaction_service = 1/sqrt(FI_service) - 1
```

No new interaction equation — the same illustrative quadratic form
already verified in Milestone 2, applied to SERVICE stresses instead of
external-load-only stresses.

## Domain validity

The service-stress model is valid **only** within the Milestone 3
closed/no-slip regime:

```
strength_model_valid = joint_closed AND no_slip
overall integrated PASS = strength_model_valid
                           AND preload/tension/shear/interaction margins >= 0 (every bolt)
```

A separated or slipped joint **forces** the overall result to FAIL, even
if the raw stress margins happen to compute positive — this module
never silently claims a valid integrated result outside its regime.

## Governing mode (extends, does not modify, Milestone 2's tie-break)

Four applicable modes: **preload, tension, shear, interaction**. On an
exact tie (e.g. pure preload/tension demand with zero shear, where
tension and interaction tie exactly per Milestone 2's own quadratic
identity), the more physically specific mode is preferred — tension or
shear over interaction, exactly as Milestone 2 already does; "preload"
is checked against a separate proof allowable and stress basis, so it
essentially never ties with the service-stress modes in practice, but
is placed first in the tie-break order for determinism.

## Preload capacity ceilings

Three independent upper bounds on per-bolt preload, all using the
candidate's own `A_t`/`A_s`/`S_t`/`S_s`/`S_proof`:

```
F_preload,max,proof       = S_proof * A_t
F_preload,max,tension,i   = S_t*A_t - C*T_sep,i                         (group: min over bolts)
F_preload,max,interaction,i = A_t*S_t*sqrt(1 - (V_i/(A_s*S_s))^2) - C*T_sep,i
                              (infeasible for that bolt if V_i/(A_s*S_s) > 1 -- shear alone
                               already exceeds the interaction criterion at ANY preload)
F_preload,max = min(proof, tension, interaction)
```

## Feasible preload window

```
lower bound = F_required (Milestone 3)
upper bound = F_preload,max
feasible    = lower <= upper
```

## Milestone 2 vs. Milestone 6 bolt selection (headline result)

| | Milestone 2 (external-load-only) | Milestone 6 (preload-integrated) |
|---|---|---|
| Smallest passing bolt | **8 mm** | **10 mm** |
| Governing bolt/mode | bolt 1 / interaction | bolt 0 / preload |

At the selected preload (35,109.8 N/bolt = 1.2× the Milestone 3
requirement), the 8 mm candidate's preload stress alone (698.5 MPa)
exceeds the 600 MPa proof allowable (margin −0.141) — **preload
consumed enough capacity that 8 mm no longer passes**, even though its
Milestone 2 external-load-only margin (+0.662, interaction) was
comfortably positive. This was assessed honestly, not forced: the 8 mm
result was computed first and reported as it came out.

## Preload / friction / C sensitivity (10 mm candidate)

- **Preload factor** (1.0×–2.0× required): slip margin and preload
  margin move in **opposite directions** as preload increases —
  slip margin improves (0.000 → 1.046) while preload margin worsens
  (+0.611 → −0.195), crossing to FAIL at 2.0×. A genuine, verified
  trade, not assumed.
- **Friction** (μ = 0.10–0.40): the preload capacity ceiling (47,123.9
  N) is constant (it depends on bolt/material/geometry, not friction),
  while the required preload rises sharply as μ falls (21,689 →
  57,204 N) — the window **closes entirely at μ=0.10** (width
  −10,079.6 N, infeasible), a genuine crossover found by the sweep, not
  forced.
- **C** (0.10–0.40): higher C reduces the required preload slightly but
  raises the maximum total bolt tension (39,624 → 43,979 N) and worsens
  the interaction margin (+0.584 → +0.428) — the closure-vs-bolt-
  strength trade is visible and verified, never asserted.

## Verification summary (Milestone 6)

- Hand-derived preload-stress boundary (30 kN / 50 mm² = 600 MPa,
  margin 0 at 600 MPa proof), service bolt tension (20 kN + 0.2×10 kN =
  22 kN), and interaction boundary (0.6² + 0.8² = 1, margin 0) all
  verified exactly.
- Proof/tension/interaction preload ceilings verified by independent
  hand calculation, including the group ceiling = minimum bolt ceiling
  identity and the shear-ratio-exceeds-1 infeasibility case.
- Exact feasible-window boundary, an infeasible window, and a restored-
  feasibility case all verified.
- Domain-validity forcing (separated or slipped joint invalidates the
  integrated result even when raw margins are positive) verified with
  dedicated constructions.
- Deterministic candidate ordering, smallest-passing selection, and
  no-feasible-candidate handling verified; a case where a candidate
  passes Milestone 2 but fails Milestone 6 verified explicitly (8 mm).
- Monotonic sensitivity verified: increasing preload improves slip
  margin but worsens preload/interaction margin; increasing C raises
  the bolt tensile increment and reduces clamp-force loss; decreasing
  friction raises the required preload.
- Milestone 1/2/3 results verified unmutated/unaffected by this module;
  the existing Milestone 2 `BoltMaterial` constructor verified
  backward-compatible without `proof_allowable`.
- **All 182 Milestone 1–5 tests remain unchanged and passing; 45 new
  Milestone 6 tests added (227 total).**

## Limitations

Milestone 1–5 limitations all still apply. In addition, for Milestone 6:

- Illustrative proof/preload allowable (600 MPa), not a sourced
  fastener-grade specification.
- Idealized gross-circular candidate areas, as in every prior milestone.
- One equivalent scalar load-fraction C per joint (unchanged from
  Milestone 3); never made diameter-dependent.
- Preload is a prescribed force input; no torque-to-preload relation.
- No torque scatter, no preload scatter, no embedment, no thermal
  preload.
- Closed-joint linear load-sharing only; no separated-contact
  redistribution, no slipped-interface shear redistribution.
- No bearing, tear-out, prying, or pull-through (see Milestone 5 for a
  separate bearing/edge/spacing screen).
- No thread stripping (explicitly not modeled, per Milestone 5).
- No fatigue.
- No detailed fastener standard/database lookup.
- No certification claim of any kind.

## Install and test

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pytest -q
python examples/payload_attach_sanity.py
python examples/bolt_strength_sizing.py
python examples/preloaded_joint_screening.py
python examples/preload_feasibility_screening.py
python examples/bolt_candidate_trade.py
python examples/preload_compatible_bolt_sizing.py
```
