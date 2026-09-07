"""Integrated preload + bolt-strength sizing (Milestone 6).

Builds on the exact Milestone 1 8-bolt / R=0.5 m load case, the
Milestone 2 external-load-only strength model, and the Milestone 3
joint stiffness/friction/required-preload results -- all reused
unchanged. Answers a question none of the prior milestones directly
answer: does the preload required to prevent joint separation/slip
remain structurally acceptable for the selected bolt, once preload and
launch loads are assessed TOGETHER?

Milestone 6 checks whether the preload required to keep the joint
closed and resist slip is itself compatible with bolt proof/tensile/
shear strength. Torque, preload scatter, bearing, prying, thread
failure, and fatigue remain deferred (see Milestone 4/5 for a separate,
force-based proof/yield installation-window screen and local-joint
checks).

Run with:
    python examples/preload_compatible_bolt_sizing.py
"""

from payload_bolts import (
    BoltMaterial,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    PreloadState,
    assess_bolt_group_strength,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    required_preload,
)
from payload_bolts.preloaded_strength import (
    evaluate_preloaded_candidates,
    select_smallest_passing_preloaded_bolt,
)
from payload_bolts.strength import NoFeasibleCandidateError

# ---------------------------------------------------------------------------
# Same Milestone 1 group-load case as the earlier examples.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5  # m
pattern = circular_pattern(N_BOLTS, RADIUS)

load = InterfaceLoad(
    Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0
)
group_load_result = distribute_loads(pattern, load)

# Milestone 3 joint stiffness/friction (unchanged, C=0.2, mu=0.2).
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)

# Milestone 6 material: same tensile/shear allowables as Milestone 2,
# PLUS the new illustrative proof/preload allowable (600 MPa).
MATERIAL = BoltMaterial(
    name="Illustrative high-strength steel bolt",
    tensile_allowable=800e6,
    shear_allowable=480e6,
    proof_allowable=600e6,
)

CANDIDATE_DIAMETERS_MM = [5.0, 6.0, 8.0, 10.0, 12.0]
CANDIDATES = [circular_unthreaded_bolt(d / 1000.0) for d in CANDIDATE_DIAMETERS_MM]
PRELOAD_FACTOR = 1.2


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt(m):
    return "n/a" if m is None else f"{m:+.3f}"


def main() -> None:
    print("PRELOAD-COMPATIBLE BOLT SIZING (Milestone 6)")
    print("Reuses the exact Milestone 1-3 canonical load case, joint stiffness, and friction.")

    max_shear = group_load_result.max_shear_bolt()
    max_tensile = group_load_result.max_tensile_bolt()

    _rule("CANONICAL JOINT LOADS")
    print(f"  governing external tensile bolt/load : index {max_tensile.index}, T = {max_tensile.axial_total:,.1f} N")
    print(f"  max shear bolt/load                  : index {max_shear.index}, V = {max_shear.shear_resultant:,.1f} N")

    req = required_preload(group_load_result, STIFFNESS, FRICTION)
    selected_preload = req.overall_required * PRELOAD_FACTOR

    _rule("JOINT REQUIREMENT")
    print(f"  C (load fraction)          = {STIFFNESS.C:.3f}")
    print(f"  mu (friction coefficient)  = {FRICTION.friction_coefficient:.3f}")
    print(f"  required separation preload = {req.separation_required:>12,.1f} N/bolt")
    print(f"  required slip preload        = {req.slip_required:>12,.1f} N/bolt")
    print(f"  required overall preload     = {req.overall_required:>12,.1f} N/bolt")
    print(f"  selected preload ({PRELOAD_FACTOR:.1f}x)   = {selected_preload:>12,.1f} N/bolt")

    _rule("MATERIAL")
    print(f"  tensile allowable = {MATERIAL.tensile_allowable / 1e6:,.0f} MPa")
    print(f"  shear allowable   = {MATERIAL.shear_allowable / 1e6:,.0f} MPa")
    print(f"  proof/preload allowable = {MATERIAL.proof_allowable / 1e6:,.0f} MPa (illustrative)")

    m2_section_8mm = circular_unthreaded_bolt(0.008)
    m2_strength = assess_bolt_group_strength(group_load_result, m2_section_8mm, MATERIAL)
    _rule("MILESTONE 2 REFERENCE")
    print(f"  external-load-only smallest passing bolt = 8 mm (governing bolt {m2_strength.governing_bolt_index}, "
          f"mode {m2_strength.governing_mode}, margin {m2_strength.governing_margin:+.3f})")

    _rule("CANDIDATE TABLE (selected preload applied to every candidate)")
    header = (
        f"{'d_mm':>5} {'preload_sig':>11} {'MS_preload':>10} {'MS_t_svc':>9} {'MS_s_svc':>9} "
        f"{'MS_int':>8} {'proof_ceil':>11} {'int_ceil':>10} {'window':>7} {'gov_bolt':>8} {'gov_mode':>11} {'PASS':>6}"
    )
    print(header)
    print("-" * len(header))

    results = evaluate_preloaded_candidates(group_load_result, CANDIDATES, MATERIAL, STIFFNESS, FRICTION, PRELOAD_FACTOR)
    for r in results:
        d_mm = r.bolt_section.nominal_diameter * 1000.0
        s = r.strength
        min_preload_ms = min(b.preload_margin for b in s.bolts if b.preload_margin is not None)
        min_tensile_ms = min((b.service_tensile_margin for b in s.bolts if b.service_tensile_margin is not None), default=None)
        min_shear_ms = min((b.service_shear_margin for b in s.bolts if b.service_shear_margin is not None), default=None)
        min_interaction_ms = min((b.interaction_margin for b in s.bolts if b.interaction_margin is not None), default=None)
        proof_ceil = r.window.ceilings.proof_ceiling
        int_ceil = r.window.ceilings.interaction_ceiling
        verdict = "PASS" if r.passed else "FAIL"
        print(
            f"{d_mm:>5.1f} {s.bolts[0].preload_stress/1e6:>10.1f}M {_fmt(min_preload_ms):>10} "
            f"{_fmt(min_tensile_ms):>9} {_fmt(min_shear_ms):>9} {_fmt(min_interaction_ms):>8} "
            f"{proof_ceil/1e3:>10.1f}k {(int_ceil/1e3 if int_ceil is not None else float('nan')):>9.1f}k "
            f"{'YES' if r.window.feasible else 'NO':>7} {s.governing_bolt_index:>8} {str(s.governing_mode):>11} {verdict:>6}"
        )

    _rule("SELECTED")
    try:
        selected = select_smallest_passing_preloaded_bolt(
            group_load_result, CANDIDATES, MATERIAL, STIFFNESS, FRICTION, PRELOAD_FACTOR
        )
        d_mm = selected.bolt_section.nominal_diameter * 1000.0
        print(f"  smallest passing preload-compatible bolt = {d_mm:.1f} mm")
        print(f"  governing bolt                             = index {selected.strength.governing_bolt_index}")
        print(f"  governing mode                             = {selected.strength.governing_mode}")
        print(f"  minimum integrated margin                  = {selected.strength.governing_margin:+.3f}")
        if d_mm != 8.0:
            print(
                "\n  COMPARISON: Milestone 2 selected 8 mm (external-load-only); Milestone 6 selects "
                f"{d_mm:.1f} mm once preload is integrated -- preload consumed enough tensile/proof/"
                "interaction capacity that 8 mm no longer passes at the selected (1.2x required) preload."
            )
        else:
            print(
                "\n  COMPARISON: Milestone 2 and Milestone 6 both select 8 mm -- preload reduces margin "
                "but not enough to change the bolt-size selection."
            )
    except NoFeasibleCandidateError as exc:
        print(f"  NO FEASIBLE CANDIDATE: {exc}")

    # -----------------------------------------------------------------
    # Sensitivity studies
    # -----------------------------------------------------------------
    from payload_bolts import assess_preloaded_joint
    from payload_bolts.preloaded_strength import assess_preloaded_bolt_strength

    selected_diam_mm = selected.bolt_section.nominal_diameter * 1000.0
    selected_section = selected.bolt_section

    _rule(f"PRELOAD SENSITIVITY (candidate: {selected_diam_mm:.1f} mm)")
    header = f"{'factor':>7} {'preload/bolt':>13} {'M3 slip MS':>11} {'preload MS':>11} {'interaction MS':>15} {'overall':>8}"
    print(header)
    print("-" * len(header))
    for factor in (1.0, 1.1, 1.2, 1.3, 1.5, 2.0):
        preload_val = req.overall_required * factor
        ps = PreloadState(preload_per_bolt=preload_val)
        joint = assess_preloaded_joint(group_load_result, ps, STIFFNESS, FRICTION)
        strength = assess_preloaded_bolt_strength(group_load_result, ps, STIFFNESS, FRICTION, selected_section, MATERIAL)
        min_int_ms = min((b.interaction_margin for b in strength.bolts if b.interaction_margin is not None), default=None)
        min_preload_ms = min(b.preload_margin for b in strength.bolts if b.preload_margin is not None)
        print(
            f"{factor:>7.2f} {preload_val:>13,.1f} {_fmt(joint.min_slip_margin):>11} "
            f"{_fmt(min_preload_ms):>11} {_fmt(min_int_ms):>15} {'PASS' if strength.passed else 'FAIL':>8}"
        )

    _rule("FRICTION SENSITIVITY")
    header = f"{'mu':>6} {'required preload':>17} {'max allowable preload':>22} {'window width':>13} {'feasible':>9}"
    print(header)
    print("-" * len(header))
    from payload_bolts.preloaded_strength import preload_capacity_window

    for mu in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40):
        fric = FrictionModel(friction_coefficient=mu)
        window = preload_capacity_window(group_load_result, selected_section, MATERIAL, STIFFNESS, fric)
        ceiling = window.ceilings.overall_ceiling
        width_str = f"{window.window_width:,.1f}" if window.window_width is not None else "n/a"
        print(
            f"{mu:>6.2f} {window.required_preload:>17,.1f} "
            f"{(ceiling if ceiling is not None else float('nan')):>22,.1f} {width_str:>13} "
            f"{'YES' if window.feasible else 'NO':>9}"
        )

    _rule("C SENSITIVITY")
    header = f"{'C':>6} {'required preload':>17} {'selected preload':>17} {'max total tension':>18} {'interaction MS':>15} {'PASS/FAIL':>10}"
    print(header)
    print("-" * len(header))
    kb = STIFFNESS.bolt_stiffness
    for c_val in (0.10, 0.20, 0.30, 0.40):
        km = kb * (1.0 - c_val) / c_val
        stiffness_c = JointStiffness(bolt_stiffness=kb, member_stiffness=km)
        req_c = required_preload(group_load_result, stiffness_c, FRICTION)
        selected_c = req_c.overall_required * PRELOAD_FACTOR
        ps_c = PreloadState(preload_per_bolt=selected_c)
        strength_c = assess_preloaded_bolt_strength(group_load_result, ps_c, stiffness_c, FRICTION, selected_section, MATERIAL)
        max_total_tension = max(b.total_bolt_tension for b in strength_c.bolts)
        min_int_ms = min((b.interaction_margin for b in strength_c.bolts if b.interaction_margin is not None), default=None)
        print(
            f"{c_val:>6.2f} {req_c.overall_required:>17,.1f} {selected_c:>17,.1f} "
            f"{max_total_tension:>18,.1f} {_fmt(min_int_ms):>15} {'PASS' if strength_c.passed else 'FAIL':>10}"
        )
    print(
        "\n  Interpretation: higher C preserves more clamp force (better separation/slip margin)\n"
        "  but transfers more external tensile load into the bolt (worse tensile/interaction\n"
        "  margin) -- a genuine closure-vs-bolt-strength trade, not a free improvement."
    )


if __name__ == "__main__":
    main()
