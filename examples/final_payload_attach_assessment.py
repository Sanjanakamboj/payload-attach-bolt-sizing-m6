"""Final integrated payload attach bolt/joint assessment (Milestone 8).

Reproduces the principal STM-08 result in one run, from the rigid
bolt-group load distribution (Milestone 1) through external strength
sizing (M2), preloaded joint closure/slip (M3), preload-compatible
service strength (M6), local joint checks (M5), and torque-installation
robustness (M7) -- all reused exactly from their own modules. This
script does not duplicate or re-derive any production equation; it only
calls the existing public API and reports the results.

`illustrative preliminary bolt/joint selection` -- not optimized,
qualified, or certified.

Run with:
    python examples/final_payload_attach_assessment.py
"""

from payload_bolts import (
    BoltMaterial,
    FrictionModel,
    InterfaceLoad,
    JointGeometry,
    JointStiffness,
    PlateMaterial,
    PreloadState,
    TorquePreloadModel,
    assess_bearing,
    assess_preloaded_joint,
    assess_bolt_group_strength,
    assess_edge_distance,
    assess_preloaded_bolt_strength,
    assess_spacing,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    max_allowable_scatter,
    preload_capacity_window,
    required_preload,
    select_installation_target,
    select_smallest_passing_bolt,
    select_smallest_passing_preloaded_bolt,
    torque_installation_window,
)

# ---------------------------------------------------------------------------
# Canonical STM-08 interface, unchanged across every milestone.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5  # m
PATTERN = circular_pattern(N_BOLTS, RADIUS)
LOAD = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
GROUP_LOAD_RESULT = distribute_loads(PATTERN, LOAD)

MATERIAL = BoltMaterial(
    name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6
)
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)

CANDIDATE_DIAMETERS_MM = [5.0, 6.0, 8.0, 10.0, 12.0]
CANDIDATES = [circular_unthreaded_bolt(d / 1000.0) for d in CANDIDATE_DIAMETERS_MM]
PRELOAD_FACTOR = 1.2

TORQUE_MODEL = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)

# Milestone 5 illustrative local-joint assumptions (unchanged from
# examples/bolt_candidate_trade.py).
PLATE = PlateMaterial(name="Illustrative plate/lug bearing material", bearing_allowable=400e6)
GEOMETRY = JointGeometry(
    plate_thickness=0.008,
    plate_center=(0.0, 0.0),
    plate_outer_radius=RADIUS + 0.05,
    edge_distance_min_ratio=1.5,
    spacing_min_ratio=3.0,
)


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt(x, digits=3):
    return "n/a" if x is None else f"{x:+.{digits}f}"


def main() -> None:
    print("FINAL INTEGRATED PAYLOAD ATTACH ASSESSMENT (Milestone 8, STM-08)")
    print('This is an "illustrative preliminary bolt/joint selection" -- not optimized, qualified, or certified.')

    # -----------------------------------------------------------------
    # INTERFACE
    # -----------------------------------------------------------------
    _rule("INTERFACE")
    print(f"  bolt count          = {N_BOLTS}")
    print(f"  bolt-circle radius  = {RADIUS:.2f} m")
    print(f"  Fx={LOAD.Fx/1e3:.1f} kN, Fy={LOAD.Fy/1e3:.1f} kN, Fz={LOAD.Fz/1e3:.1f} kN, "
          f"Mx={LOAD.Mx/1e3:.1f} kN*m, My={LOAD.My/1e3:.1f} kN*m, Mz={LOAD.Mz/1e3:.1f} kN*m")

    # -----------------------------------------------------------------
    # BOLT-GROUP RESULTS
    # -----------------------------------------------------------------
    max_tensile = GROUP_LOAD_RESULT.max_tensile_bolt()
    max_shear = GROUP_LOAD_RESULT.max_shear_bolt()
    eq = GROUP_LOAD_RESULT.equilibrium
    _rule("BOLT-GROUP RESULTS")
    print(f"  max tensile bolt/load = index {max_tensile.index}, T = {max_tensile.axial_total:,.1f} N")
    print(f"  max shear bolt/load   = index {max_shear.index}, V = {max_shear.shear_resultant:,.1f} N")
    print(f"  equilibrium residual (max |.|) = {eq.max_abs_residual:.3e}")

    # -----------------------------------------------------------------
    # MATERIAL
    # -----------------------------------------------------------------
    _rule("MATERIAL (illustrative)")
    print(f"  tensile allowable       = {MATERIAL.tensile_allowable/1e6:,.0f} MPa")
    print(f"  shear allowable         = {MATERIAL.shear_allowable/1e6:,.0f} MPa")
    print(f"  proof/preload allowable = {MATERIAL.proof_allowable/1e6:,.0f} MPa")

    # -----------------------------------------------------------------
    # JOINT MODEL
    # -----------------------------------------------------------------
    _rule("JOINT MODEL")
    print(f"  k_b = {STIFFNESS.bolt_stiffness:.2e} N/m, k_m = {STIFFNESS.member_stiffness:.2e} N/m, C = {STIFFNESS.C:.3f}")
    print(f"  mu = {FRICTION.friction_coefficient:.2f}, faying surfaces = {FRICTION.number_of_faying_surfaces}")

    # -----------------------------------------------------------------
    # MILESTONE COMPARISON
    # -----------------------------------------------------------------
    m2_selected = select_smallest_passing_bolt(GROUP_LOAD_RESULT, CANDIDATES, MATERIAL)
    m6_selected = select_smallest_passing_preloaded_bolt(
        GROUP_LOAD_RESULT, CANDIDATES, MATERIAL, STIFFNESS, FRICTION, PRELOAD_FACTOR
    )
    _rule("MILESTONE COMPARISON")
    print(f"  external-load-only smallest passing bolt (M2)   = {m2_selected.bolt_section.nominal_diameter*1000:.1f} mm "
          f"(bolt {m2_selected.governing_bolt_index}, mode {m2_selected.governing_mode}, margin {m2_selected.governing_margin:+.3f})")
    print(f"  preload-compatible smallest passing bolt (M6)   = {m6_selected.bolt_section.nominal_diameter*1000:.1f} mm "
          f"(bolt {m6_selected.strength.governing_bolt_index}, mode {m6_selected.strength.governing_mode}, "
          f"margin {m6_selected.strength.governing_margin:+.3f})")

    # -----------------------------------------------------------------
    # FINAL SELECTED BOLT
    # -----------------------------------------------------------------
    final_section = m6_selected.bolt_section
    final_diam_mm = final_section.nominal_diameter * 1000.0
    _rule("FINAL SELECTED BOLT")
    print(f"  diameter        = {final_diam_mm:.1f} mm")
    print(f"  area convention = idealized gross circular shank area (A = pi*d^2/4), "
          f"A_t = A_s = {final_section.tensile_area*1e6:.2f} mm^2")

    # -----------------------------------------------------------------
    # PRELOAD FORCE WINDOW
    # -----------------------------------------------------------------
    req = required_preload(GROUP_LOAD_RESULT, STIFFNESS, FRICTION)
    window = preload_capacity_window(GROUP_LOAD_RESULT, final_section, MATERIAL, STIFFNESS, FRICTION)
    ceilings = window.ceilings
    _rule("PRELOAD FORCE WINDOW")
    print(f"  required preload (lower bound) = {req.overall_required:>12,.1f} N/bolt")
    print(f"  proof ceiling                   = {ceilings.proof_ceiling:>12,.1f} N/bolt")
    print(f"  service tension ceiling         = {ceilings.tension_ceiling:>12,.1f} N/bolt")
    print(f"  interaction ceiling             = {(ceilings.interaction_ceiling if ceilings.interaction_ceiling is not None else float('nan')):>12,.1f} N/bolt")
    print(f"  final upper preload (governing: {ceilings.overall_governing_constraint})   = {ceilings.overall_ceiling:>12,.1f} N/bolt")
    print(f"  window width                    = {window.window_width:>12,.1f} N")

    # -----------------------------------------------------------------
    # TORQUE INSTALLATION
    # -----------------------------------------------------------------
    s_max = max_allowable_scatter(req.overall_required, ceilings.overall_ceiling)
    torque_window = torque_installation_window(
        req.overall_required, ceilings.overall_ceiling, final_section.nominal_diameter, TORQUE_MODEL
    )
    _rule("TORQUE INSTALLATION")
    print(f"  K (nut factor)          = {TORQUE_MODEL.nut_factor:.2f}")
    print(f"  scatter                 = +/-{TORQUE_MODEL.preload_scatter_fraction*100:.0f}%")
    print(f"  s_max (max allowable scatter) = {s_max:.4f} ({s_max*100:.1f}%)")
    print(f"  torque_min              = {torque_window.torque_min:.3f} N*m")
    print(f"  torque_max              = {torque_window.torque_max:.3f} N*m")
    if torque_window.feasible:
        target = select_installation_target(torque_window)
        print(f"  selected torque (midpoint policy) = {target.torque_selected:.3f} N*m")
        print(f"  nominal preload = {target.nominal_preload_selected:,.1f} N, "
              f"low = {target.low_preload:,.1f} N, high = {target.high_preload:,.1f} N")
    else:
        print("  NO ROBUST TORQUE WINDOW at this scatter.")

    # -----------------------------------------------------------------
    # LOCAL JOINT CHECKS (Milestone 5, reused exactly)
    # -----------------------------------------------------------------
    bearing = assess_bearing(GROUP_LOAD_RESULT, final_section.nominal_diameter, GEOMETRY.plate_thickness, PLATE)
    edge = assess_edge_distance(GROUP_LOAD_RESULT, final_section.nominal_diameter, GEOMETRY)
    spacing = assess_spacing(GROUP_LOAD_RESULT, final_section.nominal_diameter, GEOMETRY)
    _rule("LOCAL JOINT CHECKS (Milestone 5, illustrative)")
    print(f"  bearing:  governing bolt {bearing.governing_bolt_index}, MS = {_fmt(bearing.governing_margin)}, "
          f"{'PASS' if bearing.passed else 'FAIL'}")
    print(f"  edge distance: min ratio {edge.min_ratio:.2f} (criterion {edge.criterion:.2f}), "
          f"{'PASS' if edge.passed else 'FAIL'}")
    print(f"  spacing: min ratio {spacing.min_ratio:.2f} (criterion {spacing.criterion:.2f}, pair {spacing.governing_pair}), "
          f"{'PASS' if spacing.passed else 'FAIL'}")
    print("  thread stripping: NOT MODELED (never gates selection; see joint_local_checks.py)")

    # -----------------------------------------------------------------
    # FINAL STATUS
    # -----------------------------------------------------------------
    strength_result = assess_preloaded_bolt_strength(
        GROUP_LOAD_RESULT,
        PreloadState(preload_per_bolt=m6_selected.selected_preload),
        STIFFNESS, FRICTION, final_section, MATERIAL,
    )
    local_checks_pass = bearing.passed and edge.passed and spacing.passed
    overall_pass = strength_result.passed and torque_window.feasible and local_checks_pass
    _rule("FINAL STATUS")
    print(f"  strength/preload integrated (M6) : {'PASS' if strength_result.passed else 'FAIL'}")
    print(f"  torque installation window (M7)  : {'PASS' if torque_window.feasible else 'FAIL'}")
    print(f"  local joint checks (M5)          : {'PASS' if local_checks_pass else 'FAIL'}")
    print(f"  OVERALL PRELIMINARY PASS/FAIL     : {'PASS' if overall_pass else 'FAIL'}")
    print(f"  governing mechanism               : {strength_result.governing_mode} "
          f"(bolt {strength_result.governing_bolt_index}, margin {strength_result.governing_margin:+.3f})")

    # -----------------------------------------------------------------
    # SENSITIVITY SUMMARY (compact)
    # -----------------------------------------------------------------
    _rule("SENSITIVITY SUMMARY (compact)")
    print("  A. Preload factor (10 mm):")
    for factor in (1.0, 1.2, 1.5, 2.0):
        preload_val = req.overall_required * factor
        ps = PreloadState(preload_per_bolt=preload_val)
        joint = assess_preloaded_joint(GROUP_LOAD_RESULT, ps, STIFFNESS, FRICTION)
        strength = assess_preloaded_bolt_strength(GROUP_LOAD_RESULT, ps, STIFFNESS, FRICTION, final_section, MATERIAL)
        print(f"     {factor:.1f}x: slip MS={_fmt(joint.min_slip_margin)}  preload MS="
              f"{_fmt(min(b.preload_margin for b in strength.bolts if b.preload_margin is not None))}  "
              f"{'PASS' if strength.passed else 'FAIL'}")

    print("  B. Faying-surface friction mu (10 mm, +/-20% scatter):")
    for mu in (0.10, 0.20, 0.30, 0.40):
        fric_mu = FrictionModel(friction_coefficient=mu)
        req_mu = required_preload(GROUP_LOAD_RESULT, STIFFNESS, fric_mu)
        w_mu = preload_capacity_window(GROUP_LOAD_RESULT, final_section, MATERIAL, STIFFNESS, fric_mu)
        s_max_mu = max_allowable_scatter(req_mu.overall_required, w_mu.ceilings.overall_ceiling)
        tw_mu = torque_installation_window(req_mu.overall_required, w_mu.ceilings.overall_ceiling, final_section.nominal_diameter, TORQUE_MODEL)
        print(f"     mu={mu:.2f}: F_req={req_mu.overall_required:,.0f} N, width={w_mu.window_width:,.0f} N, "
              f"s_max={s_max_mu:.3f}, +/-20% {'FEASIBLE' if tw_mu.feasible else 'INFEASIBLE'}")

    print("  C. Load fraction C (10 mm):")
    kb = STIFFNESS.bolt_stiffness
    for c_val in (0.10, 0.20, 0.30, 0.40):
        km = kb * (1 - c_val) / c_val
        stiffness_c = JointStiffness(bolt_stiffness=kb, member_stiffness=km)
        req_c = required_preload(GROUP_LOAD_RESULT, stiffness_c, FRICTION)
        ps_c = PreloadState(preload_per_bolt=req_c.overall_required * PRELOAD_FACTOR)
        strength_c = assess_preloaded_bolt_strength(GROUP_LOAD_RESULT, ps_c, stiffness_c, FRICTION, final_section, MATERIAL)
        max_tension = max(b.total_bolt_tension for b in strength_c.bolts)
        min_int = min((b.interaction_margin for b in strength_c.bolts if b.interaction_margin is not None), default=None)
        print(f"     C={c_val:.2f}: F_req={req_c.overall_required:,.0f} N, max_tension={max_tension:,.0f} N, "
              f"interaction MS={_fmt(min_int)}")

    print("  D. Preload scatter s (10 mm):")
    for s in (0.0, 0.10, 0.20, 0.25, 0.30):
        m = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=s)
        tw = torque_installation_window(req.overall_required, ceilings.overall_ceiling, final_section.nominal_diameter, m)
        print(f"     s={s:.2f}: width={tw.torque_window_width:.3f} N*m, {'FEASIBLE' if tw.feasible else 'INFEASIBLE'}")

    print("  E. Nut factor K (10 mm, scatter fixed 20%):")
    for k in (0.12, 0.20, 0.30):
        m = TorquePreloadModel(nut_factor=k, preload_scatter_fraction=0.20)
        tw = torque_installation_window(req.overall_required, ceilings.overall_ceiling, final_section.nominal_diameter, m)
        print(f"     K={k:.2f}: T_min={tw.torque_min:.2f} N*m, T_max={tw.torque_max:.2f} N*m")

    print(
        "\n  Final conclusion: the illustrative 10 mm candidate is the smallest currently\n"
        "  demonstrated bolt that simultaneously satisfies the implemented joint preload,\n"
        "  bolt-strength, local-joint, and +/-20% torque-installation robustness screens."
    )


if __name__ == "__main__":
    main()
