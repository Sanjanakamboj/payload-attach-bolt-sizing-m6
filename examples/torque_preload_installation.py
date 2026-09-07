"""Torque-to-preload installation screening (Milestone 7).

Converts the Milestone 6 FORCE-based preload feasibility window into a
preliminary INSTALLATION feasibility study: given a simple illustrative
torque-to-preload relation and a symmetric preload-scatter allowance,
what installation torque range remains inside the existing force
window, and does the canonical +/-20% scatter assumption even survive
the check?

This is a first-order torque-CONTROL screening model, not an
installation specification. Nut factor and scatter are illustrative;
no detailed thread-friction or torque-angle model is included.

Run with:
    python examples/torque_preload_installation.py
"""

from dataclasses import replace

from payload_bolts import (
    BoltMaterial,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    NoRobustTorqueWindowError,
    TorquePreloadModel,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    max_allowable_scatter,
    preload_capacity_window,
    required_preload,
    select_installation_target,
    torque_installation_window,
)

# ---------------------------------------------------------------------------
# Canonical STM-08 load case, joint stiffness, and friction -- unchanged
# from Milestones 1-6.
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5  # m
pattern = circular_pattern(N_BOLTS, RADIUS)
load = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
group_load_result = distribute_loads(pattern, load)

STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.20, number_of_faying_surfaces=1)
MATERIAL = BoltMaterial(
    name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6
)

# Milestone 6 selected preload-compatible bolt (NOT the M2 8mm result).
SELECTED_DIAMETER_MM = 10.0
SELECTED_SECTION = circular_unthreaded_bolt(SELECTED_DIAMETER_MM / 1000.0)

# Illustrative torque model.
NUT_FACTOR = 0.20  # illustrative nut factor
CANONICAL_SCATTER = 0.20  # illustrative +/-20% preload scatter
MODEL = TorquePreloadModel(nut_factor=NUT_FACTOR, preload_scatter_fraction=CANONICAL_SCATTER)


def _rule(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def _fmt(x):
    return "n/a" if x is None else f"{x:+.3f}"


def main() -> None:
    print("TORQUE-TO-PRELOAD INSTALLATION SCREENING (Milestone 7)")
    print("Converts the Milestone 6 FORCE preload window into a torque-controlled installation screen.")

    req = required_preload(group_load_result, STIFFNESS, FRICTION)
    window_10mm = preload_capacity_window(group_load_result, SELECTED_SECTION, MATERIAL, STIFFNESS, FRICTION)
    f_max_10mm = window_10mm.ceilings.overall_ceiling

    _rule("JOINT REQUIREMENT")
    print(f"  selected bolt diameter        = {SELECTED_DIAMETER_MM:.1f} mm")
    print(f"  required separation preload   = {req.separation_required:>12,.1f} N/bolt")
    print(f"  required slip preload         = {req.slip_required:>12,.1f} N/bolt")
    print(f"  required overall preload      = {req.overall_required:>12,.1f} N/bolt")
    print(f"  bolt maximum allowable preload = {f_max_10mm:>12,.1f} N/bolt ({window_10mm.ceilings.overall_governing_constraint} governs)")
    print(f"  force-window width             = {f_max_10mm - req.overall_required:>12,.1f} N")

    _rule("TORQUE MODEL")
    print(f"  nut factor K       = {NUT_FACTOR:.2f} (illustrative)")
    print(f"  preload scatter    = +/-{CANONICAL_SCATTER * 100:.0f}% (illustrative)")

    s_max = max_allowable_scatter(req.overall_required, f_max_10mm)
    _rule("SCATTER CAPABILITY")
    print(f"  maximum allowable scatter s_max = {s_max:.4f} ({s_max * 100:.1f}%)")
    canonical_feasible = CANONICAL_SCATTER <= s_max
    print(f"  canonical +/-{CANONICAL_SCATTER*100:.0f}% scatter feasible? = {'YES' if canonical_feasible else 'NO'}")

    window = torque_installation_window(req.overall_required, f_max_10mm, SELECTED_SECTION.nominal_diameter, MODEL)
    _rule("ROBUST TORQUE WINDOW")
    print(f"  torque_min    = {window.torque_min:>8.3f} N*m")
    print(f"  torque_max    = {window.torque_max:>8.3f} N*m")
    print(f"  window width  = {window.torque_window_width:>8.3f} N*m")
    print(f"  feasible      = {window.feasible}")

    _rule("SELECTED INSTALLATION TARGET")
    if window.feasible:
        target = select_installation_target(window)
        print(f"  selected torque (illustrative midpoint target) = {target.torque_selected:.3f} N*m")
        print(f"  nominal preload    = {target.nominal_preload_selected:>12,.1f} N")
        print(f"  minimum achieved   = {target.low_preload:>12,.1f} N")
        print(f"  maximum achieved   = {target.high_preload:>12,.1f} N")
        print(f"  reserve to lower bound (required) = {target.reserve_to_lower_bound:>10,.1f} N")
        print(f"  reserve to upper bound (max)       = {target.reserve_to_upper_bound:>10,.1f} N")
    else:
        print("  NO ROBUST TORQUE WINDOW EXISTS at this scatter -- installation is not")
        print("  reliably achievable with this nut factor/scatter combination.")

    # -----------------------------------------------------------------
    # 8 / 10 / 12 mm comparison
    # -----------------------------------------------------------------
    _rule("COMPARISON: 8 / 10 / 12 mm")
    header = f"{'d_mm':>5} {'F_required':>11} {'F_max':>11} {'s_max':>8} {'+/-20% robust?':>15} {'torque window':>22}"
    print(header)
    print("-" * len(header))
    for d_mm in (8, 10, 12):
        sec = circular_unthreaded_bolt(d_mm / 1000.0)
        w = preload_capacity_window(group_load_result, sec, MATERIAL, STIFFNESS, FRICTION)
        ceiling = w.ceilings.overall_ceiling
        s_max_d = max_allowable_scatter(req.overall_required, ceiling)
        tw = torque_installation_window(req.overall_required, ceiling, d_mm / 1000.0, MODEL)
        window_str = f"[{tw.torque_min:.1f}, {tw.torque_max:.1f}] N*m" if tw.feasible else "INFEASIBLE"
        print(
            f"{d_mm:>5.0f} {req.overall_required:>11,.1f} {ceiling:>11,.1f} {s_max_d:>8.4f} "
            f"{'YES' if tw.feasible else 'NO':>15} {window_str:>22}"
        )
    print(
        "\n  8 mm passes Milestone 2 external-load-only strength and even the bare Milestone 6\n"
        "  force window, but its s_max is tiny (~1.5%) -- at the canonical +/-20% installation\n"
        "  scatter its torque window is INFEASIBLE. 10 mm and 12 mm both remain robust at\n"
        "  +/-20%. Bolt size here is driven by INSTALLATION ROBUSTNESS, not external strength."
    )

    # -----------------------------------------------------------------
    # Scatter sensitivity
    # -----------------------------------------------------------------
    _rule("SCATTER SENSITIVITY (10 mm)")
    header = f"{'scatter':>8} {'T_min':>9} {'T_max':>9} {'width':>9} {'PASS/FAIL':>10}"
    print(header)
    print("-" * len(header))
    for s in (0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
        m = TorquePreloadModel(nut_factor=NUT_FACTOR, preload_scatter_fraction=s)
        w = torque_installation_window(req.overall_required, f_max_10mm, SELECTED_SECTION.nominal_diameter, m)
        print(f"{s:>8.2f} {w.torque_min:>9.3f} {w.torque_max:>9.3f} {w.torque_window_width:>9.3f} {'PASS' if w.feasible else 'FAIL':>10}")

    # -----------------------------------------------------------------
    # Nut-factor sensitivity
    # -----------------------------------------------------------------
    _rule("NUT-FACTOR SENSITIVITY (10 mm, scatter fixed at 20%)")
    header = f"{'K':>6} {'T_min':>9} {'T_max':>9} {'selected T':>11}"
    print(header)
    print("-" * len(header))
    for k in (0.12, 0.15, 0.18, 0.20, 0.25, 0.30):
        m = TorquePreloadModel(nut_factor=k, preload_scatter_fraction=CANONICAL_SCATTER)
        w = torque_installation_window(req.overall_required, f_max_10mm, SELECTED_SECTION.nominal_diameter, m)
        sel_t = select_installation_target(w).torque_selected if w.feasible else None
        print(f"{k:>6.2f} {w.torque_min:>9.3f} {w.torque_max:>9.3f} {(f'{sel_t:.3f}' if sel_t is not None else 'n/a'):>11}")
    print("\n  Both bounds scale linearly with K (verified); K changes torque, not the force window or s_max.")

    # -----------------------------------------------------------------
    # Joint-friction sensitivity
    # -----------------------------------------------------------------
    _rule("JOINT-FRICTION SENSITIVITY (10 mm, scatter fixed at 20%)")
    header = f"{'mu':>6} {'F_required':>11} {'s_max':>8} {'+/-20% feasible?':>16} {'selected T':>11}"
    print(header)
    print("-" * len(header))
    for mu in (0.10, 0.15, 0.20, 0.25, 0.30, 0.40):
        fric_mu = FrictionModel(friction_coefficient=mu)
        req_mu = required_preload(group_load_result, STIFFNESS, fric_mu)
        w_mu = preload_capacity_window(group_load_result, SELECTED_SECTION, MATERIAL, STIFFNESS, fric_mu)
        ceiling_mu = w_mu.ceilings.overall_ceiling
        s_max_mu = max_allowable_scatter(req_mu.overall_required, ceiling_mu)
        tw_mu = torque_installation_window(req_mu.overall_required, ceiling_mu, SELECTED_SECTION.nominal_diameter, MODEL)
        sel_t = select_installation_target(tw_mu).torque_selected if tw_mu.feasible else None
        print(
            f"{mu:>6.2f} {req_mu.overall_required:>11,.1f} {s_max_mu:>8.4f} "
            f"{'YES' if tw_mu.feasible else 'NO':>16} {(f'{sel_t:.3f}' if sel_t is not None else 'n/a'):>11}"
        )

    _rule("ENGINEERING INTERPRETATION")
    print(
        "  Milestone 6 established a force-based preload window [F_required, F_max]. Torque\n"
        "  control does not deliver a single preload -- it delivers a SCATTER BAND around a\n"
        "  nominal value, and robust installation requires the ENTIRE band to remain inside\n"
        "  that force window, not just the nominal value. An acceptable force window can\n"
        "  still be too narrow for a torque-controlled installation once scatter is included\n"
        "  (8 mm here). Larger bolts generally raise the upper preload ceiling and widen\n"
        "  installation robustness. Lower faying-surface friction (mu) raises the required\n"
        "  preload and shrinks robustness. The nut factor K changes the torque needed for a\n"
        "  given preload but does NOT change the underlying force window or s_max. Preload\n"
        "  scatter is a major, independent design variable. This remains a preliminary\n"
        "  torque-control screening model, not an installation specification -- no detailed\n"
        "  thread/under-head friction split, torque-angle tightening, or measurement method\n"
        "  is modeled."
    )


if __name__ == "__main__":
    main()
