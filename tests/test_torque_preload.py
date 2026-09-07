"""Milestone 7 tests: torque-to-preload relation and preload-scatter
installation model.

Covers items A-AD from the Milestone 7 task brief. Existing Milestone
1-6 tests are left completely unmodified and must continue to pass
unchanged.
"""

import math

import pytest

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
    installed_preload_band,
    max_allowable_scatter,
    preload_capacity_window,
    preload_from_torque,
    required_preload,
    select_installation_target,
    torque_from_preload,
    torque_installation_window,
)


def _canonical_group_result():
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
    return distribute_loads(pattern, load)


CANONICAL_STIFFNESS = JointStiffness(bolt_stiffness=1e8, member_stiffness=4e8)  # C = 0.2
CANONICAL_FRICTION = FrictionModel(friction_coefficient=0.2)
CANONICAL_MATERIAL = BoltMaterial(
    name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6
)


def _canonical_force_window(diameter_m: float):
    group = _canonical_group_result()
    req = required_preload(group, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    sec = circular_unthreaded_bolt(diameter_m)
    window = preload_capacity_window(group, sec, CANONICAL_MATERIAL, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    return req.overall_required, window.ceilings.overall_ceiling


# ---------------------------------------------------------------------------
# A, B, C. TorquePreloadModel validation
# ---------------------------------------------------------------------------


def test_torque_preload_model_valid():
    m = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    assert m.nut_factor == 0.20
    assert m.preload_scatter_fraction == 0.20


def test_nut_factor_validation():
    with pytest.raises(ValueError):
        TorquePreloadModel(nut_factor=0.0, preload_scatter_fraction=0.2)
    with pytest.raises(ValueError):
        TorquePreloadModel(nut_factor=-0.1, preload_scatter_fraction=0.2)
    with pytest.raises(ValueError):
        TorquePreloadModel(nut_factor=float("nan"), preload_scatter_fraction=0.2)


def test_scatter_validation():
    with pytest.raises(ValueError):
        TorquePreloadModel(nut_factor=0.2, preload_scatter_fraction=-0.01)
    with pytest.raises(ValueError):
        TorquePreloadModel(nut_factor=0.2, preload_scatter_fraction=1.0)
    with pytest.raises(ValueError):
        TorquePreloadModel(nut_factor=0.2, preload_scatter_fraction=float("inf"))
    # boundary: exactly 0 is allowed
    TorquePreloadModel(nut_factor=0.2, preload_scatter_fraction=0.0)


# ---------------------------------------------------------------------------
# D, E. torque/preload hand calc + exact round trip
# ---------------------------------------------------------------------------


def test_torque_from_preload_hand_calc():
    # T = K*F*d = 0.2 * 30000 * 0.01 = 60 N*m
    t = torque_from_preload(preload=30_000.0, diameter=0.01, nut_factor=0.2)
    assert t == pytest.approx(60.0)


def test_preload_from_torque_hand_calc():
    f = preload_from_torque(torque=60.0, diameter=0.01, nut_factor=0.2)
    assert f == pytest.approx(30_000.0)


def test_torque_preload_round_trip_f_to_t_to_f():
    f0 = 24_142.1
    t = torque_from_preload(f0, 0.010, 0.20)
    f1 = preload_from_torque(t, 0.010, 0.20)
    assert f1 == pytest.approx(f0, rel=1e-12)


def test_torque_preload_round_trip_t_to_f_to_t():
    t0 = 83.3333
    f = preload_from_torque(t0, 0.010, 0.20)
    t1 = torque_from_preload(f, 0.010, 0.20)
    assert t1 == pytest.approx(t0, rel=1e-12)


def test_torque_preload_helper_validation():
    with pytest.raises(ValueError):
        torque_from_preload(preload=-1.0, diameter=0.01, nut_factor=0.2)
    with pytest.raises(ValueError):
        torque_from_preload(preload=1000.0, diameter=0.0, nut_factor=0.2)
    with pytest.raises(ValueError):
        torque_from_preload(preload=1000.0, diameter=0.01, nut_factor=0.0)
    with pytest.raises(ValueError):
        preload_from_torque(torque=-1.0, diameter=0.01, nut_factor=0.2)


# ---------------------------------------------------------------------------
# F, G, H. installed preload band hand calc / zero scatter / widening
# ---------------------------------------------------------------------------


def test_installed_preload_band_hand_calc():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    band = installed_preload_band(torque=60.0, diameter=0.01, model=model)
    assert band.nominal_preload == pytest.approx(30_000.0)
    assert band.min_preload == pytest.approx(24_000.0)
    assert band.max_preload == pytest.approx(36_000.0)


def test_zero_scatter_reproduces_nominal_exactly():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.0)
    band = installed_preload_band(torque=60.0, diameter=0.01, model=model)
    assert band.min_preload == pytest.approx(band.nominal_preload)
    assert band.max_preload == pytest.approx(band.nominal_preload)


def test_increasing_scatter_widens_preload_band():
    widths = []
    for s in (0.0, 0.1, 0.2, 0.3):
        model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=s)
        band = installed_preload_band(torque=60.0, diameter=0.01, model=model)
        widths.append(band.max_preload - band.min_preload)
    for i in range(len(widths) - 1):
        assert widths[i + 1] > widths[i]


# ---------------------------------------------------------------------------
# I, J, K, L. torque window bounds / feasibility
# ---------------------------------------------------------------------------


def test_torque_window_hand_calc_bounds():
    # Section 20: d=0.01, K=0.2, F_required=30000, F_max=50000, s=0.2
    # T_min = 75, T_max = 83.333...
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    window = torque_installation_window(30_000.0, 50_000.0, 0.010, model)
    assert window.torque_min == pytest.approx(75.0)
    assert window.torque_max == pytest.approx(83.333333333, rel=1e-9)
    assert window.feasible is True


def test_torque_window_infeasibility():
    # Very tight force window with large scatter -> infeasible.
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    window = torque_installation_window(29_000.0, 30_000.0, 0.010, model)
    assert window.feasible is False
    assert window.torque_window_width < 0.0


# ---------------------------------------------------------------------------
# M, N. exact zero-width boundary / slightly above
# ---------------------------------------------------------------------------


def test_exact_zero_width_boundary_passes():
    # At s = s_max exactly, window width == 0 (still PASS at boundary).
    required, max_p = 30_000.0, 50_000.0
    s_max = max_allowable_scatter(required, max_p)
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=s_max)
    window = torque_installation_window(required, max_p, 0.010, model)
    assert window.torque_window_width == pytest.approx(0.0, abs=1e-6)
    assert window.feasible is True


def test_slightly_above_boundary_fails():
    required, max_p = 30_000.0, 50_000.0
    s_max = max_allowable_scatter(required, max_p)
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=min(s_max * 1.05, 0.999))
    window = torque_installation_window(required, max_p, 0.010, model)
    assert window.torque_window_width < 0.0
    assert window.feasible is False


def test_slightly_below_boundary_gives_positive_window():
    required, max_p = 30_000.0, 50_000.0
    s_max = max_allowable_scatter(required, max_p)
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=s_max * 0.95)
    window = torque_installation_window(required, max_p, 0.010, model)
    assert window.torque_window_width > 0.0
    assert window.feasible is True


# ---------------------------------------------------------------------------
# O, P, Q. s_max hand calc / zero-width / force-infeasible propagation
# ---------------------------------------------------------------------------


def test_s_max_hand_calc():
    # Section 21: F_required=30kN, F_max=50kN -> s_max = 0.25
    s_max = max_allowable_scatter(30_000.0, 50_000.0)
    assert s_max == pytest.approx(0.25)


def test_s_max_zero_for_zero_width_force_window():
    s_max = max_allowable_scatter(40_000.0, 40_000.0)
    assert s_max == pytest.approx(0.0)


def test_force_window_infeasible_propagates_to_torque_infeasible():
    # F_max < F_required -> s_max negative, and torque window infeasible
    # for ANY valid scatter in [0, 1).
    s_max = max_allowable_scatter(50_000.0, 30_000.0)
    assert s_max < 0.0
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.0)
    window = torque_installation_window(50_000.0, 30_000.0, 0.010, model)
    assert window.feasible is False


# ---------------------------------------------------------------------------
# R, S, T. selected midpoint target inside the robust window
# ---------------------------------------------------------------------------


def test_selected_target_lies_inside_robust_window():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    window = torque_installation_window(30_000.0, 50_000.0, 0.010, model)
    target = select_installation_target(window)
    assert window.torque_min <= target.torque_selected <= window.torque_max


def test_selected_low_preload_meets_required():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    window = torque_installation_window(30_000.0, 50_000.0, 0.010, model)
    target = select_installation_target(window)
    assert target.low_preload >= window.required_preload - 1e-6
    assert target.reserve_to_lower_bound >= -1e-6


def test_selected_high_preload_within_max():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    window = torque_installation_window(30_000.0, 50_000.0, 0.010, model)
    target = select_installation_target(window)
    assert target.high_preload <= window.max_preload + 1e-6
    assert target.reserve_to_upper_bound >= -1e-6


def test_select_installation_target_raises_when_infeasible():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    window = torque_installation_window(29_000.0, 30_000.0, 0.010, model)
    with pytest.raises(NoRobustTorqueWindowError):
        select_installation_target(window)


# ---------------------------------------------------------------------------
# U, V. nut-factor scaling / s_max independence
# ---------------------------------------------------------------------------


def test_increasing_K_scales_both_torque_bounds_linearly():
    required, max_p = 30_000.0, 50_000.0
    ks = [0.12, 0.15, 0.18, 0.20, 0.25, 0.30]
    mins, maxs = [], []
    for k in ks:
        model = TorquePreloadModel(nut_factor=k, preload_scatter_fraction=0.20)
        w = torque_installation_window(required, max_p, 0.010, model)
        mins.append(w.torque_min)
        maxs.append(w.torque_max)
    for i in range(len(ks)):
        assert mins[i] == pytest.approx(mins[0] * ks[i] / ks[0], rel=1e-9)
        assert maxs[i] == pytest.approx(maxs[0] * ks[i] / ks[0], rel=1e-9)


def test_changing_K_leaves_s_max_unchanged():
    # s_max depends only on the force window, never on K.
    s_max = max_allowable_scatter(30_000.0, 50_000.0)
    assert s_max == pytest.approx(0.25)  # independent of any K used elsewhere


# ---------------------------------------------------------------------------
# W. increasing scatter shrinks torque window
# ---------------------------------------------------------------------------


def test_increasing_scatter_shrinks_torque_window_width():
    required, max_p = 30_000.0, 50_000.0
    widths = []
    for s in (0.0, 0.05, 0.10, 0.15, 0.20):
        model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=s)
        w = torque_installation_window(required, max_p, 0.010, model)
        widths.append(w.torque_window_width)
    for i in range(len(widths) - 1):
        assert widths[i + 1] < widths[i]


# ---------------------------------------------------------------------------
# X. bolt-size monotonicity of s_max (canonical STM-08 case)
# ---------------------------------------------------------------------------


def test_bolt_size_monotonicity_of_s_max():
    s_max_values = []
    for d_mm in (8, 10, 12):
        required, max_p = _canonical_force_window(d_mm / 1000.0)
        s_max_values.append(max_allowable_scatter(required, max_p))
    for i in range(len(s_max_values) - 1):
        assert s_max_values[i + 1] > s_max_values[i]


def test_8mm_infeasible_10mm_12mm_feasible_at_canonical_20pct_scatter():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    verdicts = {}
    for d_mm in (8, 10, 12):
        required, max_p = _canonical_force_window(d_mm / 1000.0)
        w = torque_installation_window(required, max_p, d_mm / 1000.0, model)
        verdicts[d_mm] = w.feasible
    assert verdicts[8] is False
    assert verdicts[10] is True
    assert verdicts[12] is True


# ---------------------------------------------------------------------------
# Y. low-friction joint case raises lower preload, reduces s_max
# ---------------------------------------------------------------------------


def test_low_friction_raises_required_preload_and_reduces_s_max():
    group = _canonical_group_result()
    sec = circular_unthreaded_bolt(0.010)
    s_max_values = []
    for mu in (0.40, 0.30, 0.25, 0.20, 0.15, 0.10):
        fric = FrictionModel(friction_coefficient=mu)
        req = required_preload(group, CANONICAL_STIFFNESS, fric)
        window = preload_capacity_window(group, sec, CANONICAL_MATERIAL, CANONICAL_STIFFNESS, fric)
        ceiling = window.ceilings.overall_ceiling
        s_max_values.append(max_allowable_scatter(req.overall_required, ceiling))
    for i in range(len(s_max_values) - 1):
        assert s_max_values[i + 1] < s_max_values[i]
    assert s_max_values[-1] < 0.0  # mu=0.10: force window itself infeasible


# ---------------------------------------------------------------------------
# Z. deterministic repeated results
# ---------------------------------------------------------------------------


def test_deterministic_repeated_results():
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    w1 = torque_installation_window(30_000.0, 50_000.0, 0.010, model)
    w2 = torque_installation_window(30_000.0, 50_000.0, 0.010, model)
    assert w1 == w2
    t1 = select_installation_target(w1)
    t2 = select_installation_target(w2)
    assert t1 == t2


# ---------------------------------------------------------------------------
# AA, AB, AC. no mutation of Milestone 1/3/6 results
# ---------------------------------------------------------------------------


def test_no_mutation_of_m1_load_result():
    group = _canonical_group_result()
    before_bolts = tuple(group.bolts)
    before_eq = group.equilibrium
    req = required_preload(group, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    sec = circular_unthreaded_bolt(0.010)
    window = preload_capacity_window(group, sec, CANONICAL_MATERIAL, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    torque_installation_window(req.overall_required, window.ceilings.overall_ceiling, 0.010, model)
    assert group.bolts == before_bolts
    assert group.equilibrium == before_eq


def test_no_mutation_of_m3_preload_requirement():
    group = _canonical_group_result()
    req_before = required_preload(group, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    sec = circular_unthreaded_bolt(0.010)
    preload_capacity_window(group, sec, CANONICAL_MATERIAL, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    torque_installation_window(req_before.overall_required, 47_000.0, 0.010, model)
    req_after = required_preload(group, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    assert req_before == req_after


def test_no_mutation_of_m6_preload_strength_window():
    group = _canonical_group_result()
    sec = circular_unthreaded_bolt(0.010)
    window_before = preload_capacity_window(group, sec, CANONICAL_MATERIAL, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    req = required_preload(group, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    model = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)
    torque_installation_window(req.overall_required, window_before.ceilings.overall_ceiling, 0.010, model)
    window_after = preload_capacity_window(group, sec, CANONICAL_MATERIAL, CANONICAL_STIFFNESS, CANONICAL_FRICTION)
    assert window_before == window_after
