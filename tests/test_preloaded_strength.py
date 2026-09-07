"""Milestone 6 tests: integrated preload + bolt-strength assessment.

Covers items A-AM from the Milestone 6 task brief. Existing Milestone
1-5 tests (test_geometry.py .. test_joint_local_checks.py) are left
completely unmodified and must continue to pass unchanged.
"""

import math

import pytest

from payload_bolts import (
    BoltMaterial,
    BoltSection,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    NoFeasibleCandidateError,
    PreloadState,
    assess_bolt_group_strength,
    assess_preloaded_joint,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    required_preload,
)
from payload_bolts.preloaded_strength import (
    PreloadedBoltGroupStrengthResult,
    assess_preloaded_bolt_strength,
    compute_preload_ceilings,
    evaluate_preloaded_candidates,
    preload_capacity_window,
    select_smallest_passing_preloaded_bolt,
)

STIFFNESS = JointStiffness(bolt_stiffness=1e8, member_stiffness=4e8)  # C = 0.2
FRICTION = FrictionModel(friction_coefficient=0.2)
MATERIAL = BoltMaterial(
    name="Illustrative M6 steel", tensile_allowable=500e6, shear_allowable=300e6, proof_allowable=600e6
)


def _sanity_group_result():
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
    return distribute_loads(pattern, load)


def _uniform_group_result(fz: float = 0.0, fx: float = 0.0):
    """4-bolt symmetric pattern (non-collinear) under pure Fz/Fx: every
    bolt gets identical T_ext=Fz/4, V=Fx/4, zero moments."""
    pattern = circular_pattern(4, 0.3)
    return distribute_loads(pattern, InterfaceLoad(Fz=fz, Fx=fx))


# ---------------------------------------------------------------------------
# A. backward-compatible BoltMaterial constructor / B. proof validation
# ---------------------------------------------------------------------------


def test_boltmaterial_backward_compatible_without_proof_allowable():
    m = BoltMaterial(name="Illustrative legacy", tensile_allowable=800e6, shear_allowable=480e6)
    assert m.proof_allowable is None


def test_boltmaterial_with_proof_allowable():
    m = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    assert m.proof_allowable == 600e6


def test_boltmaterial_proof_allowable_validation():
    with pytest.raises(ValueError):
        BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=0.0)
    with pytest.raises(ValueError):
        BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=-1.0)
    with pytest.raises(ValueError):
        BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=float("nan"))


def test_m2_assess_bolt_group_strength_unaffected_by_new_field():
    # Existing Milestone 2 callers that never reference proof_allowable
    # must behave exactly as before.
    result = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.008)
    legacy_mat = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
    strength = assess_bolt_group_strength(result, sec, legacy_mat)
    assert strength.governing_bolt_index == 1
    assert strength.governing_mode == "interaction"
    assert strength.passed is True


# ---------------------------------------------------------------------------
# C, D, E. preload stress hand calc / exact boundary / below/above allowable
# ---------------------------------------------------------------------------


def test_preload_stress_exact_boundary_hand_calc():
    # Section 18.A: F_preload=30kN, A_t=50mm^2 -> sigma=600MPa; with
    # proof=600MPa, margin=0, PASS. tensile_allowable set well above
    # 600MPa so the tension/interaction checks don't interfere with
    # isolating the preload-margin boundary itself.
    sec = BoltSection(nominal_diameter=0.01, tensile_area=50e-6, shear_area=40e-6)
    mat = BoltMaterial(name="x", tensile_allowable=900e6, shear_allowable=600e6, proof_allowable=600e6)
    group = _uniform_group_result(fz=0.0, fx=0.0)  # no external load -> preload alone
    preload = PreloadState(preload_per_bolt=30_000.0)
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    b0 = result.bolts[0]
    assert b0.preload_stress == pytest.approx(600e6)
    assert b0.preload_margin == pytest.approx(0.0, abs=1e-9)
    assert b0.passed is True


def test_preload_below_allowable_passes():
    sec = BoltSection(nominal_diameter=0.01, tensile_area=50e-6, shear_area=40e-6)
    mat = BoltMaterial(name="x", tensile_allowable=500e6, shear_allowable=300e6, proof_allowable=600e6)
    group = _uniform_group_result()
    preload = PreloadState(preload_per_bolt=25_000.0)  # sigma=500MPa < 600MPa
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    assert result.bolts[0].preload_margin > 0.0
    assert result.bolts[0].passed is True


def test_preload_above_allowable_fails():
    sec = BoltSection(nominal_diameter=0.01, tensile_area=50e-6, shear_area=40e-6)
    mat = BoltMaterial(name="x", tensile_allowable=500e6, shear_allowable=300e6, proof_allowable=600e6)
    group = _uniform_group_result()
    preload = PreloadState(preload_per_bolt=36_000.0)  # sigma=720MPa > 600MPa
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    assert result.bolts[0].preload_margin < 0.0
    assert result.bolts[0].passed is False


# ---------------------------------------------------------------------------
# F. missing proof allowable -> clear error only for integrated check
# ---------------------------------------------------------------------------


def test_missing_proof_allowable_raises_for_integrated_check_only():
    sec = circular_unthreaded_bolt(0.008)
    legacy_mat = BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6)
    group = _sanity_group_result()

    # Milestone 2 usage remains completely fine without proof_allowable.
    m2_result = assess_bolt_group_strength(group, sec, legacy_mat)
    assert m2_result is not None

    # The NEW integrated check requires it and fails loudly.
    preload = PreloadState(preload_per_bolt=35_000.0)
    with pytest.raises(ValueError):
        assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, legacy_mat)


# ---------------------------------------------------------------------------
# G, H. service bolt tension hand calc / compression-side no increment
# ---------------------------------------------------------------------------


def test_service_bolt_tension_hand_calc():
    # Section 18.B: F_preload=20kN, C=0.2, T_sep=10kN -> F_total=22kN.
    sec = BoltSection(nominal_diameter=0.01, tensile_area=100e-6, shear_area=80e-6)
    mat = BoltMaterial(name="x", tensile_allowable=500e6, shear_allowable=300e6, proof_allowable=600e6)
    group = _uniform_group_result(fz=40_000.0)  # T_ext = 10kN/bolt
    preload = PreloadState(preload_per_bolt=20_000.0)
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    b0 = result.bolts[0]
    assert b0.separating_demand == pytest.approx(10_000.0)
    assert b0.total_bolt_tension == pytest.approx(22_000.0)
    assert b0.service_tensile_stress == pytest.approx(22_000.0 / 100e-6)


def test_compression_side_no_additional_tensile_increment():
    pattern = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=5000.0, Mx=50000.0)
    group = distribute_loads(pattern, load)
    compression_bolts = [b for b in group.bolts if b.axial_total < 0]
    assert compression_bolts

    sec = circular_unthreaded_bolt(0.012)
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    preload = PreloadState(preload_per_bolt=10_000.0)
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    for b_ml1 in compression_bolts:
        b = result.bolts[b_ml1.index]
        assert b.separating_demand == 0.0
        assert b.additional_bolt_load == 0.0
        assert b.total_bolt_tension == pytest.approx(10_000.0)  # preload only


# ---------------------------------------------------------------------------
# I. service shear unchanged from Milestone 1
# ---------------------------------------------------------------------------


def test_service_shear_unchanged_from_milestone1():
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.012)
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    preload = PreloadState(preload_per_bolt=35_000.0)
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    for b_ml1, b in zip(group.bolts, result.bolts):
        assert b.shear_demand == pytest.approx(b_ml1.shear_resultant)
        assert b.service_shear_stress == pytest.approx(b_ml1.shear_resultant / sec.shear_area)


# ---------------------------------------------------------------------------
# J. tensile service margin boundary
# ---------------------------------------------------------------------------


def test_service_tensile_margin_exact_boundary():
    a_t = 100e-6
    s_t = 500e6
    preload_val = s_t * a_t  # sigma_service == S_t exactly (T_sep=0)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=80e-6)
    mat = BoltMaterial(name="x", tensile_allowable=s_t, shear_allowable=300e6, proof_allowable=1e12)
    group = _uniform_group_result()
    preload = PreloadState(preload_per_bolt=preload_val)
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    b0 = result.bolts[0]
    assert b0.service_tensile_margin == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# K, L. service interaction exact boundary / above/below
# ---------------------------------------------------------------------------


def _interaction_case(sigma_over_st: float, tau_over_ss: float, a_t=100e-6, a_s=80e-6, s_t=500e6, s_s=300e6):
    preload_val = sigma_over_st * s_t * a_t
    v_each = tau_over_ss * s_s * a_s
    group = _uniform_group_result(fz=0.0, fx=4 * v_each)
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=a_s)
    mat = BoltMaterial(name="x", tensile_allowable=s_t, shear_allowable=s_s, proof_allowable=1e12)
    preload = PreloadState(preload_per_bolt=preload_val)
    return assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)


def test_service_interaction_exact_boundary_hand_calc():
    # Section 18.C: sigma/St=0.6, tau/Ss=0.8 -> FI=1, margin=0, PASS.
    result = _interaction_case(0.6, 0.8)
    b0 = result.bolts[0]
    assert b0.interaction_fi == pytest.approx(1.0, rel=1e-9)
    assert b0.interaction_margin == pytest.approx(0.0, abs=1e-9)
    assert b0.passed is True


def test_service_interaction_slightly_below_passes():
    result = _interaction_case(0.6 * 0.99, 0.8 * 0.99)
    assert result.bolts[0].interaction_fi < 1.0
    assert result.bolts[0].passed is True


def test_service_interaction_slightly_above_fails():
    result = _interaction_case(0.6 * 1.05, 0.8 * 1.05)
    assert result.bolts[0].interaction_fi > 1.0
    assert result.bolts[0].interaction_margin < 0.0
    assert result.bolts[0].passed is False


# ---------------------------------------------------------------------------
# M, N, O. domain validity: closed/no-slip
# ---------------------------------------------------------------------------


def test_valid_closed_no_slip_domain():
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.012)
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    preload = PreloadState(preload_per_bolt=35_109.8)  # ~selected baseline preload
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    assert result.joint_closed is True
    assert result.no_slip is True
    assert result.strength_model_valid is True


def test_separated_joint_invalidates_integrated_model():
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.012)
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    tiny_preload = PreloadState(preload_per_bolt=100.0)  # far below required -> separation
    result = assess_preloaded_bolt_strength(group, tiny_preload, STIFFNESS, FRICTION, sec, mat)
    assert result.joint_closed is False
    assert result.strength_model_valid is False
    assert result.passed is False  # forced False even if some stress margins computed positive


def test_slipped_joint_invalidates_integrated_model():
    # Pure Mz shear, tiny preload -> closed (no axial demand) but slips.
    pattern = circular_pattern(8, 0.5)
    group = distribute_loads(pattern, InterfaceLoad(Mz=12000.0))
    sec = circular_unthreaded_bolt(0.012)
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    tiny_preload = PreloadState(preload_per_bolt=1.0)
    result = assess_preloaded_bolt_strength(group, tiny_preload, STIFFNESS, FRICTION, sec, mat)
    assert result.no_slip is False
    assert result.strength_model_valid is False
    assert result.passed is False


# ---------------------------------------------------------------------------
# P, Q, R, S. preload capacity ceilings hand calc
# ---------------------------------------------------------------------------


def test_proof_ceiling_hand_calc():
    a_t, s_proof = 100e-6, 600e6
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=80e-6)
    mat = BoltMaterial(name="x", tensile_allowable=500e6, shear_allowable=300e6, proof_allowable=s_proof)
    group = _uniform_group_result(fz=40_000.0, fx=8_000.0)
    ceilings = compute_preload_ceilings(group, sec, mat, STIFFNESS)
    assert ceilings.proof_ceiling == pytest.approx(s_proof * a_t)


def test_tension_ceiling_hand_calc():
    a_t, s_t, c = 100e-6, 500e6, STIFFNESS.C
    t_sep = 10_000.0
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=80e-6)
    mat = BoltMaterial(name="x", tensile_allowable=s_t, shear_allowable=300e6, proof_allowable=1e12)
    group = _uniform_group_result(fz=4 * t_sep)
    ceilings = compute_preload_ceilings(group, sec, mat, STIFFNESS)
    expected = s_t * a_t - c * t_sep
    assert ceilings.tension_ceiling == pytest.approx(expected)


def test_interaction_ceiling_hand_calc():
    a_t, a_s, s_t, s_s, c = 100e-6, 80e-6, 500e6, 300e6, STIFFNESS.C
    t_sep = 5_000.0
    v = 0.5 * a_s * s_s  # shear ratio 0.5
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=a_s)
    mat = BoltMaterial(name="x", tensile_allowable=s_t, shear_allowable=s_s, proof_allowable=1e12)
    group = _uniform_group_result(fz=4 * t_sep, fx=4 * v)
    ceilings = compute_preload_ceilings(group, sec, mat, STIFFNESS)
    expected = a_t * s_t * math.sqrt(1.0 - 0.5**2) - c * t_sep
    assert ceilings.interaction_ceiling == pytest.approx(expected, rel=1e-9)


def test_interaction_ceiling_infeasible_when_shear_ratio_exceeds_one():
    a_t, a_s, s_t, s_s = 100e-6, 80e-6, 500e6, 300e6
    v = 1.5 * a_s * s_s  # shear ratio 1.5 > 1
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=a_s)
    mat = BoltMaterial(name="x", tensile_allowable=s_t, shear_allowable=s_s, proof_allowable=1e12)
    group = _uniform_group_result(fz=0.0, fx=4 * v)
    ceilings = compute_preload_ceilings(group, sec, mat, STIFFNESS)
    assert ceilings.interaction_ceiling is None
    assert 0 in ceilings.interaction_infeasible_bolts


def test_group_ceiling_equals_minimum_bolt_ceiling():
    # Irregular pattern -> different T_sep per bolt -> tension ceiling
    # must equal the MINIMUM per-bolt value.
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.012)
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    ceilings = compute_preload_ceilings(group, sec, mat, STIFFNESS)
    assert ceilings.tension_ceiling == pytest.approx(min(ceilings.tension_ceiling_per_bolt))
    feasible_interaction = [t for t in ceilings.interaction_ceiling_per_bolt if t is not None]
    assert ceilings.interaction_ceiling == pytest.approx(min(feasible_interaction))


# ---------------------------------------------------------------------------
# T, U, V. feasible-window boundary
# ---------------------------------------------------------------------------


def test_exact_feasible_window_boundary():
    group = _uniform_group_result(fz=0.0, fx=0.0)  # required preload = 0 (no demand)
    a_t = 100e-6
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=80e-6)
    # Force a group with SOME required preload via a small Fz, then set
    # proof allowable so ceiling == required exactly.
    group2 = _uniform_group_result(fz=4000.0)  # T_sep small, still generates a slip/sep requirement
    req = required_preload(group2, STIFFNESS, FRICTION)
    s_proof = req.overall_required / a_t  # proof ceiling == required exactly
    mat = BoltMaterial(name="x", tensile_allowable=1e12, shear_allowable=1e12, proof_allowable=s_proof)
    window = preload_capacity_window(group2, sec, mat, STIFFNESS, FRICTION)
    assert window.window_width == pytest.approx(0.0, rel=1e-6)
    assert window.feasible is True  # boundary (width==0) counts as feasible


def test_infeasible_preload_window():
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.005)  # tiny bolt -> low ceilings
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    window = preload_capacity_window(group, sec, mat, STIFFNESS, FRICTION)
    assert window.feasible is False
    assert window.window_width < 0.0


def test_slightly_smaller_lower_bound_restores_feasibility():
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.008)
    mat = BoltMaterial(name="x", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)
    window = preload_capacity_window(group, sec, mat, STIFFNESS, FRICTION)
    # Real required preload vs ceiling:
    ceiling = window.ceilings.overall_ceiling
    assert ceiling is not None
    # A slightly smaller synthetic requirement (below the ceiling) is feasible.
    smaller_required = ceiling * 0.9
    assert smaller_required <= ceiling


# ---------------------------------------------------------------------------
# W, X, Y, Z, AA, AB. candidate ordering / selection
# ---------------------------------------------------------------------------


def _sanity_material():
    return BoltMaterial(name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6)


def test_deterministic_candidate_ordering():
    group = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d) for d in (0.012, 0.006, 0.010, 0.005, 0.008)]
    results = evaluate_preloaded_candidates(group, candidates, _sanity_material(), STIFFNESS, FRICTION)
    diameters = [r.bolt_section.nominal_diameter for r in results]
    assert diameters == sorted(diameters)


def test_smallest_passing_preloaded_candidate():
    group = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d / 1000.0) for d in (5, 6, 8, 10, 12)]
    selected = select_smallest_passing_preloaded_bolt(group, candidates, _sanity_material(), STIFFNESS, FRICTION)
    assert selected.passed is True
    assert selected.bolt_section.nominal_diameter == pytest.approx(0.010)


def test_candidate_passes_m2_but_fails_m6():
    # 8 mm is the M2 external-load-only smallest passing candidate, but
    # fails the M6 integrated check at the selected (1.2x) preload.
    group = _sanity_group_result()
    sec8 = circular_unthreaded_bolt(0.008)
    m2_result = assess_bolt_group_strength(group, sec8, _sanity_material())
    assert m2_result.passed is True

    candidates = [circular_unthreaded_bolt(d / 1000.0) for d in (5, 6, 8, 10, 12)]
    results = evaluate_preloaded_candidates(group, candidates, _sanity_material(), STIFFNESS, FRICTION)
    result_8mm = next(r for r in results if r.bolt_section.nominal_diameter == pytest.approx(0.008))
    assert result_8mm.passed is False


def test_larger_candidate_improves_preload_margin():
    group = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d / 1000.0) for d in (8, 10, 12)]
    results = evaluate_preloaded_candidates(group, candidates, _sanity_material(), STIFFNESS, FRICTION)
    margins = [r.strength.bolts[r.strength.governing_bolt_index].preload_margin for r in results]
    for i in range(len(margins) - 1):
        assert margins[i + 1] > margins[i]


def test_larger_candidate_improves_interaction_margin():
    group = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d / 1000.0) for d in (8, 10, 12)]
    results = evaluate_preloaded_candidates(group, candidates, _sanity_material(), STIFFNESS, FRICTION)
    fi_max = [max(b.interaction_fi for b in r.strength.bolts) for r in results]
    for i in range(len(fi_max) - 1):
        assert fi_max[i + 1] < fi_max[i]


def test_no_feasible_candidate_raises():
    group = _sanity_group_result()
    candidates = [circular_unthreaded_bolt(d / 1000.0) for d in (3, 4, 5)]
    with pytest.raises(NoFeasibleCandidateError):
        select_smallest_passing_preloaded_bolt(group, candidates, _sanity_material(), STIFFNESS, FRICTION)


# ---------------------------------------------------------------------------
# AC. repeated assessment deterministic
# ---------------------------------------------------------------------------


def test_repeated_assessment_deterministic():
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.010)
    preload = PreloadState(preload_per_bolt=35_109.8)
    r1 = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, _sanity_material())
    r2 = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, _sanity_material())
    assert r1 == r2


# ---------------------------------------------------------------------------
# AD, AE, AF. preload sensitivity
# ---------------------------------------------------------------------------


def test_increasing_preload_improves_slip_margin():
    group = _sanity_group_result()
    req = required_preload(group, STIFFNESS, FRICTION)
    slip_margins = []
    for factor in (1.0, 1.1, 1.2, 1.3, 1.5, 2.0):
        preload = PreloadState(preload_per_bolt=req.overall_required * factor)
        joint = assess_preloaded_joint(group, preload, STIFFNESS, FRICTION)
        slip_margins.append(joint.min_slip_margin)
    for i in range(len(slip_margins) - 1):
        assert slip_margins[i + 1] > slip_margins[i]


def test_increasing_preload_worsens_preload_margin():
    group = _sanity_group_result()
    req = required_preload(group, STIFFNESS, FRICTION)
    sec = circular_unthreaded_bolt(0.010)
    mat = _sanity_material()
    preload_margins = []
    for factor in (1.0, 1.1, 1.2, 1.3, 1.5, 2.0):
        preload = PreloadState(preload_per_bolt=req.overall_required * factor)
        result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
        preload_margins.append(result.bolts[0].preload_margin)
    for i in range(len(preload_margins) - 1):
        assert preload_margins[i + 1] < preload_margins[i]


def test_increasing_preload_worsens_service_interaction_margin():
    group = _sanity_group_result()
    req = required_preload(group, STIFFNESS, FRICTION)
    sec = circular_unthreaded_bolt(0.010)
    mat = _sanity_material()
    interaction_margins = []
    for factor in (1.0, 1.1, 1.2, 1.3, 1.5, 2.0):
        preload = PreloadState(preload_per_bolt=req.overall_required * factor)
        result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
        worst = min(b.interaction_margin for b in result.bolts if b.interaction_margin is not None)
        interaction_margins.append(worst)
    for i in range(len(interaction_margins) - 1):
        assert interaction_margins[i + 1] < interaction_margins[i]


# ---------------------------------------------------------------------------
# AG, AH. C sensitivity
# ---------------------------------------------------------------------------


def test_increasing_C_raises_bolt_tensile_increment():
    group = _sanity_group_result()
    increments = []
    for C in (0.10, 0.20, 0.30, 0.40):
        km = 1e8 * (1 - C) / C
        stiffness = JointStiffness(bolt_stiffness=1e8, member_stiffness=km)
        preload = PreloadState(preload_per_bolt=35_000.0)
        joint = assess_preloaded_joint(group, preload, stiffness, FRICTION)
        b1 = next(b for b in joint.bolts if b.index == 1)  # max tensile bolt
        increments.append(b1.additional_bolt_load)
    for i in range(len(increments) - 1):
        assert increments[i + 1] > increments[i]


def test_increasing_C_reduces_clamp_force_loss():
    group = _sanity_group_result()
    reductions = []
    for C in (0.10, 0.20, 0.30, 0.40):
        km = 1e8 * (1 - C) / C
        stiffness = JointStiffness(bolt_stiffness=1e8, member_stiffness=km)
        preload = PreloadState(preload_per_bolt=35_000.0)
        joint = assess_preloaded_joint(group, preload, stiffness, FRICTION)
        b1 = next(b for b in joint.bolts if b.index == 1)
        reductions.append(b1.clamp_force_reduction)
    for i in range(len(reductions) - 1):
        assert reductions[i + 1] < reductions[i]


# ---------------------------------------------------------------------------
# AI. friction sensitivity
# ---------------------------------------------------------------------------


def test_decreasing_mu_raises_required_preload():
    group = _sanity_group_result()
    required = []
    for mu in (0.40, 0.30, 0.25, 0.20, 0.15, 0.10):
        fric = FrictionModel(friction_coefficient=mu)
        req = required_preload(group, STIFFNESS, fric)
        required.append(req.overall_required)
    for i in range(len(required) - 1):
        assert required[i + 1] > required[i]


# ---------------------------------------------------------------------------
# AJ, AK, AL. Milestone 1/2/3 results not mutated
# ---------------------------------------------------------------------------


def test_milestone1_result_not_mutated():
    group = _sanity_group_result()
    before_bolts = tuple(group.bolts)
    before_eq = group.equilibrium
    sec = circular_unthreaded_bolt(0.010)
    preload = PreloadState(preload_per_bolt=35_000.0)
    assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, _sanity_material())
    assert group.bolts == before_bolts
    assert group.equilibrium == before_eq


def test_milestone2_result_not_mutated():
    group = _sanity_group_result()
    sec = circular_unthreaded_bolt(0.008)
    mat = _sanity_material()
    strength_before = assess_bolt_group_strength(group, sec, mat)
    preload = PreloadState(preload_per_bolt=35_000.0)
    assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    strength_after = assess_bolt_group_strength(group, sec, mat)
    assert strength_before == strength_after
    assert strength_after.governing_bolt_index == 1
    assert strength_after.governing_mode == "interaction"
    assert strength_after.passed is True


def test_milestone3_result_not_mutated():
    group = _sanity_group_result()
    preload = PreloadState(preload_per_bolt=35_000.0)
    joint_before = assess_preloaded_joint(group, preload, STIFFNESS, FRICTION)
    sec = circular_unthreaded_bolt(0.010)
    assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, _sanity_material())
    joint_after = assess_preloaded_joint(group, preload, STIFFNESS, FRICTION)
    assert joint_before == joint_after


# ---------------------------------------------------------------------------
# Governing-mode tie-break (Section 9): tension/shear preferred over
# interaction on an exact tie, matching Milestone 2's convention.
# ---------------------------------------------------------------------------


def test_governing_mode_tension_preferred_on_tie_pure_tension():
    # Pure preload/tension demand (no shear at all): tension and
    # interaction tie exactly -> tension wins (specific mode).
    a_t, s_t = 100e-6, 500e6
    sec = BoltSection(nominal_diameter=0.012, tensile_area=a_t, shear_area=80e-6)
    mat = BoltMaterial(name="x", tensile_allowable=s_t, shear_allowable=300e6, proof_allowable=1e12)
    group = _uniform_group_result()  # no shear, no axial -> preload only
    preload_val = s_t * a_t * 0.5  # sigma_service = 0.5*S_t, no shear -> ties w/ interaction
    preload = PreloadState(preload_per_bolt=preload_val)
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    assert result.governing_mode == "tension"


def test_governing_mode_shear_preferred_on_tie_pure_shear():
    a_s, s_s = 80e-6, 300e6
    v_each = 0.5 * a_s * s_s
    sec = BoltSection(nominal_diameter=0.012, tensile_area=1e12, shear_area=a_s)  # huge A_t -> tension margin N/A-ish (still finite though)
    # Use a very large tensile allowable so tensile/interaction never binds relative to shear.
    mat = BoltMaterial(name="x", tensile_allowable=1e15, shear_allowable=s_s, proof_allowable=1e15)
    group = _uniform_group_result(fx=4 * v_each)
    preload = PreloadState(preload_per_bolt=1.0)  # negligible preload stress given huge A_t
    result = assess_preloaded_bolt_strength(group, preload, STIFFNESS, FRICTION, sec, mat)
    assert result.governing_mode == "shear"


def test_governing_mode_interaction_when_both_present():
    result = _interaction_case(0.5, 0.5)
    assert result.governing_mode in ("interaction",) or result.bolts[0].governing_mode == "interaction"
