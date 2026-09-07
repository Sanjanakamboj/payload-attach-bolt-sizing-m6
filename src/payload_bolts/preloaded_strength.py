"""Integrated preload + bolt-strength assessment (Milestone 6).

This module answers a question neither Milestone 2 (external-load-only
bolt strength) nor Milestone 4 (installation-preload feasibility window
against proof/yield *force*, `preload_limits.py`) nor Milestone 5
(local-joint screens, `joint_local_checks.py`) directly answers:

    Does the preload required to prevent joint separation/slip remain
    structurally acceptable for the selected bolt, and if not, what
    bolt size is required when preload and launch loads are assessed
    together -- at the STRESS level, using the same tensile/shear
    allowables and quadratic interaction criterion already verified in
    Milestone 2?

It is a distinct, additive layer alongside (not a replacement for)
`preload_limits.py`'s force-based proof/yield installation window:

- `preload_limits.py` asks "is the installation preload itself, and the
  resulting in-service bolt FORCE, within proof/yield force limits?"
  using a separate `BoltStrengthLimits` (proof/yield strength) property
  set and an installation-fraction/scatter-allowance convention.
- This module asks "once the bolt carries preload + external tension +
  external shear simultaneously, do the Milestone 2 STRESS-based
  tensile/shear/interaction margins (against `BoltMaterial`'s
  tensile/shear allowables) still pass?" and separately, "is the
  preload stress itself within a proof/preload *stress* allowable?"

This module does **not** recompute or modify:

- Milestone 1 bolt-group geometry, direct/torsional shear, overturning
  axial distribution, or equilibrium verification;
- Milestone 2 bolt tensile/shear strength margins for EXTERNAL-LOAD-ONLY
  assessment (`assess_bolt_group_strength` remains available and
  unchanged for that comparison), the quadratic interaction criterion,
  or its governing-mode tie-break convention (extended here, not
  altered -- see `_governing_for_bolt` below);
- Milestone 3 load-fraction C, separation/slip screening, or the
  analytical required-preload equations (`required_preload`,
  `assess_preloaded_joint` are called here exactly as-is).

Signed-load / zero-demand conventions mirror Milestone 2/3 throughout.

Domain validity
----------------
This module's SERVICE stress model (bolt carries preload plus the
Milestone-3 closed-joint linear load-sharing increment) is valid only
within the Milestone 3 closed/no-slip regime. If any bolt has
separated or slipped, this module does NOT silently claim a valid
integrated strength result: `strength_model_valid` is set False and the
overall integrated pass is forced False, even if the raw stress-based
margins happen to compute a positive number.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .preload import (
    BoltPreloadGroupResult,
    FrictionModel,
    JointStiffness,
    PreloadState,
    assess_preloaded_joint,
    required_preload,
)
from .solver import BoltGroupResult
from .strength import BoltMaterial, BoltSection, NoFeasibleCandidateError

# Deterministic governing-mode tie-break order for the FOUR Milestone 6
# modes (preload, tension, shear, interaction). This EXTENDS, and does
# not modify, Milestone 2's tie-break convention
# (strength.py: tension, shear, interaction -- interaction only governs
# outright, not by tie-break, when both tensile and shear demand are
# simultaneously present). "preload" is placed first: it is evaluated
# against a completely separate allowable (proof/preload) and stress
# basis (preload-only, not the combined service stress), so it is
# always the most "physically specific" possible governing reason and
# essentially never ties with the service-stress modes in practice; it
# is ordered first purely so a genuine exact tie is still resolved
# deterministically rather than left to dict/list ordering.
_MODE_ORDER = {"preload": 0, "tension": 1, "shear": 2, "interaction": 3}


@dataclass(frozen=True)
class PreloadedBoltStrengthResult:
    """Per-bolt integrated preload + strength result."""

    index: int
    axial_total: float  # signed external axial load, Milestone 1, unchanged
    separating_demand: float  # T_sep,i = max(axial_total, 0), Milestone 3
    preload: float  # F_preload
    load_fraction: float  # C
    additional_bolt_load: float  # Delta_F_b,i = C * T_sep,i, Milestone 3
    total_bolt_tension: float  # F_preload + Delta_F_b,i (SERVICE tension), Milestone 3
    shear_demand: float  # V_i, Milestone 1 shear_resultant, unchanged
    preload_stress: float  # sigma_preload = F_preload / A_t
    service_tensile_stress: float  # sigma_service = total_bolt_tension / A_t
    service_shear_stress: float  # tau_service = V_i / A_s
    preload_margin: Optional[float]  # S_proof/sigma_preload - 1
    service_tensile_margin: Optional[float]  # S_t/sigma_service - 1
    service_shear_margin: Optional[float]  # S_s/tau_service - 1
    interaction_fi: float  # (sigma_service/S_t)^2 + (tau_service/S_s)^2
    interaction_margin: Optional[float]  # 1/sqrt(FI) - 1
    governing_margin: Optional[float]
    governing_mode: Optional[str]  # "preload" | "tension" | "shear" | "interaction" | None
    passed: bool
    separation_pass: bool  # Milestone 3, unchanged, retained for reporting
    slip_pass: bool  # Milestone 3, unchanged, retained for reporting


@dataclass(frozen=True)
class PreloadedBoltGroupStrengthResult:
    """Group-level integrated preload + strength result."""

    bolts: Tuple[PreloadedBoltStrengthResult, ...]
    bolt_section: BoltSection
    material: BoltMaterial
    preload: float
    load_fraction: float
    friction: FrictionModel
    joint_closed: bool  # Milestone 3 all_locations_closed
    no_slip: bool  # Milestone 3 all_locations_no_slip
    strength_model_valid: bool  # joint_closed AND no_slip
    governing_bolt_index: int
    governing_mode: Optional[str]
    governing_margin: Optional[float]
    all_bolts_pass: bool  # every bolt's own preload/tensile/shear/interaction check passes
    passed: bool  # overall integrated PASS: strength_model_valid AND all_bolts_pass


def _select_governing_min(values: Sequence[Tuple[str, Optional[float]]]) -> Tuple[Optional[float], Optional[str]]:
    """Deterministic governing-mode selection among (mode, margin) pairs.
    None margins are not applicable (no demand); ties broken by
    `_MODE_ORDER`. Returns (None, None) if nothing is applicable."""
    candidates = [(mode, margin) for mode, margin in values if margin is not None]
    if not candidates:
        return None, None
    candidates.sort(key=lambda item: (item[1], _MODE_ORDER[item[0]]))
    mode, margin = candidates[0]
    return margin, mode


def _assess_single_bolt(
    bolt, preload_per_bolt: float, c: float, section: BoltSection, material: BoltMaterial
) -> PreloadedBoltStrengthResult:
    sigma_preload = preload_per_bolt / section.tensile_area
    sigma_service = bolt.total_bolt_tension / section.tensile_area
    tau_service = bolt.shear_demand / section.shear_area

    if material.proof_allowable is None:
        raise ValueError(
            "assess_preloaded_bolt_strength requires BoltMaterial.proof_allowable to be set "
            "(Milestone 2-only materials with proof_allowable=None cannot be used for the "
            "integrated preload/strength check)."
        )
    preload_margin = material.proof_allowable / sigma_preload - 1.0 if sigma_preload > 0.0 else None

    tensile_margin = material.tensile_allowable / sigma_service - 1.0 if sigma_service > 0.0 else None
    shear_margin = material.shear_allowable / tau_service - 1.0 if tau_service > 0.0 else None

    fi = (sigma_service / material.tensile_allowable) ** 2 + (tau_service / material.shear_allowable) ** 2
    interaction_margin = 1.0 / math.sqrt(fi) - 1.0 if fi > 0.0 else None

    governing_margin, governing_mode = _select_governing_min(
        [
            ("preload", preload_margin),
            ("tension", tensile_margin),
            ("shear", shear_margin),
            ("interaction", interaction_margin),
        ]
    )
    passed = governing_margin is None or governing_margin >= 0.0

    return PreloadedBoltStrengthResult(
        index=bolt.index,
        axial_total=bolt.axial_total,
        separating_demand=bolt.separating_demand,
        preload=preload_per_bolt,
        load_fraction=c,
        additional_bolt_load=bolt.additional_bolt_load,
        total_bolt_tension=bolt.total_bolt_tension,
        shear_demand=bolt.shear_demand,
        preload_stress=sigma_preload,
        service_tensile_stress=sigma_service,
        service_shear_stress=tau_service,
        preload_margin=preload_margin,
        service_tensile_margin=tensile_margin,
        service_shear_margin=shear_margin,
        interaction_fi=fi,
        interaction_margin=interaction_margin,
        governing_margin=governing_margin,
        governing_mode=governing_mode,
        passed=passed,
        separation_pass=bolt.separation_pass,
        slip_pass=bolt.slip_pass,
    )


def assess_preloaded_bolt_strength(
    group_load_result: BoltGroupResult,
    preload: PreloadState,
    stiffness: JointStiffness,
    friction: FrictionModel,
    bolt_section: BoltSection,
    material: BoltMaterial,
) -> PreloadedBoltGroupStrengthResult:
    """Integrated Milestone 6 assessment: preload stress, SERVICE
    (preload + external-tension-increment) tensile stress, external
    shear stress, and the Milestone 2 quadratic interaction criterion
    applied to the SERVICE stresses -- all reusing Milestone 3's
    `assess_preloaded_joint` per-bolt loads unchanged (no recomputation
    of bolt-group mechanics or load-sharing).

    Raises ValueError if `material.proof_allowable` is None (see
    `BoltMaterial` docstring) -- this integrated check has no meaningful
    zero-demand convention for a missing proof allowable, so it fails
    loudly rather than silently skipping the preload check.
    """
    joint_result: BoltPreloadGroupResult = assess_preloaded_joint(
        group_load_result, preload, stiffness, friction
    )
    per_bolt = tuple(
        _assess_single_bolt(b, preload.preload_per_bolt, joint_result.load_fraction, bolt_section, material)
        for b in joint_result.bolts
    )

    joint_closed = joint_result.all_locations_closed
    no_slip = joint_result.all_locations_no_slip
    strength_model_valid = joint_closed and no_slip

    def _sort_key(b: PreloadedBoltStrengthResult):
        margin = b.governing_margin if b.governing_margin is not None else math.inf
        return (margin, b.index)

    governing_bolt = min(per_bolt, key=_sort_key)
    all_bolts_pass = all(b.passed for b in per_bolt)

    return PreloadedBoltGroupStrengthResult(
        bolts=per_bolt,
        bolt_section=bolt_section,
        material=material,
        preload=preload.preload_per_bolt,
        load_fraction=joint_result.load_fraction,
        friction=friction,
        joint_closed=joint_closed,
        no_slip=no_slip,
        strength_model_valid=strength_model_valid,
        governing_bolt_index=governing_bolt.index,
        governing_mode=governing_bolt.governing_mode,
        governing_margin=governing_bolt.governing_margin,
        all_bolts_pass=all_bolts_pass,
        passed=strength_model_valid and all_bolts_pass,
    )


# ---------------------------------------------------------------------------
# Preload capacity ceilings (Section 10) and feasibility window (Section 11)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PreloadCeilings:
    """Upper bounds on per-bolt preload from three independent bolt-
    strength considerations, all using the exact Milestone 2 tensile/
    shear stress areas and allowables (never a new/conflicting area or
    allowable convention)."""

    proof_ceiling: Optional[float]  # F_preload,max,proof = S_proof * A_t (None if proof_allowable missing)
    tension_ceiling_per_bolt: Tuple[float, ...]  # S_t*A_t - C*T_sep,i, indexed by bolt
    tension_ceiling: float  # min_i(tension_ceiling_per_bolt)
    tension_ceiling_governing_bolt: int
    interaction_ceiling_per_bolt: Tuple[Optional[float], ...]  # None where shear ratio > 1 (infeasible)
    interaction_ceiling: Optional[float]  # min over bolts with a feasible value, or None if all infeasible
    interaction_ceiling_governing_bolt: Optional[int]
    interaction_infeasible_bolts: Tuple[int, ...]  # bolt indices where V/(A_s*S_s) > 1
    overall_ceiling: Optional[float]  # min(proof, tension, interaction), None if any ceiling absent/infeasible
    overall_governing_constraint: Optional[str]  # "proof" | "tension" | "interaction"


def compute_preload_ceilings(
    group_load_result: BoltGroupResult,
    bolt_section: BoltSection,
    material: BoltMaterial,
    stiffness: JointStiffness,
) -> PreloadCeilings:
    """Compute the three independent preload capacity ceilings (Section
    10): proof, service-tensile, and quadratic-interaction, all against
    the SAME candidate bolt section/material used for the Milestone 2
    strength assessment.

    A_t, A_s, S_t, S_s, S_proof are exactly `bolt_section`/`material`'s
    fields -- no new stress-area convention is introduced here.
    """
    a_t = bolt_section.tensile_area
    a_s = bolt_section.shear_area
    s_t = material.tensile_allowable
    s_s = material.shear_allowable
    c = stiffness.C

    proof_ceiling = material.proof_allowable * a_t if material.proof_allowable is not None else None

    t_sep = [max(b.axial_total, 0.0) for b in group_load_result.bolts]
    v = [b.shear_resultant for b in group_load_result.bolts]

    tension_terms = [s_t * a_t - c * t for t in t_sep]
    tension_ceiling_idx = min(range(len(tension_terms)), key=lambda i: (tension_terms[i], i))
    tension_ceiling = tension_terms[tension_ceiling_idx]

    interaction_terms: List[Optional[float]] = []
    infeasible: List[int] = []
    for i in range(len(t_sep)):
        shear_ratio = v[i] / (a_s * s_s)
        if shear_ratio > 1.0:
            interaction_terms.append(None)
            infeasible.append(i)
        else:
            term = a_t * s_t * math.sqrt(1.0 - shear_ratio**2) - c * t_sep[i]
            interaction_terms.append(term)

    if infeasible:
        # ANY bolt with shear ratio > 1 already fails the interaction
        # criterion from shear alone, at ANY preload (even F_preload=0)
        # -- this makes the whole candidate interaction-infeasible, not
        # merely a matter of excluding that bolt from a group minimum.
        interaction_ceiling_idx, interaction_ceiling = None, None
    else:
        interaction_ceiling_idx, interaction_ceiling = min(
            enumerate(interaction_terms), key=lambda item: (item[1], item[0])
        )

    if interaction_ceiling is None:
        # Interaction infeasibility overrides regardless of the proof/
        # tension ceilings -- report that plainly rather than silently
        # falling back to a ceiling that ignores the failing bolt.
        overall_ceiling = None
        overall_constraint = "interaction"
    else:
        named = [
            ("proof", proof_ceiling),
            ("tension", tension_ceiling),
            ("interaction", interaction_ceiling),
        ]
        named = [(name, val) for name, val in named if val is not None]
        overall_constraint, overall_ceiling = min(named, key=lambda item: item[1])

    return PreloadCeilings(
        proof_ceiling=proof_ceiling,
        tension_ceiling_per_bolt=tuple(tension_terms),
        tension_ceiling=tension_ceiling,
        tension_ceiling_governing_bolt=tension_ceiling_idx,
        interaction_ceiling_per_bolt=tuple(interaction_terms),
        interaction_ceiling=interaction_ceiling,
        interaction_ceiling_governing_bolt=interaction_ceiling_idx,
        interaction_infeasible_bolts=tuple(infeasible),
        overall_ceiling=overall_ceiling,
        overall_governing_constraint=overall_constraint,
    )


@dataclass(frozen=True)
class PreloadCapacityWindow:
    """Feasible-preload window: [required_preload, overall_ceiling]."""

    required_preload: float
    ceilings: PreloadCeilings
    window_width: Optional[float]  # ceilings.overall_ceiling - required_preload, unclipped
    feasible: bool


def preload_capacity_window(
    group_load_result: BoltGroupResult,
    bolt_section: BoltSection,
    material: BoltMaterial,
    stiffness: JointStiffness,
    friction: FrictionModel,
) -> PreloadCapacityWindow:
    """Combine the Milestone 3 required preload (lower bound) with the
    Milestone 6 preload capacity ceilings (upper bound) into one
    feasibility window. Reports width/feasibility honestly -- a
    negative width (infeasible window) is never clipped."""
    req = required_preload(group_load_result, stiffness, friction)
    ceilings = compute_preload_ceilings(group_load_result, bolt_section, material, stiffness)
    if ceilings.overall_ceiling is None:
        return PreloadCapacityWindow(
            required_preload=req.overall_required, ceilings=ceilings, window_width=None, feasible=False
        )
    width = ceilings.overall_ceiling - req.overall_required
    return PreloadCapacityWindow(
        required_preload=req.overall_required,
        ceilings=ceilings,
        window_width=width,
        feasible=width >= 0.0,
    )


# ---------------------------------------------------------------------------
# Preload-compatible candidate sizing
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PreloadedCandidateResult:
    """One candidate's full Milestone 6 evaluation: capacity window plus
    the integrated strength assessment at the selected preload."""

    bolt_section: BoltSection
    window: PreloadCapacityWindow
    selected_preload: float
    strength: PreloadedBoltGroupStrengthResult

    @property
    def passed(self) -> bool:
        return self.window.feasible and self.strength.passed


def evaluate_preloaded_candidates(
    group_load_result: BoltGroupResult,
    candidates: Sequence[BoltSection],
    material: BoltMaterial,
    stiffness: JointStiffness,
    friction: FrictionModel,
    preload_factor: float = 1.2,
) -> List[PreloadedCandidateResult]:
    """Evaluate candidates sorted by increasing nominal diameter. For
    each: compute the Milestone 3 required preload (unchanged across
    candidates -- joint stiffness/C/friction/geometry are held fixed,
    per Section 13: C is never made diameter-dependent), the Milestone 6
    preload capacity window for THAT candidate's section, a selected
    preload = preload_factor * required (Milestone 3 policy, unchanged),
    and the integrated strength assessment at that selected preload."""
    ordered = sorted(candidates, key=lambda s: s.nominal_diameter)
    results = []
    req = required_preload(group_load_result, stiffness, friction)
    selected_preload = req.overall_required * preload_factor
    for section in ordered:
        window = preload_capacity_window(group_load_result, section, material, stiffness, friction)
        preload_state = PreloadState(preload_per_bolt=selected_preload)
        strength = assess_preloaded_bolt_strength(
            group_load_result, preload_state, stiffness, friction, section, material
        )
        results.append(
            PreloadedCandidateResult(
                bolt_section=section, window=window, selected_preload=selected_preload, strength=strength
            )
        )
    return results


def select_smallest_passing_preloaded_bolt(
    group_load_result: BoltGroupResult,
    candidates: Sequence[BoltSection],
    material: BoltMaterial,
    stiffness: JointStiffness,
    friction: FrictionModel,
    preload_factor: float = 1.2,
) -> PreloadedCandidateResult:
    """Return the smallest-diameter candidate (increasing-diameter
    order) whose Milestone 6 integrated assessment passes: the preload
    capacity window is feasible AND the integrated strength result
    passes (joint closed, no slip, preload/tension/shear/interaction
    margins all >= 0).

    Raises NoFeasibleCandidateError if none pass. Never enlarges beyond
    the supplied candidate list."""
    results = evaluate_preloaded_candidates(
        group_load_result, candidates, material, stiffness, friction, preload_factor
    )
    for result in results:
        if result.passed:
            return result
    diameters = [r.bolt_section.nominal_diameter for r in results]
    raise NoFeasibleCandidateError(
        f"No candidate bolt section passed the Milestone 6 integrated preload/strength "
        f"assessment out of {len(results)} candidate(s) with nominal diameters {diameters} m. "
        "Supply a larger or stronger candidate; this module does not auto-enlarge."
    )
