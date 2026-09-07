"""Preliminary bolt tensile/shear strength screening (Milestone 2).

This module consumes the already-verified Milestone 1 rigid-interface
bolt-group load distribution (`payload_bolts.solver.BoltGroupResult`) and
layers a preliminary strength assessment on top of it. It does **not**
recompute bolt-group mechanics: bolt-group geometry, direct/torsional
shear, overturning axial distribution, and the equilibrium-verification
helper are all Milestone 1 and are used here unchanged.

Scope of this module: nominal bolt tensile/shear stress, separate
tensile and shear margins of safety, and one illustrative quadratic
tension-shear interaction check. All margins produced here are
**preliminary / illustrative strength margins**, not certification
margins.

Explicitly NOT modeled here (deferred to later milestones): preload,
torque, friction/slip load sharing, joint separation/contact
redistribution, bearing, tear-out, pull-through, prying, thread
stripping, fatigue, detailed fastener-standard/database lookups, and
structural optimization.

Signed-load policy
-------------------
Milestone 1 retains the *signed* per-bolt axial load (`axial_total`),
where a negative value means the rigid linear-elastic distribution puts
that bolt location on the compression side. This module never mutates
or erases that signed value. For the tensile strength check only, the
tensile demand at each bolt is clipped at zero:

    T_positive = max(axial_total, 0.0)

A bolt with axial_total <= 0 has no tensile demand (tensile check is
"not applicable" for that bolt) and this module does not evaluate a
compressive bolt failure mode in Milestone 2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .solver import BoltGroupResult

# Deterministic governing-mode tie-break order (see module docstring /
# README "Deterministic governing logic"). Because the quadratic
# interaction FI = (sigma_t/St)^2 + (tau/Ss)^2 always satisfies
# interaction_margin <= tension_margin and interaction_margin <=
# shear_margin (with equality exactly when the *other* demand is zero),
# a bolt loaded in pure tension or pure shear ties exactly with
# "interaction". This tie-break prefers the more specific single-mode
# explanation (tension, then shear) over "interaction" in that case, so
# a pure-tension bolt is reported tension-governed (not
# interaction-governed) and a pure-shear bolt is reported
# shear-governed. Interaction only wins outright -- not by tie-break --
# when both tensile and shear demand are present, in which case its
# margin is strictly lower than either individual margin.
_MODE_ORDER = {"tension": 0, "shear": 1, "interaction": 2}


class NoFeasibleCandidateError(ValueError):
    """Raised by select_smallest_passing_bolt when no supplied candidate
    bolt section passes the strength assessment. Milestone 2 never
    silently enlarges beyond the supplied candidate list."""


@dataclass(frozen=True)
class BoltMaterial:
    """Bolt material strength representation.

    Units: Pa (pascals). `tensile_allowable` and `shear_allowable` are
    required and must be finite and > 0, as in Milestone 2.

    `proof_allowable` (Milestone 6+, optional): an illustrative
    proof/preload working-stress allowable, distinct from
    `tensile_allowable` -- proof/preload strength governs installation
    tension, while `tensile_allowable` governs external-load-only (or,
    in Milestone 6+, combined service) tensile margin. Defaults to
    `None` for exact backward compatibility with Milestone 2 callers
    that never reference it; when supplied it must be finite and > 0.
    Any assessment that specifically requires a preload/proof check
    (see `preloaded_strength.py`) raises a clear error if it is missing,
    rather than silently skipping the check.

    Illustrative material property sets used in this project are
    explicitly labeled as illustrative in their `name` and are not
    claimed to be sourced from any real fastener specification unless
    stated otherwise.
    """

    name: str
    tensile_allowable: float
    shear_allowable: float
    proof_allowable: Optional[float] = None

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("BoltMaterial.name must be a non-empty string.")
        for field_name in ("tensile_allowable", "shear_allowable"):
            value = getattr(self, field_name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"BoltMaterial.{field_name} must be finite and > 0, got {value}.")
        if self.proof_allowable is not None:
            if not math.isfinite(self.proof_allowable) or self.proof_allowable <= 0:
                raise ValueError(
                    f"BoltMaterial.proof_allowable must be finite and > 0, got {self.proof_allowable}."
                )


@dataclass(frozen=True)
class BoltSection:
    """Bolt cross-section representation for strength screening.

    Units: meters (diameter), square meters (areas).

    `tensile_area` and `shear_area` are supplied explicitly by the
    caller rather than assumed equal to the gross circular shank area --
    a threaded fastener's tensile stress area is smaller than its gross
    shank area, and this module does not assume otherwise. Use
    `circular_unthreaded_bolt()` only when an idealized shank-area
    approximation is explicitly wanted.
    """

    nominal_diameter: float
    tensile_area: float
    shear_area: float

    def __post_init__(self):
        for field_name in ("nominal_diameter", "tensile_area", "shear_area"):
            value = getattr(self, field_name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"BoltSection.{field_name} must be finite and > 0, got {value}.")


def circular_unthreaded_bolt(diameter: float) -> BoltSection:
    """Idealized bolt section using the gross circular shank area
    (A = pi*d^2/4) for BOTH the tensile and shear area.

    This is an idealized shank-area helper only -- it is NOT an actual
    ISO/SAE threaded tensile-stress-area calculation, which is smaller
    than the gross shank area for a real external thread. Use this only
    for order-of-magnitude / illustrative sizing studies.
    """
    if not math.isfinite(diameter) or diameter <= 0:
        raise ValueError(f"diameter must be finite and > 0, got {diameter}.")
    area = math.pi * diameter * diameter / 4.0
    return BoltSection(nominal_diameter=diameter, tensile_area=area, shear_area=area)


@dataclass(frozen=True)
class BoltStrengthResult:
    """Preliminary per-bolt strength result.

    All margins are "preliminary bolt strength margins" per this
    milestone's screening formulas, not certification margins. A margin
    field is `None` when that failure mode has zero demand at this bolt
    (not applicable), per this module's documented zero-demand
    convention (see README).
    """

    index: int
    axial_total: float  # signed, unchanged from Milestone 1
    tensile_load: float  # T_positive = max(axial_total, 0)
    shear_load: float  # V_resultant, unchanged from Milestone 1
    tensile_stress: float
    shear_stress: float
    tensile_margin: Optional[float]
    shear_margin: Optional[float]
    interaction_fi: float
    interaction_margin: Optional[float]
    governing_margin: Optional[float]
    governing_mode: Optional[str]  # "tension" | "shear" | "interaction" | None
    passed: bool


@dataclass(frozen=True)
class BoltGroupStrengthResult:
    """Group-level preliminary strength result."""

    bolts: Tuple[BoltStrengthResult, ...]
    bolt_section: BoltSection
    material: BoltMaterial
    governing_bolt_index: int
    governing_mode: Optional[str]
    governing_margin: Optional[float]
    passed: bool

    def governing_bolt(self) -> BoltStrengthResult:
        return self.bolts[self.governing_bolt_index]

    def min_tension_margin(self) -> Optional[float]:
        values = [b.tensile_margin for b in self.bolts if b.tensile_margin is not None]
        return min(values) if values else None

    def min_shear_margin(self) -> Optional[float]:
        values = [b.shear_margin for b in self.bolts if b.shear_margin is not None]
        return min(values) if values else None

    def min_interaction_margin(self) -> Optional[float]:
        values = [b.interaction_margin for b in self.bolts if b.interaction_margin is not None]
        return min(values) if values else None


def _governing_for_bolt(
    tensile_margin: Optional[float], shear_margin: Optional[float], interaction_margin: Optional[float]
) -> Tuple[Optional[float], Optional[str]]:
    """Deterministic per-bolt governing-mode selection.

    Governing margin is the smallest applicable (non-None) margin. If
    two applicable margins are numerically tied, ties are broken by the
    fixed mode order: interaction, then tension, then shear. If no mode
    is applicable (no demand at all at this bolt), returns (None, None).
    """
    candidates: List[Tuple[str, float]] = []
    if interaction_margin is not None:
        candidates.append(("interaction", interaction_margin))
    if tensile_margin is not None:
        candidates.append(("tension", tensile_margin))
    if shear_margin is not None:
        candidates.append(("shear", shear_margin))

    if not candidates:
        return None, None

    candidates.sort(key=lambda item: (item[1], _MODE_ORDER[item[0]]))
    mode, margin = candidates[0]
    return margin, mode


def _assess_single_bolt(index: int, axial_total: float, shear_load: float, section: BoltSection, material: BoltMaterial) -> BoltStrengthResult:
    tensile_load = max(axial_total, 0.0)

    sigma_t = tensile_load / section.tensile_area
    tau = shear_load / section.shear_area

    tensile_margin = (material.tensile_allowable / sigma_t - 1.0) if sigma_t > 0.0 else None
    shear_margin = (material.shear_allowable / tau - 1.0) if tau > 0.0 else None

    fi = (sigma_t / material.tensile_allowable) ** 2 + (tau / material.shear_allowable) ** 2
    interaction_margin = (1.0 / math.sqrt(fi) - 1.0) if fi > 0.0 else None

    governing_margin, governing_mode = _governing_for_bolt(tensile_margin, shear_margin, interaction_margin)
    passed = governing_margin is None or governing_margin >= 0.0

    return BoltStrengthResult(
        index=index,
        axial_total=axial_total,
        tensile_load=tensile_load,
        shear_load=shear_load,
        tensile_stress=sigma_t,
        shear_stress=tau,
        tensile_margin=tensile_margin,
        shear_margin=shear_margin,
        interaction_fi=fi,
        interaction_margin=interaction_margin,
        governing_margin=governing_margin,
        governing_mode=governing_mode,
        passed=passed,
    )


def assess_bolt_group_strength(
    group_load_result: BoltGroupResult, bolt_section: BoltSection, material: BoltMaterial
) -> BoltGroupStrengthResult:
    """Assess preliminary bolt tensile/shear strength for every bolt in
    a Milestone 1 `BoltGroupResult`, using one candidate `BoltSection`
    and `BoltMaterial` applied uniformly across the group.

    This function reuses the Milestone 1 per-bolt loads (`axial_total`,
    `shear_resultant`) exactly as computed by `distribute_loads` -- it
    does not recompute or modify any bolt-group mechanics.
    """
    per_bolt = tuple(
        _assess_single_bolt(b.index, b.axial_total, b.shear_resultant, bolt_section, material)
        for b in group_load_result.bolts
    )

    # Group governing bolt: smallest applicable governing margin across
    # bolts (None treated as +inf, i.e. "no constraint"), ties broken by
    # lowest bolt index. Each bolt's own governing mode was already
    # resolved deterministically above.
    def _sort_key(b: BoltStrengthResult):
        margin = b.governing_margin if b.governing_margin is not None else math.inf
        return (margin, b.index)

    governing_bolt = min(per_bolt, key=_sort_key)
    overall_passed = all(b.passed for b in per_bolt)

    return BoltGroupStrengthResult(
        bolts=per_bolt,
        bolt_section=bolt_section,
        material=material,
        governing_bolt_index=governing_bolt.index,
        governing_mode=governing_bolt.governing_mode,
        governing_margin=governing_bolt.governing_margin,
        passed=overall_passed,
    )


def evaluate_candidates(
    group_load_result: BoltGroupResult, candidates: Sequence[BoltSection], material: BoltMaterial
) -> List[BoltGroupStrengthResult]:
    """Assess a list of candidate bolt sections against the same
    Milestone 1 group load result, returned sorted by increasing
    nominal diameter (stable for equal diameters, preserving input
    order among ties)."""
    ordered = sorted(candidates, key=lambda s: s.nominal_diameter)
    return [assess_bolt_group_strength(group_load_result, section, material) for section in ordered]


def select_smallest_passing_bolt(
    group_load_result: BoltGroupResult, candidates: Sequence[BoltSection], material: BoltMaterial
) -> BoltGroupStrengthResult:
    """Return the strength result for the smallest-diameter candidate
    (in increasing-nominal-diameter order) that passes the preliminary
    strength assessment.

    Raises NoFeasibleCandidateError if no supplied candidate passes.
    Never enlarges beyond the supplied candidate list.
    """
    results = evaluate_candidates(group_load_result, candidates, material)
    for result in results:
        if result.passed:
            return result
    diameters = [r.bolt_section.nominal_diameter for r in results]
    raise NoFeasibleCandidateError(
        f"No candidate bolt section passed the preliminary strength assessment "
        f"out of {len(results)} candidate(s) with nominal diameters {diameters} m. "
        "Supply a larger or stronger candidate; Milestone 2 does not auto-enlarge."
    )
