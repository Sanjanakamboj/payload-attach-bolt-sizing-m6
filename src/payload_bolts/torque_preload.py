"""First-order torque-to-preload and preload-scatter installation model
(Milestone 7).

Milestone 6 established a FORCE-based preload feasibility window:

    F_required_joint  <=  F_preload  <=  F_preload,max_bolt

(the lower bound from Milestone 3's `required_preload()`, the upper
bound from Milestone 6's preload capacity ceilings, e.g.
`preload_capacity_window(...).ceilings.overall_ceiling`). This module
does **not** recompute or modify either bound -- it consumes them as
plain float inputs, exactly as computed by the existing Milestone 3/6
functions.

Milestone 7 asks a different, narrower question: what installation
TORQUE range maps into that already-accepted force window, once a
simple torque-to-preload relation and a symmetric preload-scatter
uncertainty are included?

Torque-to-preload relation (illustrative, first-order):

    T_install = K * F_preload * d
    F_preload = T_install / (K * d)

where K (the "nut factor" or "torque coefficient") is a single lumped,
illustrative coefficient standing in for BOTH thread friction and
under-head/bearing friction combined -- it is emphatically NOT a
universal constant: real nut factors vary substantially with
lubrication, plating/coating, surface finish, and tightening method
(commonly cited ranges span roughly 0.10-0.30 depending on condition).
This module makes no claim about which real fastener/lubricant
combination corresponds to any K value used here.

Preload scatter is modeled as a single symmetric fractional band around
the nominal (K,d)-predicted preload:

    F_low  = F_nominal * (1 - s)
    F_high = F_nominal * (1 + s)

This is a deterministic illustrative uncertainty allowance, not a
statistical confidence interval and not sourced from a specific torque
wrench/method specification.

Explicitly NOT modeled here (deferred): detailed thread geometry,
pitch-dependent thread-torque decomposition, separate thread/under-head
friction coefficients, prevailing torque, torque-angle tightening,
direct-tension-indicating fasteners, ultrasonic preload measurement,
embedment loss, relaxation, thermal preload change, fatigue, or any
detailed installation standard.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


class NoRobustTorqueWindowError(ValueError):
    """Raised when a robust installation target is requested but no
    feasible torque window exists (T_min > T_max)."""


# ---------------------------------------------------------------------------
# Torque/preload model and direct helpers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TorquePreloadModel:
    """An illustrative torque-to-preload installation model.

    nut_factor (K): dimensionless, > 0, finite. Lumps thread AND
        under-head/bearing friction into a single coefficient -- not a
        universal constant, and not associated with any specific real
        lubricant/coating/surface condition unless the caller
        explicitly sources it.
    preload_scatter_fraction (s): dimensionless, in [0, 1), finite.
        Symmetric fractional installation-preload uncertainty around
        the nominal (K,d)-predicted preload for a commanded torque.
        `s = 0.20` means the achieved preload for a given commanded
        torque is modeled as ranging from -20% to +20% of nominal.
    label: optional free-text description.
    """

    nut_factor: float
    preload_scatter_fraction: float
    label: str = ""

    def __post_init__(self) -> None:
        if not math.isfinite(self.nut_factor) or self.nut_factor <= 0.0:
            raise ValueError(f"nut_factor must be finite and > 0, got {self.nut_factor}.")
        if not math.isfinite(self.preload_scatter_fraction) or not (0.0 <= self.preload_scatter_fraction < 1.0):
            raise ValueError(
                f"preload_scatter_fraction must be finite and in [0, 1), got {self.preload_scatter_fraction}."
            )


def _validate_positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and > 0, got {value}.")


def torque_from_preload(preload: float, diameter: float, nut_factor: float) -> float:
    """T_install = K * F_preload * d [N*m]. Exact algebraic relation --
    the inverse of `preload_from_torque`."""
    _validate_positive_finite("preload", preload)
    _validate_positive_finite("diameter", diameter)
    _validate_positive_finite("nut_factor", nut_factor)
    return nut_factor * preload * diameter


def preload_from_torque(torque: float, diameter: float, nut_factor: float) -> float:
    """F_preload = T_install / (K * d) [N]. Exact algebraic relation --
    the inverse of `torque_from_preload`."""
    _validate_positive_finite("torque", torque)
    _validate_positive_finite("diameter", diameter)
    _validate_positive_finite("nut_factor", nut_factor)
    return torque / (nut_factor * diameter)


# ---------------------------------------------------------------------------
# Installed preload band
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class InstalledPreloadBand:
    """Nominal/min/max preload achieved by a commanded installation
    torque, given a `TorquePreloadModel`."""

    torque: float
    diameter: float
    nut_factor: float
    scatter_fraction: float
    nominal_preload: float
    min_preload: float
    max_preload: float


def installed_preload_band(torque: float, diameter: float, model: TorquePreloadModel) -> InstalledPreloadBand:
    """F_nominal = T/(K*d); F_low = F_nominal*(1-s); F_high = F_nominal*(1+s)."""
    nominal = preload_from_torque(torque, diameter, model.nut_factor)
    s = model.preload_scatter_fraction
    return InstalledPreloadBand(
        torque=torque,
        diameter=diameter,
        nut_factor=model.nut_factor,
        scatter_fraction=s,
        nominal_preload=nominal,
        min_preload=nominal * (1.0 - s),
        max_preload=nominal * (1.0 + s),
    )


# ---------------------------------------------------------------------------
# Robust torque installation window
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TorqueInstallationWindow:
    """Robust torque window mapping the Milestone 3/6 FORCE window
    through the torque-to-preload relation and preload scatter.

    A commanded torque is "robustly acceptable" only if BOTH scatter
    tails remain inside the force window:

        F_low  = [T/(K*d)]*(1-s)  >= required_preload
        F_high = [T/(K*d)]*(1+s)  <= max_preload

    which gives the closed-form bounds:

        T_min = K*d*required_preload / (1-s)
        T_max = K*d*max_preload     / (1+s)

    A robust window exists iff T_min <= T_max. This can fail (T_min >
    T_max) even when the underlying FORCE window is feasible
    (required_preload <= max_preload) -- scatter can make an otherwise
    feasible force window impossible to install robustly. That is the
    key Milestone 7 result; see `max_allowable_scatter` for the
    corresponding diagnostic.
    """

    diameter: float
    nut_factor: float
    scatter_fraction: float
    required_preload: float  # lower bound, unchanged from Milestone 3
    max_preload: float  # upper bound, unchanged from Milestone 6
    torque_min: float
    torque_max: float
    torque_window_width: float  # torque_max - torque_min, unclipped (may be negative)
    feasible: bool
    nominal_preload_at_torque_min: float  # == required_preload / (1 - s)
    nominal_preload_at_torque_max: float  # == max_preload / (1 + s)
    force_window_width: float  # max_preload - required_preload, unclipped


def torque_installation_window(
    required_preload: float, max_preload: float, diameter: float, model: TorquePreloadModel
) -> TorqueInstallationWindow:
    """Build the robust torque installation window from the existing
    Milestone 3 (`required_preload`) and Milestone 6 (`max_preload`)
    force-window bounds -- neither is recomputed here."""
    _validate_positive_finite("required_preload", required_preload)
    _validate_positive_finite("max_preload", max_preload)
    _validate_positive_finite("diameter", diameter)

    k = model.nut_factor
    s = model.preload_scatter_fraction

    nominal_at_min = required_preload / (1.0 - s)
    nominal_at_max = max_preload / (1.0 + s)
    torque_min = k * diameter * nominal_at_min
    torque_max = k * diameter * nominal_at_max
    width = torque_max - torque_min

    return TorqueInstallationWindow(
        diameter=diameter,
        nut_factor=k,
        scatter_fraction=s,
        required_preload=required_preload,
        max_preload=max_preload,
        torque_min=torque_min,
        torque_max=torque_max,
        torque_window_width=width,
        feasible=(torque_min <= torque_max),
        nominal_preload_at_torque_min=nominal_at_min,
        nominal_preload_at_torque_max=nominal_at_max,
        force_window_width=max_preload - required_preload,
    )


def max_allowable_scatter(required_preload: float, max_preload: float) -> float:
    """Maximum symmetric preload-scatter fraction that still permits
    SOME nominal preload satisfying both:

        F_nom*(1-s) >= required_preload
        F_nom*(1+s) <= max_preload

    At the limiting (zero-width) case, F_required/(1-s) ==
    F_max/(1+s), giving the closed form:

        s_max = (F_max - F_required) / (F_max + F_required)

    Returns a negative value if max_preload < required_preload (the
    underlying FORCE window is already infeasible -- no scatter, however
    small, can be tolerated); returns exactly 0.0 for a zero-width force
    window; otherwise 0 < s_max < 1. Never clipped -- a negative result
    is a valid, honest diagnostic, not an error.
    """
    _validate_positive_finite("required_preload", required_preload)
    _validate_positive_finite("max_preload", max_preload)
    return (max_preload - required_preload) / (max_preload + required_preload)


# ---------------------------------------------------------------------------
# Selected installation target (illustrative midpoint policy)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SelectedInstallationTarget:
    """An illustrative midpoint installation target within a robust
    `TorqueInstallationWindow`. NOT claimed to be an optimum -- see
    module docstring and `select_installation_target` docstring."""

    torque_selected: float
    nominal_preload_selected: float
    low_preload: float
    high_preload: float
    reserve_to_lower_bound: float  # low_preload - required_preload
    reserve_to_upper_bound: float  # max_preload - high_preload


def select_installation_target(window: TorqueInstallationWindow) -> SelectedInstallationTarget:
    """Choose an illustrative midpoint installation target: the nominal
    preload midway between the two feasible-nominal-preload extremes
    (`nominal_preload_at_torque_min`, `nominal_preload_at_torque_max`),
    then convert that preload to a torque via the SAME (K, d) relation
    used throughout -- torque and nominal preload are linearly
    equivalent for fixed K and d, so a preload-space midpoint and a
    torque-space midpoint coincide here; the choice is stated as a
    policy, not derived as an optimum.

    Raises NoRobustTorqueWindowError if `window.feasible` is False.
    """
    if not window.feasible:
        raise NoRobustTorqueWindowError(
            f"No robust torque window exists (torque_min={window.torque_min:.4f} N*m > "
            f"torque_max={window.torque_max:.4f} N*m); cannot select an installation target."
        )
    f_nom_selected = 0.5 * (window.nominal_preload_at_torque_min + window.nominal_preload_at_torque_max)
    s = window.scatter_fraction
    low = f_nom_selected * (1.0 - s)
    high = f_nom_selected * (1.0 + s)
    torque_selected = window.nut_factor * window.diameter * f_nom_selected
    return SelectedInstallationTarget(
        torque_selected=torque_selected,
        nominal_preload_selected=f_nom_selected,
        low_preload=low,
        high_preload=high,
        reserve_to_lower_bound=low - window.required_preload,
        reserve_to_upper_bound=window.max_preload - high,
    )
