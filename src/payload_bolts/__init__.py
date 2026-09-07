"""payload_bolts: rigid-interface bolt-group load distribution and
preliminary bolt strength sizing.

Milestone 1 scope: bolt-pattern geometry, applied interface loads, rigid
elastic bolt-group load distribution (direct shear, torsional shear,
axial/overturning), and independent equilibrium verification.

Milestone 2 scope: preliminary bolt tensile/shear strength screening on
top of the unchanged Milestone 1 loads -- bolt material/section
representation, tensile/shear stress, separate tensile/shear margins,
an illustrative quadratic tension-shear interaction check, deterministic
governing-bolt/mode identification, and candidate bolt-size comparison.

Milestone 3 scope: a first-order preloaded-joint closure and
friction-slip screen on top of the unchanged Milestone 1/2 results --
explicit bolt preload, a bolt/member load-fraction C, joint-separation
screening, remaining clamp force, local friction-capacity/slip
screening, deterministic governing bolt/constraint identification, and
analytical required-preload equations (not a torque specification).

Milestone 4 scope: preload feasibility, proof/yield screening, and an
installation-preload window on top of the unchanged Milestone 1-3
results -- illustrative bolt proof/yield strength limits reusing the
exact Milestone 2 tensile stress area, a proof-load-fraction
installation ceiling, a deterministic installation preload
scatter/loss allowance applied to the Milestone 3 required preload,
the resulting installation-preload window (honestly reported as
infeasible when it is), classification of a selected preload against
that window, and a maximum in-service bolt-tension screen against
proof/yield load using the Milestone 3 closed-joint load-sharing
formula. Not a torque specification, qualification procedure,
certification analysis, or detailed threaded-joint design.

Milestone 5 scope: a transparent conceptual bolt-size trade on top of
the unchanged Milestone 1-4 results -- for each candidate diameter,
Milestone 2 strength and Milestone 3/4 preload-window feasibility are
reused exactly and combined with new local-joint screens: bearing
stress at the plate/hole interface (using the Milestone 1 in-plane
shear resultant, never the axial/tensile load), an edge-distance
geometry screen, and a bolt-spacing geometry screen. Thread stripping
is explicitly NOT modeled (no source-verified formula could be
established from the thread-geometry information available to this
project) rather than approximated. A predeclared, deterministic
admissibility rule and smallest-admissible-diameter selection rule
resolve whether the Milestone 2 minimum-strength 8 mm bolt remains a
defensible conceptual choice once preload feasibility and local-joint
screening are added.

Milestone 6 scope: an integrated preload + bolt-strength assessment on
top of the unchanged Milestone 1-5 results -- a backward-compatible
optional `BoltMaterial.proof_allowable`, installation preload stress
against that proof allowable, total SERVICE bolt tension reusing
Milestone 3's closed-joint load sharing exactly (preload plus the
external tensile increment, not Milestone 2's external-load-only
force), unmodified Milestone 1 shear demand, and the exact Milestone 2
quadratic interaction criterion applied to the combined service
stresses. Explicit domain validity (the model is only valid while the
joint remains closed and unslipped, per Milestone 3), three independent
preload capacity ceilings (proof, service-tensile, interaction) and the
resulting feasible-preload window, and preload-compatible candidate
sizing that may select a different (typically larger) bolt than
Milestone 2's external-load-only result. A distinct, additive layer
alongside -- not a replacement for -- Milestone 4's force-based proof/
yield installation window or Milestone 5's local-joint screens.

Milestone 7 scope: a first-order torque-to-preload and preload-scatter
INSTALLATION model consuming, unmodified, the Milestone 3 required
preload and Milestone 6 preload capacity ceiling as a plain force
window -- an illustrative T=K*F*d torque/preload relation, a lumped
nut factor, a symmetric preload-scatter fraction, the installed preload
band for a commanded torque, the closed-form robust torque window
(both scatter tails must remain inside the force window), the maximum
allowable scatter fraction (the central diagnostic: a feasible force
window can still be installation-infeasible once scatter exceeds this
value), and an illustrative midpoint installation-torque target. Not a
torque specification, thread-friction model, or installation standard.

Explicitly out of scope through Milestone 7 (deferred to later
milestones): detailed thread geometry, pitch-dependent thread-torque
decomposition, separate thread/under-head friction coefficients,
prevailing torque, torque-angle tightening, direct-tension-indicating
fasteners, ultrasonic preload measurement, preload relaxation/
embedment, thermal preload change, fatigue, prying, nonlinear plate
flexibility, detailed flange bending, detailed bearing/tear-out
interaction, net-section rupture, nonlinear contact FEA, fracture
mechanics, thread stripping (explicitly not modeled, see above), proof
testing, detailed fastener standards/database lookup, structural
optimization, and certification/qualification.
"""

from .geometry import BoltPattern, circular_pattern, rectangular_pattern
from .loads import InterfaceLoad, inertial_force
from .solver import (
    BoltLoadResult,
    BoltGroupResult,
    EquilibriumCheck,
    distribute_loads,
)
from .strength import (
    BoltMaterial,
    BoltSection,
    BoltStrengthResult,
    BoltGroupStrengthResult,
    NoFeasibleCandidateError,
    circular_unthreaded_bolt,
    assess_bolt_group_strength,
    evaluate_candidates,
    select_smallest_passing_bolt,
)
from .preload import (
    PreloadState,
    JointStiffness,
    FrictionModel,
    BoltPreloadResult,
    BoltPreloadGroupResult,
    RequiredPreloadResult,
    assess_preloaded_joint,
    required_preload,
    apply_preload_factor,
)
from .preload_limits import (
    BoltStrengthLimits,
    PreloadLimitResult,
    InstallationPreloadWindow,
    PreloadFeasibilityStatus,
    PreloadFeasibilityResult,
    compute_preload_limits,
    min_installation_preload,
    max_installation_preload,
    installation_preload_window,
    classify_selected_preload,
    assess_preload_feasibility,
)
from .joint_local_checks import (
    PlateMaterial,
    JointGeometry,
    CheckStatus,
    PerBoltBearing,
    BearingCheckResult,
    PerBoltEdgeDistance,
    EdgeDistanceCheckResult,
    PairSpacing,
    SpacingCheckResult,
    ThreadCheckStatus,
    ThreadStripCheckResult,
    BoltCandidateTradeResult,
    BoltSelectionResult,
    assess_bearing,
    assess_edge_distance,
    assess_spacing,
    thread_strip_not_modeled,
    evaluate_candidate_trade,
    select_bolt_candidate,
)
from .preloaded_strength import (
    PreloadedBoltStrengthResult,
    PreloadedBoltGroupStrengthResult,
    PreloadCeilings,
    PreloadCapacityWindow,
    PreloadedCandidateResult,
    assess_preloaded_bolt_strength,
    compute_preload_ceilings,
    preload_capacity_window,
    evaluate_preloaded_candidates,
    select_smallest_passing_preloaded_bolt,
)
from .torque_preload import (
    TorquePreloadModel,
    InstalledPreloadBand,
    TorqueInstallationWindow,
    SelectedInstallationTarget,
    NoRobustTorqueWindowError,
    torque_from_preload,
    preload_from_torque,
    installed_preload_band,
    torque_installation_window,
    max_allowable_scatter,
    select_installation_target,
)

__all__ = [
    "BoltPattern",
    "circular_pattern",
    "rectangular_pattern",
    "InterfaceLoad",
    "inertial_force",
    "BoltLoadResult",
    "BoltGroupResult",
    "EquilibriumCheck",
    "distribute_loads",
    "BoltMaterial",
    "BoltSection",
    "BoltStrengthResult",
    "BoltGroupStrengthResult",
    "NoFeasibleCandidateError",
    "circular_unthreaded_bolt",
    "assess_bolt_group_strength",
    "evaluate_candidates",
    "select_smallest_passing_bolt",
    "PreloadState",
    "JointStiffness",
    "FrictionModel",
    "BoltPreloadResult",
    "BoltPreloadGroupResult",
    "RequiredPreloadResult",
    "assess_preloaded_joint",
    "required_preload",
    "apply_preload_factor",
    "BoltStrengthLimits",
    "PreloadLimitResult",
    "InstallationPreloadWindow",
    "PreloadFeasibilityStatus",
    "PreloadFeasibilityResult",
    "compute_preload_limits",
    "min_installation_preload",
    "max_installation_preload",
    "installation_preload_window",
    "classify_selected_preload",
    "assess_preload_feasibility",
    "PlateMaterial",
    "JointGeometry",
    "CheckStatus",
    "PerBoltBearing",
    "BearingCheckResult",
    "PerBoltEdgeDistance",
    "EdgeDistanceCheckResult",
    "PairSpacing",
    "SpacingCheckResult",
    "ThreadCheckStatus",
    "ThreadStripCheckResult",
    "BoltCandidateTradeResult",
    "BoltSelectionResult",
    "assess_bearing",
    "assess_edge_distance",
    "assess_spacing",
    "thread_strip_not_modeled",
    "evaluate_candidate_trade",
    "select_bolt_candidate",
    "PreloadedBoltStrengthResult",
    "PreloadedBoltGroupStrengthResult",
    "PreloadCeilings",
    "PreloadCapacityWindow",
    "PreloadedCandidateResult",
    "assess_preloaded_bolt_strength",
    "compute_preload_ceilings",
    "preload_capacity_window",
    "evaluate_preloaded_candidates",
    "select_smallest_passing_preloaded_bolt",
    "TorquePreloadModel",
    "InstalledPreloadBand",
    "TorqueInstallationWindow",
    "SelectedInstallationTarget",
    "NoRobustTorqueWindowError",
    "torque_from_preload",
    "preload_from_torque",
    "installed_preload_band",
    "torque_installation_window",
    "max_allowable_scatter",
    "select_installation_target",
]

__version__ = "0.1.0"
