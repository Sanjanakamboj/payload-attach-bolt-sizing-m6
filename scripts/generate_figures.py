"""Generate the STM-08 portfolio figure set into results/figures/
(Milestone 8).

Deterministic: every figure is produced from the exact same canonical
inputs used by the example scripts, via the public production API only
-- no equations are re-derived here.

Run with:
    python scripts/generate_figures.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from payload_bolts import (
    BoltMaterial,
    FrictionModel,
    InterfaceLoad,
    JointStiffness,
    PreloadState,
    TorquePreloadModel,
    circular_pattern,
    circular_unthreaded_bolt,
    distribute_loads,
    evaluate_candidates,
    evaluate_preloaded_candidates,
    max_allowable_scatter,
    preload_capacity_window,
    required_preload,
    torque_installation_window,
)

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"
FIGURES_DIR = RESULTS_DIR / "figures"

# ---------------------------------------------------------------------------
# Canonical STM-08 inputs (identical to every example script).
# ---------------------------------------------------------------------------
N_BOLTS = 8
RADIUS = 0.5
PATTERN = circular_pattern(N_BOLTS, RADIUS)
LOAD = InterfaceLoad(Fx=20_000.0, Fy=-10_000.0, Fz=80_000.0, Mx=25_000.0, My=-15_000.0, Mz=12_000.0)
GROUP = distribute_loads(PATTERN, LOAD)

MATERIAL = BoltMaterial(
    name="Illustrative high-strength steel bolt", tensile_allowable=800e6, shear_allowable=480e6, proof_allowable=600e6
)
STIFFNESS = JointStiffness(bolt_stiffness=1.0e8, member_stiffness=4.0e8)
FRICTION = FrictionModel(friction_coefficient=0.20)
CANDIDATE_DIAMETERS_MM = [5.0, 6.0, 8.0, 10.0, 12.0]
CANDIDATES = [circular_unthreaded_bolt(d / 1000.0) for d in CANDIDATE_DIAMETERS_MM]
PRELOAD_FACTOR = 1.2
TORQUE_MODEL = TorquePreloadModel(nut_factor=0.20, preload_scatter_fraction=0.20)


def _save(fig: plt.Figure, name: str) -> None:
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    path = FIGURES_DIR / name
    fig.savefig(path, dpi=200, bbox_inches="tight")
    print(f"wrote {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 1: bolt-group load distribution
# ---------------------------------------------------------------------------


def figure_1_load_distribution() -> None:
    fig, ax = plt.subplots(figsize=(7.0, 7.0))

    xs = [b.x for b in GROUP.bolts]
    ys = [b.y for b in GROUP.bolts]
    axial = [b.axial_total for b in GROUP.bolts]
    vx = [b.Vx_total for b in GROUP.bolts]
    vy = [b.Vy_total for b in GROUP.bolts]

    max_abs_axial = max(abs(a) for a in axial)
    max_v = max((vx_i**2 + vy_i**2) ** 0.5 for vx_i, vy_i in zip(vx, vy))

    # Bolt-circle reference.
    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(RADIUS * np.cos(theta), RADIUS * np.sin(theta), color="#999999", linewidth=0.8, linestyle=":", zorder=1)

    # Signed axial load: marker color (red=tension, blue=compression),
    # size scaled by |load|. Labels are placed radially OUTWARD from the
    # bolt-circle center (not on top of the marker/arrow) to avoid
    # overlap.
    for b, a in zip(GROUP.bolts, axial):
        color = "#c0392b" if a > 0 else "#2c6fa8"
        size = 200.0 + 900.0 * (abs(a) / max_abs_axial)
        ax.scatter(
            [b.x], [b.y], s=size, color=color, alpha=0.55, edgecolors="black", linewidths=0.8, zorder=3
        )
        r = math.hypot(b.x, b.y)
        ux, uy = (b.x / r, b.y / r) if r > 0 else (0.0, 1.0)
        label_x, label_y = b.x + ux * 0.135, b.y + uy * 0.135
        ax.annotate(
            f"#{b.index}: {a/1e3:+.1f} kN",
            (label_x, label_y),
            ha="center",
            va="center",
            fontsize=8,
            color=color,
            zorder=4,
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85),
        )

    # Shear resultant: arrows, scaled to a fixed max arrow length.
    arrow_scale = 0.14 / max_v  # meters per Newton, tuned so the largest arrow spans ~0.14 m
    for b, vx_i, vy_i in zip(GROUP.bolts, vx, vy):
        ax.annotate(
            "",
            xy=(b.x + vx_i * arrow_scale, b.y + vy_i * arrow_scale),
            xytext=(b.x, b.y),
            arrowprops=dict(arrowstyle="-|>", color="#1a1a1a", linewidth=1.6),
            zorder=5,
        )

    margin = RADIUS * 0.55
    ax.set_xlim(-RADIUS - margin, RADIUS + margin)
    ax.set_ylim(-RADIUS - margin, RADIUS + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        "Figure 1 — Bolt-group load distribution (8-bolt, R=0.5 m)\n"
        "marker size/color = signed axial load (red=tension, blue=compression); arrow = in-plane shear"
    )
    ax.text(
        0.02, 0.02,
        f"max tensile: bolt {GROUP.max_tensile_bolt().index}, {GROUP.max_tensile_bolt().axial_total/1e3:.2f} kN\n"
        f"max shear: bolt {GROUP.max_shear_bolt().index}, {GROUP.max_shear_bolt().shear_resultant/1e3:.2f} kN",
        transform=ax.transAxes, fontsize=8, va="bottom", ha="left",
        bbox=dict(boxstyle="round", fc="white", alpha=0.8, ec="none"),
    )
    fig.tight_layout()
    _save(fig, "fig1_bolt_group_load_distribution.png")


# ---------------------------------------------------------------------------
# Figure 2: candidate bolt sizing progression (M2 vs. M6)
# ---------------------------------------------------------------------------


def figure_2_candidate_progression() -> None:
    m2_results = evaluate_candidates(GROUP, CANDIDATES, MATERIAL)
    m6_results = evaluate_preloaded_candidates(GROUP, CANDIDATES, MATERIAL, STIFFNESS, FRICTION, PRELOAD_FACTOR)

    diam_mm = [r.bolt_section.nominal_diameter * 1000.0 for r in m2_results]
    m2_margins = [r.governing_margin for r in m2_results]
    m6_margins = [r.strength.governing_margin for r in m6_results]

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.axhline(0.0, color="black", linewidth=1.0, linestyle="--", label="zero-margin boundary")
    ax.plot(diam_mm, m2_margins, marker="o", color="#3a6f8f", linewidth=2.2, label="external-load-only (M2)")
    ax.plot(diam_mm, m6_margins, marker="s", color="#c9503a", linewidth=2.2, label="preload-integrated (M6, 1.2x required)")

    for d, m2, m6 in zip(diam_mm, m2_margins, m6_margins):
        ax.annotate(f"{m2:+.2f}", (d, m2), textcoords="offset points", xytext=(0, 8), ha="center", fontsize=7.5, color="#3a6f8f")
        ax.annotate(f"{m6:+.2f}", (d, m6), textcoords="offset points", xytext=(0, -13), ha="center", fontsize=7.5, color="#c9503a")

    ax.set_xlabel("Nominal bolt diameter [mm]")
    ax.set_ylabel("Governing margin of safety [-]")
    ax.set_title(
        "Figure 2 — Candidate bolt sizing progression\n"
        "external-load-only (M2) selects 8 mm; preload-integrated (M6) selects 10 mm"
    )
    ax.set_xticks(diam_mm)
    ax.legend(fontsize=8.5, loc="upper left")
    fig.tight_layout()
    _save(fig, "fig2_candidate_sizing_progression.png")


# ---------------------------------------------------------------------------
# Figure 3: preload feasibility window (8/10/12 mm)
# ---------------------------------------------------------------------------


def figure_3_preload_window() -> None:
    req = required_preload(GROUP, STIFFNESS, FRICTION)
    selected_preload = req.overall_required * PRELOAD_FACTOR
    diam_mm = [8.0, 10.0, 12.0]

    lowers, uppers = [], []
    for d in diam_mm:
        sec = circular_unthreaded_bolt(d / 1000.0)
        w = preload_capacity_window(GROUP, sec, MATERIAL, STIFFNESS, FRICTION)
        lowers.append(req.overall_required)
        uppers.append(w.ceilings.overall_ceiling)

    fig, ax = plt.subplots(figsize=(7.0, 4.6))
    y_pos = np.arange(len(diam_mm))
    for i, (lo, hi) in enumerate(zip(lowers, uppers)):
        color = "#6a8f5b" if selected_preload <= hi else "#c9503a"
        ax.plot([lo / 1e3, hi / 1e3], [y_pos[i], y_pos[i]], color=color, linewidth=6, solid_capstyle="round", zorder=2)
        ax.scatter([lo / 1e3], [y_pos[i]], color="#2b3a66", zorder=4, s=50, label="required (lower)" if i == 0 else None)
        ax.scatter([hi / 1e3], [y_pos[i]], color="#1c4a2e", zorder=4, s=50, label="max allowable (upper)" if i == 0 else None)
    ax.scatter(
        [selected_preload / 1e3] * len(diam_mm), y_pos, color="black", marker="x", s=90, zorder=5,
        label=f"selected preload ({PRELOAD_FACTOR:.1f}x required)",
    )

    ax.set_yticks(y_pos)
    ax.set_yticklabels([f"{d:.0f} mm" for d in diam_mm])
    ax.set_xlabel("Preload [kN/bolt]")
    ax.set_title(
        "Figure 3 — Preload feasibility window\n"
        "selected preload lies OUTSIDE the 8 mm window (red), inside 10/12 mm windows (green)"
    )
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    _save(fig, "fig3_preload_feasibility_window.png")


# ---------------------------------------------------------------------------
# Figure 4: torque installation robustness (8/10/12 mm at +/-20% scatter)
# ---------------------------------------------------------------------------


def figure_4_torque_robustness() -> None:
    req = required_preload(GROUP, STIFFNESS, FRICTION)
    diam_mm = [8.0, 10.0, 12.0]

    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    y_pos = np.arange(len(diam_mm))
    max_high_kn = 0.0

    for i, d in enumerate(diam_mm):
        sec = circular_unthreaded_bolt(d / 1000.0)
        w = preload_capacity_window(GROUP, sec, MATERIAL, STIFFNESS, FRICTION)
        ceiling = w.ceilings.overall_ceiling
        s_max = max_allowable_scatter(req.overall_required, ceiling)
        tw = torque_installation_window(req.overall_required, ceiling, d / 1000.0, TORQUE_MODEL)

        # Force window (background band), in kN.
        ax.plot([req.overall_required / 1e3, ceiling / 1e3], [y_pos[i], y_pos[i]], color="#c7c7c7", linewidth=10, zorder=1, solid_capstyle="round")

        # Achieved preload scatter band at a representative nominal preload
        # (midpoint of the two window-edge nominal preloads -- well
        # defined even when infeasible).
        nominal = 0.5 * (tw.nominal_preload_at_torque_min + tw.nominal_preload_at_torque_max)
        s = TORQUE_MODEL.preload_scatter_fraction
        low = nominal * (1 - s)
        high = nominal * (1 + s)
        color = "#6a8f5b" if tw.feasible else "#c9503a"
        ax.plot([low / 1e3, high / 1e3], [y_pos[i], y_pos[i]], color=color, linewidth=5, zorder=3, solid_capstyle="round")
        ax.scatter([nominal / 1e3], [y_pos[i]], color="black", marker="|", s=140, zorder=4)

        # Annotation is anchored to a FIXED fraction of the axes width
        # (not to the band's own right edge), so its position never
        # depends on how wide any individual band is -- this keeps every
        # row's text clear of the bands/markers regardless of diameter.
        s_max_label = f"s_max = {s_max*100:.1f}%"
        verdict_label = "FEASIBLE" if tw.feasible else "INFEASIBLE"
        verdict_color = "#3d6b34" if tw.feasible else "#a83a24"
        ax.annotate(
            s_max_label,
            xy=(0.985, y_pos[i] + 0.16),
            xycoords=("axes fraction", "data"),
            fontsize=8.5,
            va="center",
            ha="right",
            color="#333333",
        )
        ax.annotate(
            verdict_label,
            xy=(0.985, y_pos[i] - 0.16),
            xycoords=("axes fraction", "data"),
            fontsize=8.5,
            va="center",
            ha="right",
            color=verdict_color,
            fontweight="bold",
        )
        max_high_kn = max(max_high_kn, high / 1e3, ceiling / 1e3)

    ax.set_yticks(y_pos)
    ax.set_yticklabels([f"{d:.0f} mm" for d in diam_mm])
    ax.set_xlabel("Preload [kN/bolt]")
    ax.set_title(
        "Figure 4 — Torque-installation robustness at ±20% preload scatter\n"
        "gray band = force window [F_required, F_max]; colored band = achieved ±20% scatter"
    )
    # Extra right-hand margin reserved for the fixed-fraction annotation
    # column so it never overlaps the widest band (12 mm).
    ax.set_xlim(0, max_high_kn * 1.85)
    ax.set_ylim(-0.5, len(diam_mm) - 0.15)
    fig.tight_layout()
    _save(fig, "fig4_torque_installation_robustness.png")


def main() -> None:
    figure_1_load_distribution()
    figure_2_candidate_progression()
    figure_3_preload_window()
    figure_4_torque_robustness()
    print("All figures generated successfully.")


if __name__ == "__main__":
    main()
