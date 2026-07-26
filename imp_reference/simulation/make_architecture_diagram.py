"""Generate the generator-realizer architecture schematic for paper.md Figure 1."""
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_imp_reference_mpl")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle

BLUE = "#0072B2"
ORANGE = "#D55E00"
GRAY = "#4D4D4D"
LIGHT = "#F2F2F2"

fig, ax = plt.subplots(figsize=(10.2, 9.0))
ax.set_xlim(-2.8, 11.2)
ax.set_ylim(3.6, 11.3)
ax.set_aspect("equal")
ax.axis("off")


def box(xy, w, h, lines, *, fc="white", ec="black", lw=1.8, dashed=False):
    """lines: list of (text, fontsize, color, weight, style) tuples, stacked top-to-bottom."""
    x, y = xy
    b = FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", fc=fc, ec=ec, lw=lw,
        linestyle="dashed" if dashed else "solid", zorder=3, clip_on=False,
    )
    ax.add_patch(b)
    n = len(lines)
    top = y + h - h / (n + 1)
    step = h / (n + 1)
    for i, (text, fontsize, color, weight, style) in enumerate(lines):
        ax.text(x + w / 2, top - i * step, text, ha="center", va="center",
                 fontsize=fontsize, color=color, zorder=4, fontweight=weight,
                 fontstyle=style, linespacing=1.3)
    return b


def arrow(p0, p1, *, color="black", lw=1.8, connectionstyle="arc3,rad=0.0"):
    a = FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=15,
                         color=color, lw=lw, connectionstyle=connectionstyle,
                         zorder=2, clip_on=False, shrinkA=0, shrinkB=0)
    ax.add_patch(a)


ax.text(4.0, 10.95, "One realization layer, any interaction dynamics",
        ha="center", va="center", fontsize=15, fontweight="bold")

# --- Main vertical pipeline --------------------------------------------------

box((1.0, 9.55), 5.2, 0.95,
    [("Measured interaction state $(e,\\dot e)$", 10.5, "black", "normal", "normal"),
     ("and human force $F_h$", 10.5, "black", "normal", "normal")],
    fc=LIGHT, ec=GRAY)

box((1.0, 7.75), 5.2, 1.35,
    [("Interaction-Dynamics Generator  $f_\\theta$", 12, "black", "bold", "normal"),
     ("(pluggable, §3.3)", 10, "black", "bold", "normal"),
     ("impedance · admittance · energy-tank passive ·", 8.3, GRAY, "normal", "italic"),
     ("human model · learned model  (§4)", 8.3, GRAY, "normal", "italic")],
    fc="white", ec=GRAY, dashed=True)

box((1.0, 6.0), 5.2, 1.35,
    [("Predictive Realization  (QP, §5)", 12, "white", "bold", "normal"),
     ("$\\min\\;\\|a-a^{\\mathrm{id}}\\|_W^2$", 12, "white", "bold", "normal"),
     ("s.t. torque, workspace, speed, rate limits", 8.3, "white", "normal", "italic"),
     ("(§6, §8.1)", 8.3, "white", "normal", "italic")],
    fc=BLUE, ec=BLUE)

box((1.0, 4.85), 5.2, 0.75,
    [("Robot / Plant  (§3.1, §8.1)", 11.5, "white", "bold", "normal")],
    fc=GRAY, ec=GRAY)

arrow((3.6, 9.55), (3.6, 9.10), color="black")
arrow((3.6, 7.75), (3.6, 7.35), color="black")
ax.text(3.95, 7.55, "$a^{\\mathrm{id}}$", fontsize=10, ha="left", va="center")
arrow((3.6, 6.0), (3.6, 5.60), color="black")
ax.text(3.95, 5.8, "$F_{\\mathrm{cmd}}/\\tau$", fontsize=10, ha="left", va="center")

# --- Feedback loop: plant -> measured state, routed around the left --------

arrow((1.0, 5.22), (-1.8, 5.22), color=GRAY, lw=1.6)
arrow((-1.8, 5.22), (-1.8, 10.02), color=GRAY, lw=1.6)
arrow((-1.8, 10.02), (1.0, 10.02), color=GRAY, lw=1.6)
ax.text(-2.3, 7.6, "closes the loop every tick", fontsize=8.8, color=GRAY,
        ha="center", va="center", rotation=90)

# --- Residual branch, routed around the right -------------------------------

box((8.1, 7.55), 2.7, 1.85,
    [("Realization residual", 11, ORANGE, "bold", "normal"),
     ("$r_k=a-a^{\\mathrm{id}}$", 11, ORANGE, "bold", "normal"),
     ("reported, not hidden", 9.3, ORANGE, "bold", "normal"),
     ("(§3.4)", 9.3, ORANGE, "bold", "normal")],
    fc="white", ec=ORANGE, lw=2.2)

sum_xy = (9.45, 6.05)
circ = Circle(sum_xy, 0.24, fc="white", ec=ORANGE, lw=2.0, zorder=3, clip_on=False)
ax.add_patch(circ)
ax.text(*sum_xy, "$\\Sigma$", ha="center", va="center", fontsize=11, color=ORANGE, zorder=4)

# a^id tap: top-right corner of generator box -> right -> down -> into LEFT side of sum node
tap_y = 8.42
arrow((6.2, tap_y), (7.9, tap_y), color=ORANGE, lw=1.4)
arrow((7.9, tap_y), (7.9, 6.05), color=ORANGE, lw=1.4)
arrow((7.9, 6.05), (9.21, 6.05), color=ORANGE, lw=1.4)
ax.text(7.3, tap_y + 0.18, "$a^{\\mathrm{id}}$", fontsize=9.5, color=ORANGE, ha="center", va="bottom")

# a tap: plant -> right -> into BOTTOM-right of sum node
arrow((6.2, 5.22), (9.0, 5.22), color=ORANGE, lw=1.4)
arrow((9.0, 5.22), (9.29, 5.88), color=ORANGE, lw=1.4)
ax.text(7.6, 5.02, "$a$", fontsize=9.5, color=ORANGE, ha="center", va="top")

# summing node -> residual box (single clean output arrow, top of circle)
arrow((9.45, 6.29), (9.45, 7.55), color=ORANGE, lw=2.0)

output_dir = Path(__file__).resolve().parents[1] / "results"
output_dir.mkdir(parents=True, exist_ok=True)
fig.savefig(output_dir / "architecture_diagram.png", dpi=220, bbox_inches="tight")
print(f"saved {output_dir / 'architecture_diagram.png'}")
