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
ax.set_xlim(-0.6, 9.0)
ax.set_ylim(3.6, 11.0)
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

box((0.7, 9.5), 4.0, 0.95,
    [("Measured interaction state $(e,\\dot e)$", 10.5, "black", "normal", "normal"),
     ("and human force $F_h$", 10.5, "black", "normal", "normal")],
    fc=LIGHT, ec=GRAY)

box((0.7, 7.4), 4.0, 1.35,
    [("Interaction-Dynamics Generator  $f_\\theta$", 10, "black", "bold", "normal"),
     ("(pluggable)", 10, "black", "normal", "normal"),
     ("impedance · admittance · energy-tank passive ·", 8.3, GRAY, "normal", "italic"),
     ("human model · learned model", 8.3, GRAY, "normal", "italic")],
    fc="white", ec=GRAY, dashed=True)

box((0.7, 5.4), 4.0, 1.35,
    [("Predictive Realization", 11, "white", "bold", "normal"),
     ("$\\min\\;\\|a-a^{\\mathrm{id}}\\|_W^2$", 10, "white", "bold", "normal"),
     ("s.t. torque, workspace, speed, rate limits", 8.3, "white", "normal", "italic")],
    fc=BLUE, ec=BLUE)

box((0.7, 4.0), 4.0, 0.75,
    [("Robot / Plant", 11, "white", "bold", "normal")],
    fc=GRAY, ec=GRAY)

arrow((2.8, 9.5), (2.8, 8.8), color="black")
arrow((2.8, 7.35), (2.8, 6.7), color="black")
ax.text(3.15, 7.05, "$a^{\\mathrm{id}}$", fontsize=10, ha="left", va="center")
arrow((2.8, 5.4), (2.8, 4.75), color="black")
ax.text(3.15, 5.1, "$F_{\\mathrm{cmd}}/\\tau$", fontsize=10, ha="left", va="center")

# --- Feedback loop: plant -> measured state, routed around the left --------

arrow((0.7, 4.4), (-0.5, 4.4), color=GRAY, lw=1.6)
arrow((-0.5, 4.4), (-0.5, 10.02), color=GRAY, lw=1.6)
arrow((-0.5, 10.02), (0.7, 10.02), color=GRAY, lw=1.6)
ax.text(-0.2, 7.1, "closes the loop every tick", fontsize=8.8, color=GRAY,
        ha="center", va="center", rotation=90)

# --- Residual branch, routed around the right -------------------------------

box((6.1, 9.2), 2.8, 1.2,
    [("Realization residual", 11, ORANGE, "bold", "normal"),
     ("$r_k=a-a^{\\mathrm{id}}$", 11, ORANGE, "bold", "normal"),
     ("reported, not hidden", 9., ORANGE, "normal", "normal")],
    fc="white", ec=ORANGE, lw=2.2)

sum_x = 7.5
sum_xy = (sum_x, 8)
circ = Circle(sum_xy, 0.24, fc="white", ec=ORANGE, lw=2.0, zorder=3, clip_on=False)
ax.add_patch(circ)
ax.text(*sum_xy, "$\\Sigma$", ha="center", va="center", fontsize=11, color=ORANGE, zorder=4)

# a^id tap: top-right corner of generator box -> right -> down -> into LEFT side of sum node
tap_y = 8.0
arrow((4.7, tap_y), (7.26, tap_y), color=ORANGE, lw=1.4)
#arrow((6, tap_y), (6, 6.05), color=ORANGE, lw=1.4)
#arrow((6, 6.05), (9.21, 6.05), color=ORANGE, lw=1.4)
ax.text(6.3, tap_y + 0.18, "$a^{\\mathrm{id}}$", fontsize=9.5, color=ORANGE, ha="center", va="bottom")

# a tap: plant -> right -> into BOTTOM-right of sum node
arrow((4.7, 4.4), (sum_x, 4.4), color=ORANGE, lw=1.4)
arrow((sum_x, 4.44), (sum_x, 7.76), color=ORANGE, lw=1.4)
ax.text(6.3, 4.8, "$a$", fontsize=9.5, color=ORANGE, ha="center", va="top")

# summing node -> residual box (single clean output arrow, top of circle)
arrow((sum_x, 8.24), (sum_x, 9.2), color=ORANGE, lw=1.4)

output_dir = Path(__file__).resolve().parents[1] / "results"
output_dir.mkdir(parents=True, exist_ok=True)
fig.savefig(output_dir / "architecture_diagram.png", dpi=220, bbox_inches="tight")
print(f"saved {output_dir / 'architecture_diagram.png'}")
