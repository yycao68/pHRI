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
ax.set_xlim(-0.1, 8.5)
ax.set_ylim(3.6, 10.0)
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


ax.text(4.2, 9.8, "Behavior specification separated from constrained realization",
        ha="center", va="center", fontsize=15, fontweight="bold")

# --- Main vertical pipeline --------------------------------------------------

box_w = 3.4
box((1.0, 8.6), box_w, 0.8,
    [("Measured interaction state $(e,\\dot e)$", 10.5, "black", "normal", "normal"),
     ("and human force $F_h$", 10.5, "black", "normal", "normal")],
    fc=LIGHT, ec=GRAY)

box((1.0, 6.8), box_w, 1.2,
    [("Behavior Layer", 10.5, "black", "bold", "normal"),
     ("Desired-Acceleration Generator  $f_\\theta$", 9.5, "black", "normal", "normal"),
     ("validated here:", 8.3, GRAY, "normal", "italic"),
     ("impedance · admittance", 8.8, GRAY, "normal", "italic")],
    fc="white", ec=GRAY, dashed=True)

box((1.0, 5.1), box_w, 1.1,
    [("Realization Runtime", 11, "white", "bold", "normal"),
     ("predictive QP implementation", 8.5, "white", "normal", "normal"),
     ("$\\min\\;\\|a-a^{\\mathrm{id}}\\|_W^2$", 9.5, "white", "bold", "normal"),
     ("s.t. torque, workspace, speed, rate limits", 8.3, "white", "normal", "italic")],
    fc=BLUE, ec=BLUE)

box((1.0, 4.0), box_w, 0.5,
    [("Robot / Plant", 11, "white", "bold", "normal")],
    fc=GRAY, ec=GRAY)

arrow((2.8, 8.6), (2.8, 8.0), color="black")
arrow((2.8, 6.8), (2.8, 6.2), color="black")
ax.text(3.15, 6.5, "$a^{\\mathrm{id}}$", fontsize=10, ha="left", va="center")
arrow((2.8, 5.1), (2.8, 4.5), color="black")
ax.text(3.15, 4.8, "$F_{\\mathrm{cmd}}/\\tau$", fontsize=10, ha="left", va="center")

# --- Feedback loop: plant -> measured state, routed around the left --------

arrow((1.0, 4.25), (0.0, 4.25), color=GRAY, lw=1.6)
arrow((0.0, 4.25), (0.0, 9.0), color=GRAY, lw=1.6)
arrow((0.0, 9.0), (1.0, 9.0), color=GRAY, lw=1.6)
ax.text(0.2, 6.5, "closes the loop every tick", fontsize=8.8, color=GRAY,
        ha="center", va="center", rotation=90)

# --- Residual branch, routed around the right -------------------------------

box((5.6, 8.4), 2.8, 1.0,
    [("Total residual + audit", 11, ORANGE, "bold", "normal"),
     ("$r_k=a-a^{\\mathrm{id}}$", 11, ORANGE, "bold", "normal"),
     ("reg. / constraint / model", 8.6, ORANGE, "normal", "normal")],
    fc="white", ec=ORANGE, lw=2.2)

sum_x = 7.1
tap_y = 7.4
sum_xy = (sum_x, tap_y)
circ = Circle(sum_xy, 0.24, fc="white", ec=ORANGE, lw=2.0, zorder=3, clip_on=False)
ax.add_patch(circ)
ax.text(*sum_xy, "$\\Sigma$", ha="center", va="center", fontsize=11, color=ORANGE, zorder=4)

# a^id tap: top-right corner of generator box -> right -> down -> into LEFT side of sum node
arrow((4.4, tap_y), (sum_x-0.24, tap_y), color=ORANGE, lw=1.4)
#arrow((6, tap_y), (6, 6.05), color=ORANGE, lw=1.4)
#arrow((6, 6.05), (9.21, 6.05), color=ORANGE, lw=1.4)
ax.text(6.3, tap_y + 0.18, "$a^{\\mathrm{id}}$", fontsize=9.5, color=ORANGE, ha="center", va="bottom")

# a tap: plant -> right -> into BOTTOM-right of sum node
arrow((4.4, 4.25), (sum_x, 4.25), color=ORANGE, lw=1.4)
arrow((sum_x, 4.25), (sum_x, tap_y - 0.24), color=ORANGE, lw=1.4)
ax.text(6.3, 4.8, "$a$", fontsize=9.5, color=ORANGE, ha="center", va="top")

# summing node -> residual box (single clean output arrow, top of circle)
arrow((sum_x, tap_y+0.24), (sum_x, 8.4), color=ORANGE, lw=1.4)

output_dir = Path(__file__).resolve().parents[1] / "results"
output_dir.mkdir(parents=True, exist_ok=True)
fig.savefig(output_dir / "architecture_diagram.png", dpi=220, bbox_inches="tight")
print(f"saved {output_dir / 'architecture_diagram.png'}")
