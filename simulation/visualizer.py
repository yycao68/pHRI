"""
Real-time simulation dashboard for FR3 MuJoCo.

Layout
------
Left column (full height): 3D EE trajectory
Right columns (3 × 2 grid):
    [pos error]   [EE position x/y/z]
    [ext force]   [joint torques]
    [EE speed ]   [mode-specific panel]

Usage
-----
    viz = SimVisualizer(mode='impedance', dt=env.dt, x_d=x_d)
    for i in range(n_steps):
        viz.push(env.time,
                 pos_err=err, ee_pos=state.ee_pos,
                 tau=tau, dq=state.dq,
                 f_ext=np.linalg.norm(state.f_ext[:3]),
                 ee_vel=np.linalg.norm(state.ee_vel[:3]))
        if i % viz.update_every == 0:
            viz.refresh()
    viz.save('result.png')
    viz.close()
"""

from __future__ import annotations
import numpy as np

# ── lazy matplotlib import ────────────────────────────────────────────────────
_plt = None

def _get_plt():
    global _plt
    if _plt is None:
        import matplotlib
        for backend in ("TkAgg", "Qt5Agg", "MacOSX", "Agg"):
            try:
                matplotlib.use(backend)
                import matplotlib.pyplot as plt
                _plt = plt
                break
            except Exception:
                continue
    return _plt


# ── colour palette ────────────────────────────────────────────────────────────
_BG_DARK   = "#1e1e2e"
_BG_PANEL  = "#2d2d3f"
_GRID_CLR  = "#444444"
_TICK_CLR  = "#aaaaaa"
_COLORS    = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
              "#9467bd", "#8c564b", "#e377c2"]
_JOINT_LBL = [f"J{i+1}" for i in range(7)]

# Trail length: keep the last N points bright, fade older ones
_TRAIL_RECENT = 200   # steps shown in bright colour


class SimVisualizer:
    """
    Live matplotlib dashboard with a 3D EE trajectory panel.

    Parameters
    ----------
    mode         : 'impedance' | 'phri' | 'rl_random'
    dt           : simulation timestep (s)
    x_d          : (3,) desired EE position
    update_every : refresh every this many sim steps (default ~100 ms)
    figsize      : figure size in inches
    """

    def __init__(
        self,
        mode:         str = "impedance",
        dt:           float = 0.002,
        x_d:          np.ndarray | None = None,
        update_every: int | None = None,
        figsize:      tuple = (18, 8),
    ):
        self.mode         = mode
        self.dt           = dt
        self.x_d          = np.asarray(x_d, dtype=float) if x_d is not None else None
        self.update_every = update_every or max(1, int(0.10 / dt))

        # ── data buffers ──────────────────────────────────────────────────
        self._t       : list[float]      = []
        self._pos_err : list[float]      = []
        self._ee_pos  : list[np.ndarray] = []
        self._ee_vel  : list[float]      = []
        self._tau     : list[np.ndarray] = []
        self._dq      : list[np.ndarray] = []
        self._f_ext   : list[float]      = []
        self._contact : list[float]      = []
        self._cusum   : list[float]      = []
        self._params  : list[np.ndarray] = []
        self._reward  : list[float]      = []

        # ── figure ────────────────────────────────────────────────────────
        plt = _get_plt()
        plt.ion()
        self.fig = plt.figure(figsize=figsize)
        self.fig.patch.set_facecolor(_BG_DARK)
        self.fig.suptitle(
            f"FR3 Simulation  ·  {mode}  mode",
            color="white", fontsize=13, fontweight="bold", y=0.98,
        )

        self._build_layout()
        try:
            plt.tight_layout(rect=[0, 0, 1, 0.96])
        except Exception:
            pass  # 3D axes are not always compatible with tight_layout
        plt.pause(0.05)

    # ─────────────────────────────────────────────────────────────────────
    # Layout
    # ─────────────────────────────────────────────────────────────────────

    def _build_layout(self):
        from matplotlib.gridspec import GridSpec
        # 3 rows × 3 cols; left column spans all rows → 3-D plot
        gs = GridSpec(3, 3, figure=self.fig,
                      left=0.06, right=0.97, top=0.93, bottom=0.08,
                      hspace=0.55, wspace=0.38)

        # ── 3-D EE trajectory (left column, full height) ──────────────────
        self._ax3d = self.fig.add_subplot(gs[:, 0], projection="3d")
        self._setup_3d_axes()

        # right 2×3 panels
        self.fig.add_subplot(gs[0, 1])
        self._ax_pos_err = self._style_ax("Position error  |x − x_d|", ylabel="m")
        self._line_pos_err, = self._ax_pos_err.plot([], [], color="#4fc3f7", lw=1.5)
        self._ax_pos_err.axhline(0, color="#555", lw=0.6)

        self.fig.add_subplot(gs[0, 2])
        self._ax_ee_pos = self._style_ax("EE position", ylabel="m")
        self._lines_ee_pos = [
            self._ax_ee_pos.plot([], [], color=_COLORS[i], lw=1.2, label=lbl)[0]
            for i, lbl in enumerate(["x", "y", "z"])
        ]
        if self.x_d is not None:
            for i, v in enumerate(self.x_d[:3]):
                self._ax_ee_pos.axhline(v, color=_COLORS[i], lw=0.6,
                                        ls="--", alpha=0.5)
        self._ax_ee_pos.legend(fontsize=7, facecolor="#333", labelcolor="white",
                               loc="upper right", framealpha=0.7)

        self.fig.add_subplot(gs[1, 1])
        self._ax_f_ext = self._style_ax("|f_ext|  contact force", ylabel="N")
        self._line_f_ext, = self._ax_f_ext.plot([], [], color="#ff8a65", lw=1.5)
        self._ax_f_ext.axhline(0, color="#555", lw=0.6)

        self.fig.add_subplot(gs[1, 2])
        self._ax_tau = self._style_ax("Joint torques", ylabel="Nm")
        self._lines_tau = [
            self._ax_tau.plot([], [], color=_COLORS[i % 7], lw=0.9,
                              label=_JOINT_LBL[i])[0]
            for i in range(7)
        ]
        self._ax_tau.legend(fontsize=6, facecolor="#333", labelcolor="white",
                            loc="upper right", ncol=4, framealpha=0.7)
        self._ax_tau.axhline(0, color="#555", lw=0.6)

        self.fig.add_subplot(gs[2, 1])
        self._ax_ee_vel = self._style_ax("|ẋ|  EE speed", ylabel="m/s")
        self._line_ee_vel, = self._ax_ee_vel.plot([], [], color="#a5d6a7", lw=1.5)
        self._ax_ee_vel.axhline(0, color="#555", lw=0.6)

        self.fig.add_subplot(gs[2, 2])
        self._build_mode_panel()

    def _style_ax(self, title: str, xlabel: str = "t (s)", ylabel: str = ""):
        plt = _get_plt()
        ax = plt.gca()
        ax.set_facecolor(_BG_PANEL)
        ax.set_title(title, color="white", fontsize=8.5, pad=3)
        ax.set_xlabel(xlabel, color=_TICK_CLR, fontsize=7.5)
        if ylabel:
            ax.set_ylabel(ylabel, color=_TICK_CLR, fontsize=7.5)
        ax.tick_params(colors=_TICK_CLR, labelsize=7)
        for spine in ax.spines.values():
            spine.set_edgecolor("#555")
        ax.grid(color=_GRID_CLR, linewidth=0.35, linestyle="--")
        return ax

    def _setup_3d_axes(self):
        ax = self._ax3d
        ax.set_facecolor(_BG_PANEL)
        ax.set_title("EE trajectory  (3D)", color="white", fontsize=9, pad=6)
        ax.set_xlabel("x (m)", color=_TICK_CLR, fontsize=7.5, labelpad=2)
        ax.set_ylabel("y (m)", color=_TICK_CLR, fontsize=7.5, labelpad=2)
        ax.set_zlabel("z (m)", color=_TICK_CLR, fontsize=7.5, labelpad=2)
        ax.tick_params(colors=_TICK_CLR, labelsize=6.5)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor("#444")
        ax.yaxis.pane.set_edgecolor("#444")
        ax.zaxis.pane.set_edgecolor("#444")
        ax.grid(True, color=_GRID_CLR, linewidth=0.3, linestyle="--")

        # trajectory lines: faded history + bright recent trail
        self._line3d_hist,  = ax.plot([], [], [], color="#4fc3f7",
                                      lw=0.8, alpha=0.35)
        self._line3d_trail, = ax.plot([], [], [], color="#4fc3f7",
                                      lw=2.0, alpha=1.0)
        # start marker (hollow circle)
        self._pt3d_start, = ax.plot([], [], [], "o",
                                    color="#aaaaaa", ms=6, mew=1.5,
                                    mfc="none", label="start")
        # current EE position (filled dot)
        self._pt3d_curr,  = ax.plot([], [], [], "o",
                                    color="#4fc3f7", ms=8,
                                    label="EE now")
        # target marker (yellow star)
        if self.x_d is not None:
            ax.plot([self.x_d[0]], [self.x_d[1]], [self.x_d[2]],
                    "*", color="#ffd54f", ms=12, label="target",
                    zorder=10)
            # vertical dashed line down to xy-plane for spatial reference
            ax.plot([self.x_d[0], self.x_d[0]],
                    [self.x_d[1], self.x_d[1]],
                    [0.0, self.x_d[2]],
                    "--", color="#ffd54f", lw=0.6, alpha=0.4)

        ax.legend(fontsize=7, facecolor="#2d2d3f", labelcolor="white",
                  loc="upper left", framealpha=0.7)

        # default view angle
        ax.view_init(elev=25, azim=-60)

    def _build_mode_panel(self):
        plt = _get_plt()
        if self.mode == "impedance":
            self._ax_mode = self._style_ax("Joint velocities  q̇", ylabel="rad/s")
            self._lines_dq = [
                self._ax_mode.plot([], [], color=_COLORS[i % 7], lw=0.9,
                                   label=_JOINT_LBL[i])[0]
                for i in range(7)
            ]
            self._ax_mode.legend(fontsize=6, facecolor="#333", labelcolor="white",
                                 loc="upper right", ncol=4, framealpha=0.7)
            self._ax_mode.axhline(0, color="#555", lw=0.6)

        elif self.mode == "phri":
            self._ax_mode = self._style_ax("CUSUM contact score  Sₖ", ylabel="Sₖ")
            self._line_cusum, = self._ax_mode.plot([], [], color="#ce93d8", lw=1.4)
            self._ax_mode.axhline(0, color="#555", lw=0.6)

        elif self.mode == "rl_random":
            self._ax_mode = self._style_ax("Impedance parameters")
            labels = ["K_d (N/m)", "D_d (N·s/m)", "k_n (Nm/rad)"]
            self._lines_params = [
                self._ax_mode.plot([], [], color=_COLORS[i], lw=1.2,
                                   label=labels[i])[0]
                for i in range(3)
            ]
            self._ax_mode.legend(fontsize=7, facecolor="#333", labelcolor="white",
                                 framealpha=0.7)
        else:
            self._ax_mode = self._style_ax("Joint velocities  q̇", ylabel="rad/s")
            self._lines_dq = [
                self._ax_mode.plot([], [], color=_COLORS[i % 7], lw=0.9,
                                   label=_JOINT_LBL[i])[0]
                for i in range(7)
            ]
            self._ax_mode.legend(fontsize=6, facecolor="#333", labelcolor="white",
                                 loc="upper right", ncol=4, framealpha=0.7)
            self._ax_mode.axhline(0, color="#555", lw=0.6)

    # ─────────────────────────────────────────────────────────────────────
    # Data ingestion
    # ─────────────────────────────────────────────────────────────────────

    def push(
        self,
        t:       float,
        pos_err: float             = 0.0,
        ee_pos:  np.ndarray | None = None,
        tau:     np.ndarray | None = None,
        dq:      np.ndarray | None = None,
        f_ext:   float             = 0.0,
        ee_vel:  float             = 0.0,
        contact: bool              = False,
        cusum:   float             = 0.0,
        params:  np.ndarray | None = None,
        reward:  float             = 0.0,
    ):
        """Append one data point to all buffers."""
        self._t.append(t)
        self._pos_err.append(pos_err)
        self._ee_pos.append(np.zeros(3) if ee_pos is None else np.asarray(ee_pos).copy())
        self._tau.append(np.zeros(7)    if tau    is None else tau[:7].copy())
        self._dq.append(np.zeros(7)     if dq     is None else dq[:7].copy())
        self._f_ext.append(f_ext)
        self._ee_vel.append(ee_vel)
        self._contact.append(float(contact))
        self._cusum.append(cusum)
        self._params.append(np.zeros(3) if params is None else np.asarray(params).copy())
        self._reward.append(reward)

    # ─────────────────────────────────────────────────────────────────────
    # Rendering
    # ─────────────────────────────────────────────────────────────────────

    def refresh(self):
        """Redraw all panels with current buffer data and yield to GUI loop."""
        plt = _get_plt()
        if not self._t:
            return

        t   = np.asarray(self._t)
        ee  = np.asarray(self._ee_pos)     # (N, 3)
        tau = np.asarray(self._tau)         # (N, 7)

        # ── 3-D trajectory ────────────────────────────────────────────────
        n = len(t)
        split = max(0, n - _TRAIL_RECENT)

        xs, ys, zs = ee[:, 0], ee[:, 1], ee[:, 2]

        # faded history
        self._line3d_hist.set_data_3d(xs[:split+1], ys[:split+1], zs[:split+1])
        # bright recent trail
        self._line3d_trail.set_data_3d(xs[split:], ys[split:], zs[split:])

        # start marker (first point only)
        self._pt3d_start.set_data_3d([xs[0]], [ys[0]], [zs[0]])

        # current EE dot
        self._pt3d_curr.set_data_3d([xs[-1]], [ys[-1]], [zs[-1]])

        # auto-scale 3D axes with a small margin
        margin = 0.05
        for i, (data, setter) in enumerate(
            [(xs, "set_xlim"), (ys, "set_ylim"), (zs, "set_zlim")]
        ):
            lo, hi = float(data.min()) - margin, float(data.max()) + margin
            if self.x_d is not None:
                lo = min(lo, float(self.x_d[i]) - margin)
                hi = max(hi, float(self.x_d[i]) + margin)
            getattr(self._ax3d, setter)(lo, hi)

        # ── time-series helpers ───────────────────────────────────────────
        def _upd(ax, line, ydata, margin=0.05):
            line.set_data(t, ydata)
            ax.set_xlim(t[0], max(t[-1], t[0] + 0.1))
            mn, mx = float(ydata.min()), float(ydata.max())
            sp = max(mx - mn, 1e-3)
            ax.set_ylim(mn - margin * sp, mx + margin * sp)

        _upd(self._ax_pos_err, self._line_pos_err, np.asarray(self._pos_err))
        _upd(self._ax_f_ext,   self._line_f_ext,   np.asarray(self._f_ext))
        _upd(self._ax_ee_vel,  self._line_ee_vel,  np.asarray(self._ee_vel))

        # EE x/y/z
        ymn, ymx = ee.min(), ee.max()
        sp = max(ymx - ymn, 0.01)
        for i, line in enumerate(self._lines_ee_pos):
            line.set_data(t, ee[:, i])
        self._ax_ee_pos.set_xlim(t[0], max(t[-1], 0.1))
        self._ax_ee_pos.set_ylim(ymn - 0.05 * sp, ymx + 0.05 * sp)

        # joint torques
        tmn, tmx = tau.min(), tau.max()
        tsp = max(tmx - tmn, 1.0)
        for i, line in enumerate(self._lines_tau):
            line.set_data(t, tau[:, i])
        self._ax_tau.set_xlim(t[0], max(t[-1], 0.1))
        self._ax_tau.set_ylim(tmn - 0.05 * tsp, tmx + 0.05 * tsp)

        # mode panel
        self._refresh_mode(t)

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def _refresh_mode(self, t: np.ndarray):
        if self.mode in ("impedance",) or self.mode not in ("phri", "rl_random"):
            dq  = np.asarray(self._dq)
            mn, mx = dq.min(), dq.max()
            sp = max(mx - mn, 0.01)
            for i, line in enumerate(self._lines_dq):
                line.set_data(t, dq[:, i])
            self._ax_mode.set_xlim(t[0], max(t[-1], 0.1))
            self._ax_mode.set_ylim(mn - 0.05 * sp, mx + 0.05 * sp)

        elif self.mode == "phri":
            cs = np.asarray(self._cusum)
            ct = np.asarray(self._contact)
            self._line_cusum.set_data(t, cs)
            # rebuild fill_between shading for contact periods
            for coll in self._ax_mode.collections:
                coll.remove()
            ceiling = max(float(cs.max()) * 1.1, 0.5)
            self._ax_mode.fill_between(t, 0, ceiling, where=ct > 0.5,
                                        color="#ff5252", alpha=0.20)
            self._ax_mode.set_xlim(t[0], max(t[-1], 0.1))
            self._ax_mode.set_ylim(0, ceiling * 1.1)

        elif self.mode == "rl_random":
            pr  = np.asarray(self._params)
            mn, mx = pr.min(), pr.max()
            sp = max(mx - mn, 1.0)
            for i, line in enumerate(self._lines_params):
                line.set_data(t, pr[:, i])
            self._ax_mode.set_xlim(t[0], max(t[-1], 0.1))
            self._ax_mode.set_ylim(mn - 0.05 * sp, mx + 0.05 * sp)

    # ─────────────────────────────────────────────────────────────────────
    # Save / close
    # ─────────────────────────────────────────────────────────────────────

    def save(self, path: str = "simulation_dashboard.png"):
        self.refresh()
        self.fig.savefig(path, dpi=150, bbox_inches="tight",
                         facecolor=self.fig.get_facecolor())
        print(f"[viz] Dashboard saved → {path}")

    def close(self):
        plt = _get_plt()
        plt.ioff()
        plt.close(self.fig)
