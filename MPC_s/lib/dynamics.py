"""Rigid-body dynamics (gravity G(q) and joint-space mass matrix M(q)) for the
3-DOF arm derived from OpenManipulator-X after the waist (original joint1, yaw
about z) was physically removed. Joint order matches kinematics.py:
[j1, j2, j3] = original [joint2, joint3, joint4].

The removed waist joint rotated about z, which is parallel to gravity, so the
recursive base linear-acceleration trick (a[0] = -g) is unaffected by folding
it out -- only the fixed base->joint1 translation (D[0]) changes, becoming the
combined original d01+d12 offset. M(q) and G(q) below are otherwise the same
RNEA formulation as the original 4-DOF model, just with n=3.

G(q)/C(q,dq)dq are RNEA at zero/actual velocity (gravity on/off respectively);
`bias_force()` gets both in a single RNEA pass. M(q) is NOT built via n
separate RNEA calls (the original approach, kept below as `_mass_matrix_rnea`
for regression testing only) -- it uses the standard closed-form Jacobian
formula instead (one forward pass instead of n RNEA passes), verified to
match `_mass_matrix_rnea` to machine precision (see `test_local.py`). This
was the dominant cost of the per-tick control loop: each RNEA call costs
about the same regardless of what it's computing, and the old mass_matrix()
alone issued 3 of them (plus one each for gravity() and coriolis()) -- 5 RNEA
calls/tick in total. This file now issues at most 2 (one bias_force() pass
plus the Jacobian-formula mass matrix, which is not RNEA at all).

Optional JIT (`use_jit=True`, see `OpenManipulatorDynamics.__init__`):
_rnea() is Python/numpy-overhead-bound, not FLOP-bound (these are 3x3
matrices) -- measured at ~395us/call in pure numpy vs ~5us/call Numba-JIT-
compiled (~79x, see implementation_fix.md's "would C++ help" section, which
is where this number first came from). Requires `pip install numba`
(deliberately NOT a hard dependency of this numpy-only project -- see
requirements.txt); raises a clear error if `use_jit=True` and numba isn't
installed, exactly like DynamixelCurrentBackend does for dynamixel-sdk.
Call `warmup()` once before a real-time loop starts -- the FIRST call to a
Numba function pays its one-time compile cost (can be ~0.1-1s), which must
not happen on the loop's first real tick.
"""
from __future__ import annotations

import numpy as np

try:
    from numba import njit as _njit
    _NUMBA_AVAILABLE = True
except ImportError:
    _NUMBA_AVAILABLE = False

G_VEC = np.array([0.0, 0.0, -9.81])
AXES = [np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])]
# joint-frame offsets (parent frame -> joint i origin), matching kinematics.py.
# D[0] is the fixed base->joint1 offset (folded original d01+d12); VERIFY this
# against the actual bracket dimensions on hardware.
D = [np.array([0.012, 0.0, 0.0595]), np.array([0.024, 0.0, 0.128]), np.array([0.124, 0.0, 0.0])]
# link inertial params in each link frame (mass [kg], COM [m], inertia_com [kg m^2]).
# Dropped the original link1 (removed waist link) entry.
LINKS = [
    dict(mass=0.13851, com=[0.01031, 0.00038, 0.1017],
         I=[[3.306e-04, -1e-07, -3.85e-05], [-1e-07, 3.429e-04, -1.6e-06], [-3.85e-05, -1.6e-06, 6.03e-05]]),
    dict(mass=0.13275, com=[0.09091, 0.00039, 0.00022],
         I=[[3.07e-05, -1.3e-06, -3e-07], [-1.3e-06, 2.423e-04, 0.0], [-3e-07, 0.0, 2.516e-04]]),
    dict(mass=0.14328, com=[0.04421, 0.0, 0.00891],
         I=[[8.09e-05, 0.0, -1e-06], [0.0, 7.6e-05, 0.0], [-1e-06, 0.0, 9.31e-05]]),
]


def _rot(axis: np.ndarray, a: float) -> np.ndarray:
    x, y, z = axis
    c, s, C = np.cos(a), np.sin(a), 1 - np.cos(a)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


if _NUMBA_AVAILABLE:
    @_njit(cache=True, fastmath=True)
    def _rnea_jit(q, dq, ddq, gravity_scale, axes, D, m, c, I, g_vec):
        """Numba-compiled restatement of `OpenManipulatorDynamics._rnea()`,
        identical step for step, on plain arrays instead of `self`/Python
        lists (njit cannot compile a bound method touching those). The
        rotation-matrix construction is inlined rather than calling out to
        the module's own `_rot()`, since a njit function can only call other
        njit'd functions, not an arbitrary Python one -- verified bit-for-bit
        (~1e-14 max abs diff, floating-point noise) against `_rot()`/`_rnea()`
        across random (q, dq, ddq, gravity_scale) before this was adopted."""
        n = 3
        R = np.empty((n, 3, 3))
        for i in range(n):
            x, y, z = axes[i]
            a_ = q[i]
            ca = np.cos(a_); sa = np.sin(a_); C = 1.0 - ca
            R[i, 0, 0] = ca + x * x * C;   R[i, 0, 1] = x * y * C - z * sa; R[i, 0, 2] = x * z * C + y * sa
            R[i, 1, 0] = y * x * C + z * sa; R[i, 1, 1] = ca + y * y * C;   R[i, 1, 2] = y * z * C - x * sa
            R[i, 2, 0] = z * x * C - y * sa; R[i, 2, 1] = z * y * C + x * sa; R[i, 2, 2] = ca + z * z * C
        w = np.zeros((n + 1, 3)); wd = np.zeros((n + 1, 3)); a = np.zeros((n + 1, 3))
        a[0] = -g_vec * gravity_scale
        F = np.zeros((n, 3)); N = np.zeros((n, 3))
        for i in range(n):
            iR = R[i].T
            ax = axes[i]
            w[i + 1] = iR @ w[i] + dq[i] * ax
            wd[i + 1] = iR @ wd[i] + np.cross(iR @ w[i], dq[i] * ax) + ddq[i] * ax
            a[i + 1] = iR @ (a[i] + np.cross(wd[i], D[i]) + np.cross(w[i], np.cross(w[i], D[i])))
            ci = c[i]
            a_ci = a[i + 1] + np.cross(wd[i + 1], ci) + np.cross(w[i + 1], np.cross(w[i + 1], ci))
            F[i] = m[i] * a_ci
            N[i] = I[i] @ wd[i + 1] + np.cross(w[i + 1], I[i] @ w[i + 1])
        f = np.zeros(3); nn = np.zeros(3); tau = np.zeros(n)
        for i in range(n - 1, -1, -1):
            if i + 1 < n:
                Rc = R[i + 1]
                p = D[i + 1]
                f_child = f.copy(); n_child = nn.copy()
                f = Rc @ f_child + F[i]
                nn = N[i] + Rc @ n_child + np.cross(c[i], F[i]) + np.cross(p, Rc @ f_child)
            else:
                f = F[i]
                nn = N[i] + np.cross(c[i], F[i])
            tau[i] = nn @ axes[i]
        return tau


class OpenManipulatorDynamics:
    """3-DOF (all pitch) RNEA dynamics for the waist-removed OpenManipulator-X."""

    def __init__(self, use_jit: bool = False):
        """use_jit=True routes _rnea() through the Numba-compiled
        `_rnea_jit()` above instead of this class's own pure-Python/numpy
        loop -- see this module's docstring for the ~79x measured speedup
        and why it exists as an opt-in, not the default. Raises immediately
        (not on first use) if numba isn't installed, matching how
        DynamixelCurrentBackend fails fast on a missing dynamixel-sdk."""
        if use_jit and not _NUMBA_AVAILABLE:
            raise RuntimeError(
                "use_jit=True requires numba, which is not installed. "
                "pip install numba (see requirements.txt's comment on it)."
            )
        self.use_jit = bool(use_jit)
        self._warmed_up = False
        self.n = 3
        self.m = [L["mass"] for L in LINKS]
        self.c = [np.asarray(L["com"], dtype=float) for L in LINKS]
        self.I = [np.asarray(L["I"], dtype=float) for L in LINKS]
        # Numba needs plain contiguous arrays, not Python lists of arrays --
        # precomputed once here rather than rebuilt on every _rnea() call.
        self._axes_arr = np.array(AXES)
        self._D_arr = np.array(D)
        self._m_arr = np.array(self.m)
        self._c_arr = np.array(self.c)
        self._I_arr = np.array(self.I)

    def warmup(self) -> float:
        """Trigger Numba's one-time JIT compile now, not on the control
        loop's first real tick (that first call can take ~0.1-1s -- see
        this module's docstring). No-op (returns 0.0) if use_jit is False.
        Call this once, right after construction, before any real-time loop
        starts. Returns the wall-clock seconds this took."""
        if not self.use_jit or self._warmed_up:
            return 0.0
        import time
        t0 = time.perf_counter()
        self._rnea(np.zeros(self.n), np.zeros(self.n), np.zeros(self.n), gravity=True)
        self._warmed_up = True
        return time.perf_counter() - t0

    def _rnea(self, q, dq, ddq, gravity: bool = False, gravity_scale: float = 1.0) -> np.ndarray:
        """`gravity` (bool) keeps the original on/off call sites unchanged;
        `gravity_scale` generalizes the same "a[0]=-g" trick to a scaled
        gravity vector so gravity and Coriolis/centrifugal torques can be
        obtained together in ONE pass (RNEA's forward recursion is linear in
        a[0], so this is exact, not an approximation -- verified in
        test_local.py against calling gravity()/coriolis() separately)."""
        gs = gravity_scale if gravity else 0.0
        if self.use_jit:
            return _rnea_jit(np.asarray(q, dtype=float), np.asarray(dq, dtype=float),
                              np.asarray(ddq, dtype=float), gs, self._axes_arr, self._D_arr,
                              self._m_arr, self._c_arr, self._I_arr, G_VEC)
        n = self.n
        R = [_rot(AXES[i], q[i]) for i in range(n)]      # {}^{i-1}R_i
        w = [np.zeros(3) for _ in range(n + 1)]
        wd = [np.zeros(3) for _ in range(n + 1)]
        a = [np.zeros(3) for _ in range(n + 1)]
        a[0] = -G_VEC * gravity_scale if gravity else np.zeros(3)  # base linear accel (gravity trick)
        F = [np.zeros(3) for _ in range(n)]
        N = [np.zeros(3) for _ in range(n)]
        # forward
        for i in range(n):
            iR = R[i].T                                   # {}^iR_{i-1}
            ax = AXES[i]
            w[i + 1] = iR @ w[i] + dq[i] * ax
            wd[i + 1] = iR @ wd[i] + np.cross(iR @ w[i], dq[i] * ax) + ddq[i] * ax
            a[i + 1] = iR @ (a[i] + np.cross(wd[i], D[i]) + np.cross(w[i], np.cross(w[i], D[i])))
            ci = self.c[i]
            a_ci = a[i + 1] + np.cross(wd[i + 1], ci) + np.cross(w[i + 1], np.cross(w[i + 1], ci))
            F[i] = self.m[i] * a_ci
            N[i] = self.I[i] @ wd[i + 1] + np.cross(w[i + 1], self.I[i] @ w[i + 1])
        # backward
        f = np.zeros(3); nn = np.zeros(3); tau = np.zeros(n)
        for i in range(n - 1, -1, -1):
            if i + 1 < n:
                Rc = R[i + 1]                             # {}^iR_{i+1}
                p = D[i + 1]
                f_child = f.copy(); n_child = nn.copy()
                f = Rc @ f_child + F[i]
                nn = N[i] + Rc @ n_child + np.cross(self.c[i], F[i]) + np.cross(p, Rc @ f_child)
            else:
                f = F[i]
                nn = N[i] + np.cross(self.c[i], F[i])
            tau[i] = nn @ AXES[i]
        return tau

    def gravity(self, q) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(self.n)
        return self._rnea(q, np.zeros(self.n), np.zeros(self.n), gravity=True)

    def coriolis(self, q, dq) -> np.ndarray:
        """C(q,dq)dq [Nm]: RNEA at the actual joint velocity, zero acceleration,
        gravity off -- same engine as gravity()/mass_matrix(), just with dq
        plugged in instead of zero. run_hardware.py's feedforward torque
        cancels this term explicitly, as tau_ff = C(q,dq)dq + G(q) + Jv^T pdd_d.
        Without it, C(q,dq)dq would be left inside the disturbance estimate
        d_hat for the observer to absorb."""
        q = np.asarray(q, dtype=float).reshape(self.n)
        dq = np.asarray(dq, dtype=float).reshape(self.n)
        return self._rnea(q, dq, np.zeros(self.n), gravity=False)

    def bias_force(self, q, dq, gravity_scale: float = 1.0) -> np.ndarray:
        """gravity_scale*G(q) + C(q,dq)dq [Nm] in a SINGLE RNEA pass -- the
        combined feedforward term every caller in this project actually
        wants (they never use gravity()/coriolis() separately), instead of
        the 2 RNEA passes `gravity_scale*gravity(q) + coriolis(q,dq)` costs.
        Exact, not approximate: verified in test_local.py."""
        q = np.asarray(q, dtype=float).reshape(self.n)
        dq = np.asarray(dq, dtype=float).reshape(self.n)
        return self._rnea(q, dq, np.zeros(self.n), gravity=True, gravity_scale=gravity_scale)

    def mass_matrix(self, q) -> np.ndarray:
        """Joint-space mass matrix via the standard closed-form Jacobian
        formula (Siciliano et al., "Robotics", eq. 7.31-ish): ONE forward
        kinematics pass builds each link's world-frame COM position, COM
        Jacobian and orientation; M = sum_k m_k Jv_k^T Jv_k + Jw_k^T I_k Jw_k.
        This replaces the original n-separate-RNEA-calls approach (kept
        below as `_mass_matrix_rnea` for the regression check in
        test_local.py that these two stay numerically identical) -- it was
        the single largest cost in the per-tick control loop."""
        q = np.asarray(q, dtype=float).reshape(self.n)
        n = self.n
        R_before = [np.eye(3)]                    # world orientation BEFORE joint i's own rotation
        for i in range(n):
            R_before.append(R_before[-1] @ _rot(AXES[i], q[i]))
        p = [D[0].copy()]                          # joint-pivot origins in world (fixed D[0] first)
        for i in range(1, n):
            p.append(p[-1] + R_before[i] @ D[i])
        axis_world = [R_before[i] @ AXES[i] for i in range(n)]

        M = np.zeros((n, n))
        for k in range(n):
            Rk = R_before[k + 1]                   # link k's own world orientation
            p_ck = p[k] + Rk @ self.c[k]            # link k COM in world
            Jv = np.zeros((3, n)); Jw = np.zeros((3, n))
            for j in range(k + 1):
                Jv[:, j] = np.cross(axis_world[j], p_ck - p[j])
                Jw[:, j] = axis_world[j]
            Ik_world = Rk @ self.I[k] @ Rk.T
            M += self.m[k] * (Jv.T @ Jv) + Jw.T @ Ik_world @ Jw
        return 0.5 * (M + M.T)

    def _mass_matrix_rnea(self, q) -> np.ndarray:
        """Original approach (n separate RNEA calls, unit accelerations,
        gravity off) -- kept only so test_local.py can regression-check that
        the fast mass_matrix() above stays numerically identical to it."""
        q = np.asarray(q, dtype=float).reshape(self.n)
        M = np.zeros((self.n, self.n))
        for j in range(self.n):
            e = np.zeros(self.n); e[j] = 1.0
            M[:, j] = self._rnea(q, np.zeros(self.n), e, gravity=False)
        return 0.5 * (M + M.T)

    def task_inertia(self, q, J) -> np.ndarray:
        M = self.mass_matrix(q)
        Minv = np.linalg.inv(M)
        return np.linalg.inv(J @ Minv @ J.T + 1e-6 * np.eye(3))
