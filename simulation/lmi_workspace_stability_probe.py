"""
Workspace-stability LMI probe (recovered) — Theorem 3 / Remark 5.

Samples the operational-space inverse-inertia Lambda^-1(q) = J_v M^-1 J_v^T + 1e-6 I
along the circular pHRI benchmark, builds the 64-vertex entry-wise box over the
6 symmetric entries, and solves the minimum-condition-number vertex quadratic-stabilizability LMI

    min cond(P)   over   Q = Q^T > 0, Y :   for every vertex L_v,
        [[ gamma*Q,           (A_d Q + B_v Y)^T ],
         [ (A_d Q + B_v Y),    Q                 ]]  >= 0,   gamma = rho_target^2,

with A_d the constant nilpotent double-integrator transition and B_v the vertex
input matrix. A common (parameter-independent) P = Q^-1 certifies quadratic
stability across the whole polytope (Theorem 3) at guaranteed rate <= rho_target.
This objective is fully reproducible (fixed rho_target = 0.996, CLARABEL solver).

Runs the probe for BOTH the Forward-Euler and the exact-ZOH input matrix to show
the O(dt^2) top block leaves the certificate unchanged (Remark 5).

Usage:  python3 lmi_workspace_stability_probe.py
Requires cvxpy (SDP).  dt = 10 ms as in Remark 5.
"""
import sys, os, itertools
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
DT = 0.01                       # 100 Hz, as Remark 5
SIGMA = 1e-6                    # Lambda^-1 regularization (matches impedance_mpc.py)
IDX = [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)]   # 6 symmetric entries


def sample_lambda_inv():
    """Run the proposed controller over the circular benchmark, log Lambda^-1."""
    import mujoco  # noqa: F401
    from fr3_mujoco import FR3MuJoCoEnv
    from phri import (EpisodeController, circular_ref, human_wrench,
                      PERIOD, MPC_DT_SLOW)
    env = FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    R_d = np.eye(3)
    ctrl = EpisodeController("Double-Integrator MPC + Kalman 100 Hz",
                             env, dt_mpc=MPC_DT_SLOW)
    S = []
    for i in range(int(3 * PERIOD / env.dt)):
        t = env.time
        p_d, dp_d, ddp_d = circular_ref(t)
        w = human_wrench(t)
        dyn, state = env.get_dynamics_and_state()
        if np.any(w[:3] != 0):
            env.apply_ee_wrench(w)
        tau, _ = ctrl.compute(state, dyn, p_d, dp_d, ddp_d, R_d, w, i)
        env.apply_torque(tau)
        env.step()
        if i % 5 == 0 and t > 0.5:                      # skip startup transient
            Jv = dyn.J[:3]
            S.append(Jv @ np.linalg.inv(dyn.M) @ Jv.T + SIGMA * np.eye(3))
    return np.array(S)


def build_box(S):
    lo = np.array([S[:, a, b].min() for a, b in IDX])
    hi = np.array([S[:, a, b].max() for a, b in IDX])

    def mat(vec):
        M = np.zeros((3, 3))
        for (a, b), v in zip(IDX, vec):
            M[a, b] = v
            M[b, a] = v
        return M

    verts = [mat(np.where(m, hi, lo)) for m in itertools.product([0, 1], repeat=6)]
    return [V for V in verts if np.all(np.linalg.eigvalsh(V) > 1e-6)]


def solve_mincond(verts, B_of_L, rho_target=0.996):
    """Minimum-condition-number common-P certificate at a guaranteed decay rate.

    Solves the convex program

        min t   s.t.   I <= Q <= t I,   and for every vertex L_v,
            [[ gamma*Q,          (A_d Q + B_v Y)^T ],
             [ (A_d Q + B_v Y),   Q                ]]  >= 0,   gamma = rho_target^2.

    The gamma-contractive LMI forces the closed loop A_cl = A_d + B_v (Y Q^-1) to
    contract at rate <= rho_target across the whole polytope; among all such
    certificates, minimizing t = cond(Q) = cond(P) returns the best-conditioned
    common P = Q^-1.  Returns (cond P, actual spectral radius, Lyapunov increment).
    Fully reproducible: fixed objective, fixed rho_target, CLARABEL solver.
    """
    import cvxpy as cp
    I3 = np.eye(3)
    A_d = np.block([[I3, DT * I3], [np.zeros((3, 3)), I3]])
    gamma = rho_target ** 2
    Q = cp.Variable((6, 6), symmetric=True)
    Y = cp.Variable((3, 6))
    t = cp.Variable()
    cons = [Q >> np.eye(6), Q << t * np.eye(6)]
    for L in verts:
        AB = A_d @ Q + B_of_L(L) @ Y
        cons.append(cp.bmat([[gamma * Q, AB.T], [AB, Q]]) >> 1e-6 * np.eye(12))
    cp.Problem(cp.Minimize(t), cons).solve(solver=cp.CLARABEL)
    Qv = Q.value
    P = np.linalg.inv(Qv)
    P = 0.5 * (P + P.T)
    K = Y.value @ np.linalg.inv(Qv)
    rho = max(max(abs(np.linalg.eigvals(A_d + B_of_L(L) @ K))) for L in verts)
    incr = max(max(np.linalg.eigvalsh((A_d + B_of_L(L) @ K).T @ P @ (A_d + B_of_L(L) @ K) - P))
               for L in verts)
    return np.linalg.cond(P), rho, incr


if __name__ == "__main__":
    S = sample_lambda_inv()
    eig = np.concatenate([np.linalg.eigvalsh(L) for L in S])
    print(f"{len(S)} samples   eig(Lambda^-1) in [{eig.min():.3f}, {eig.max():.3f}] kg^-1")
    verts = build_box(S)
    print(f"{len(verts)}-vertex entry-wise box   (dt = {DT*1e3:.0f} ms)\n")
    B_euler = lambda L: np.vstack([np.zeros((3, 3)), -L * DT])
    B_zoh = lambda L: np.vstack([-0.5 * DT * DT * L, -L * DT])
    for name, B in [("Forward-Euler B_v", B_euler), ("Exact-ZOH   B_v", B_zoh)]:
        c, r, d = solve_mincond(verts, B, rho_target=0.996)
        print(f"{name:18s}  cond(P)={c:7.2f}   spectral_radius<={r:.4f}   "
              f"maxeig(Acl'PAcl-P)={d:+.2e}")
