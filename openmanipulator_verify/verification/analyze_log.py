#!/usr/bin/env python3
"""Analyze an OpenManipulator-X torque-verification CSV log into paper metrics."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_csv(path: Path) -> dict:
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    if d.shape == ():
        d = d.reshape(1)
    return {n: np.asarray(d[n]) for n in d.dtype.names}


def cols(d: dict, prefix: str, n: int) -> np.ndarray:
    return np.vstack([d[f"{prefix}_{i}"].astype(float) for i in range(n)]).T


def metrics(d: dict, ndof: int = 4) -> dict:
    t = d["t"].astype(float)
    err = d["err_mm"].astype(float)
    dur = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    tail = max(1, min(len(err), int(1.0 / max(np.median(np.diff(t)) if len(t) > 1 else 0.01, 1e-3))))
    comp = d["compute_ms"].astype(float)
    tau = cols(d, "tau", ndof)
    out = {
        "samples": int(len(err)),
        "duration_s": round(dur, 2),
        "rmse_mm": round(float(np.sqrt(np.mean(err**2))), 3),
        "max_error_mm": round(float(np.max(err)), 3),
        "steady_state_error_mm": round(float(np.mean(err[-tail:])), 3),
        "d_hat_peak": round(float(np.max(np.linalg.norm(cols(d, "d_hat", 3), axis=1))), 4),
        "nis_peak": round(float(np.max(d["nis"].astype(float))), 4),
        "compute_ms_mean": round(float(np.mean(comp)), 3),
        "compute_ms_p99": round(float(np.percentile(comp, 99)), 3),
        "compute_ms_max": round(float(np.max(comp)), 3),
        "tau_peak_Nm": [round(float(np.max(np.abs(tau[:, i]))), 3) for i in range(ndof)],
    }
    # recovery time: if error exceeds 2x its steady tail, time from the peak back
    # under 2 mm (only meaningful for a push/payload trial).
    thr = 2.0
    peak_i = int(np.argmax(err))
    if err[peak_i] > max(thr, 3.0 * out["steady_state_error_mm"]):
        after = np.where(err[peak_i:] < thr)[0]
        out["recovery_to_2mm_s"] = round(float(t[peak_i + after[0]] - t[peak_i]), 3) if len(after) else None
        out["peak_disturbance_error_mm"] = round(float(err[peak_i]), 3)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--json-out", type=Path, default=None)
    args = ap.parse_args()
    m = metrics(load_csv(args.csv))
    print(json.dumps(m, indent=2))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.json_out, "w") as f:
            json.dump(m, f, indent=2)


if __name__ == "__main__":
    main()
