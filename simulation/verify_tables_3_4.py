import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fair_offset_free_comparison import run, Baseline

# Table III remaining rows: Observer, C6, C7
for label, spec in [
    ("Observer 100Hz", "DI-MPC + Kalman + Observer 100 Hz"),
    ("C6: DI-MPC 500Hz", "DI-MPC 500 Hz"),
    ("C7: DI-MPC+Kalman 500Hz", "DI-MPC + Kalman 500 Hz"),
]:
    r = run(spec, 3)
    m = r["metrics"]
    print(f"{label:28s} contact={1e3*m['rms_contact']:7.3f} mm peak={1e3*m['peak_defl']:7.3f} mm ss={1e3*m['ss_err']:7.3f} mm")

# Table IV broad-screen baselines using phri.py's own controller-name dispatch
for label, spec in [
    ("Admittance 100N/m", "Admittance"),
    ("PI impedance", "PI Impedance"),
    ("Predictive var. imp.", "Variable-Impedance MPC 100 Hz"),
]:
    r = run(spec, 3)
    m = r["metrics"]
    print(f"{label:28s} total={1e3*m['rms_total']:7.3f} mm contact={1e3*m['rms_contact']:7.3f} mm peak={1e3*m['peak_defl']:7.3f} mm ss={1e3*m['ss_err']:7.3f} mm")
