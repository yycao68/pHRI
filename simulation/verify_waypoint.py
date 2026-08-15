import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from guidance import run_episode, MPC_NAMES
from fr3_mujoco import FR3MuJoCoEnv

env = FR3MuJoCoEnv()
for name in ["DI-MPC 100Hz", "DI-MPC + Kalman 100Hz", "DI-MPC 500Hz", "DI-MPC + Kalman 500Hz"]:
    r = run_episode(name, env, viewer=None, verbose=False)
    print(f"{name:26s} rms_free={1e3*r['rms_free']:7.3f} mm  "
          f"rms_contact={1e3*r['rms_contact']:7.3f} mm  peak={1e3*r['peak_defl']:7.3f} mm")
