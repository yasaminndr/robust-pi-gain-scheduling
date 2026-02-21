from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


def ensure_results_dir(path: str | Path) -> Path:
    p = Path(path)
    p.mkdir(parents=True, exist_ok=True)
    return p


def plot_outputs(t_list, y_list, sp_ref_t, sp_ref, labels, save_path: Path):
    plt.figure()
    for t, y, lab in zip(t_list, y_list, labels):
        plt.plot(t, y, label=lab)
    plt.plot(sp_ref_t, sp_ref, linestyle="--", label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Output y")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=180)
    plt.close()


def plot_controls(t_list, u_list, labels, save_path: Path):
    plt.figure()
    for t, u, lab in zip(t_list, u_list, labels):
        plt.plot(t, u, label=lab)
    plt.xlabel("Time (s)")
    plt.ylabel("Control u")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=180)
    plt.close()