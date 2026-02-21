import numpy as np
from src.models import FirstOrderPlant
from src.simulate import SimConfig, simulate_first_order, build_time_and_setpoint
from src.controllers import PIController
from src.metrics import compute_step_metrics
from src.plotting import ensure_results_dir, plot_outputs, plot_controls


def main():
    plant = FirstOrderPlant(K=1.0, amb=0.0)
    sim_cfg = SimConfig(t_end=200.0, noise_std=0.0)

    taus = [5.0, 15.0, 25.0, 60.0]

    # دستی: یک PI ثابت
    kp, ki = 1.2, 0.06

    t_list, y_list, u_list, labels = [], [], [], []
    for tau in taus:
        ctrl = PIController(kp=kp, ki=ki, u_min=0.0, u_max=1.5, kaw=1.0)
        res = simulate_first_order(plant, tau, ctrl, sim_cfg, seed=0)
        m = compute_step_metrics(res.t, res.y, sim_cfg.step_time, sim_cfg.sp_final)
        print(tau, m, "sat%", 100*res.sat_ratio)

        t_list.append(res.t); y_list.append(res.y); u_list.append(res.u); labels.append(f"tau={tau}")

    sp_ref_t, sp_ref = build_time_and_setpoint(sim_cfg)
    out_dir = ensure_results_dir("results")
    plot_outputs(t_list, y_list, sp_ref_t, sp_ref, labels, out_dir / "sweep_outputs.png")
    plot_controls(t_list, u_list, labels, out_dir / "sweep_controls.png")
    print("Saved to results/")


if __name__ == "__main__":
    main()
