import numpy as np
from src.models import FirstOrderPlant
from src.simulate import SimConfig, simulate_first_order, build_time_and_setpoint
from src.controllers import PIController
from src.metrics import compute_step_metrics
from src.optimize import OptConfig, optimize_schedule
from src.plotting import ensure_results_dir, plot_outputs, plot_controls


def main():
    plant = FirstOrderPlant(K=1.0, amb=0.0)

    # ---- choose a global time horizon (your choice) ----
    # اگر می‌خوای همیشه 200 باشه:
    sim_cfg = SimConfig(dt=0.05, step_time=10.0, sp_final=1.0, alpha=0.08, noise_std=0.0, t_end=200.0)

    # robust taus
    taus = [5.0, 15.0, 25.0, 60.0]

    # grid (coarse; then refine later)
    a_list = np.linspace(20.0, 220.0, 25)
    b_list = np.linspace(0.2, 1.2, 20)

    opt_cfg = OptConfig(
        taus=taus,
        a_list=a_list,
        b_list=b_list,
        w_settle=1.0,
        w_overshoot=25.0,
        w_sse=400.0,
        sat_soft_limit=0.10,
        sat_penalty_weight=1200.0,
        overshoot_limit=10.0,
    )

    # optimize
    result, top10 = optimize_schedule(
        plant=plant,
        sim_cfg=sim_cfg,
        opt_cfg=opt_cfg,
        u_min=0.0,
        u_max=1.5,
        kaw=1.0,
        seed=0,
    )

    print("\nTop 10 schedules (worst-case, mean, a, b):")
    for i, (Jw, Jm, a, b) in enumerate(top10, start=1):
        print(f"{i:2d}) Jw={Jw:10.2f} Jm={Jm:10.2f} a={a:8.2f} b={b:6.3f}")

    print("\nBEST schedule:")
    print(f"a={result.schedule.a:.4f}, b={result.schedule.b:.4f}")
    print(f"worst_cost={result.best_worst_cost:.2f}, mean_cost={result.best_mean_cost:.2f}")

    # evaluate + plot for each tau
    t_list, y_list, u_list, labels = [], [], [], []
    print("\n=== Per-tau evaluation with BEST schedule ===")
    print(f"{'tau':>6} | {'Kp':>7} | {'Ki':>8} | {'Sat%':>6} | {'Ov%':>7} | {'Rise':>7} | {'Settle':>7} | {'y_ss':>7} | {'SSE':>7}")
    print("-" * 86)

    for tau in taus:
        kp, ki = result.schedule.gains(tau)
        ctrl = PIController(kp=kp, ki=ki, u_min=0.0, u_max=1.5, kaw=1.0)

        res = simulate_first_order(plant=plant, tau=tau, controller=ctrl, cfg=sim_cfg, seed=0)
        m = compute_step_metrics(res.t, res.y, sim_cfg.step_time, sim_cfg.sp_final)

        print(f"{tau:6.1f} | {kp:7.3f} | {ki:8.4f} | {100*res.sat_ratio:6.1f} | {m.overshoot_pct:7.2f} | {m.rise_s:7.1f} | {m.settle_s:7.1f} | {m.y_ss:7.3f} | {m.sse:7.3f}")

        t_list.append(res.t)
        y_list.append(res.y)
        u_list.append(res.u)
        labels.append(f"tau={tau}")

    # setpoint reference for plotting
    sp_ref_t, sp_ref = build_time_and_setpoint(sim_cfg)

    out_dir = ensure_results_dir("results")
    plot_outputs(t_list, y_list, sp_ref_t, sp_ref, labels, out_dir / "outputs.png")
    plot_controls(t_list, u_list, labels, out_dir / "controls.png")

    print("\nSaved figures:")
    print(" - results/outputs.png")
    print(" - results/controls.png")


if __name__ == "__main__":
    main()