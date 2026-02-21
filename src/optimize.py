import numpy as np
from dataclasses import dataclass
from .models import FirstOrderPlant
from .controllers import GainSchedule, PIController
from .simulate import SimConfig, simulate_first_order
from .metrics import compute_step_metrics


@dataclass(frozen=True)
class OptConfig:
    taus: list[float]
    a_list: np.ndarray
    b_list: np.ndarray

    # cost weights
    w_settle: float = 1.0
    w_overshoot: float = 25.0
    w_sse: float = 400.0

    # saturation penalty
    sat_soft_limit: float = 0.10
    sat_penalty_weight: float = 800.0

    # hard constraints
    overshoot_limit: float = 10.0


@dataclass(frozen=True)
class OptResult:
    schedule: GainSchedule
    best_worst_cost: float
    best_mean_cost: float


def _cost(
    settle_s: float,
    overshoot_pct: float,
    sse: float,
    sat_ratio: float,
    cfg: OptConfig,
) -> float:
    if not np.isfinite(settle_s):
        return 1e9
    if overshoot_pct > cfg.overshoot_limit:
        return 1e6 + 1e4 * overshoot_pct

    J = cfg.w_settle * settle_s + cfg.w_overshoot * overshoot_pct + cfg.w_sse * sse

    excess = max(0.0, sat_ratio - cfg.sat_soft_limit)
    J += cfg.sat_penalty_weight * (excess ** 2)
    return float(J)


def optimize_schedule(
    plant: FirstOrderPlant,
    sim_cfg: SimConfig,
    opt_cfg: OptConfig,
    u_min: float = 0.0,
    u_max: float = 1.5,
    kaw: float = 1.0,
    seed: int = 0,
) -> tuple[OptResult, list[tuple[float, float, float, float]]]:
    """
    Returns:
      OptResult
      top10 list of tuples: (J_worst, J_mean, a, b)
    """
    candidates: list[tuple[float, float, float, float]] = []

    for a in opt_cfg.a_list:
        for b in opt_cfg.b_list:
            schedule = GainSchedule(a=float(a), b=float(b))
            J_list = []

            for tau in opt_cfg.taus:
                kp, ki = schedule.gains(tau)
                ctrl = PIController(kp=kp, ki=ki, u_min=u_min, u_max=u_max, kaw=kaw)

                res = simulate_first_order(
                    plant=plant, tau=float(tau), controller=ctrl, cfg=sim_cfg, seed=seed
                )
                m = compute_step_metrics(res.t, res.y, sim_cfg.step_time, sim_cfg.sp_final)

                J = _cost(
                    settle_s=m.settle_s,
                    overshoot_pct=m.overshoot_pct,
                    sse=m.sse,
                    sat_ratio=res.sat_ratio,
                    cfg=opt_cfg,
                )
                J_list.append(J)

            J_worst = float(np.max(J_list))
            J_mean = float(np.mean(J_list))
            candidates.append((J_worst, J_mean, float(a), float(b)))

    candidates.sort(key=lambda x: (x[0], x[1]))
    best = candidates[0]
    best_schedule = GainSchedule(a=best[2], b=best[3])
    result = OptResult(schedule=best_schedule, best_worst_cost=best[0], best_mean_cost=best[1])
    return result, candidates[:10]