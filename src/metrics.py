import numpy as np
from dataclasses import dataclass


@dataclass(frozen=True)
class StepMetrics:
    overshoot_pct: float
    rise_s: float
    settle_s: float
    y_ss: float
    sse: float


def compute_step_metrics(
    t: np.ndarray,
    y: np.ndarray,
    step_time: float,
    sp_final: float,
    settle_band: float = 0.02,
) -> StepMetrics:
    idx0 = int(np.searchsorted(t, step_time))
    t2 = t[idx0:]
    y2 = y[idx0:]

    # steady-state estimate (last 10%)
    tail = max(10, int(0.1 * len(y2)))
    y_ss = float(np.mean(y2[-tail:]))
    sse = float(abs(sp_final - y_ss))

    # overshoot
    y_peak = float(np.max(y2))
    overshoot = max(0.0, (y_peak - sp_final) / abs(sp_final) * 100.0)

    # rise time 10->90
    y10, y90 = 0.1 * sp_final, 0.9 * sp_final

    def first_cross(level: float) -> float:
        idx = np.where(y2 >= level)[0]
        return float(t2[idx[0]]) if len(idx) else float("nan")

    t10 = first_cross(y10)
    t90 = first_cross(y90)
    rise = float(t90 - t10) if (np.isfinite(t10) and np.isfinite(t90)) else float("nan")

    # settling time: first time after which stays within ±settle_band
    band = settle_band * abs(sp_final)
    within = np.abs(y2 - sp_final) <= band
    settle = float("nan")
    for i in range(len(within)):
        if np.all(within[i:]):
            settle = float(t2[i])
            break

    return StepMetrics(
        overshoot_pct=float(overshoot),
        rise_s=float(rise),
        settle_s=float(settle),
        y_ss=float(y_ss),
        sse=float(sse),
    )