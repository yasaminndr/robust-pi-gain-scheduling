import numpy as np
from dataclasses import dataclass
from .models import FirstOrderPlant
from .controllers import PIController


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.05
    step_time: float = 10.0
    sp_final: float = 1.0

    # measurement filter
    alpha: float = 0.08
    noise_std: float = 0.0

    # simulation horizon
    t_end: float = 200.0


@dataclass(frozen=True)
class SimResult:
    t: np.ndarray
    sp: np.ndarray
    y: np.ndarray
    u: np.ndarray
    sat_ratio: float


def build_time_and_setpoint(cfg: SimConfig) -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(0.0, cfg.t_end + cfg.dt, cfg.dt)
    sp = np.ones_like(t) * cfg.sp_final
    sp[t < cfg.step_time] = 0.0
    return t, sp


def simulate_first_order(
    plant: FirstOrderPlant,
    tau: float,
    controller: PIController,
    cfg: SimConfig,
    seed: int = 0,
) -> SimResult:
    rng = np.random.default_rng(seed)
    t, sp = build_time_and_setpoint(cfg)

    y = np.zeros_like(t)
    y_meas = np.zeros_like(t)
    y_f = np.zeros_like(t)
    u = np.zeros_like(t)

    controller.reset()

    sat_count = 0
    n_steps = len(t) - 1

    for k in range(1, len(t)):
        # measurement (noise)
        y_meas[k - 1] = y[k - 1] + rng.normal(0.0, cfg.noise_std)

        # low-pass filter
        if k > 1:
            y_f[k - 1] = (1 - cfg.alpha) * y_f[k - 2] + cfg.alpha * y_meas[k - 1]
        else:
            y_f[k - 1] = y_meas[k - 1]

        e = sp[k - 1] - y_f[k - 1]

        u_sat, u_unsat, clipped = controller.step(e=e, dt=cfg.dt)
        u[k - 1] = u_sat
        if clipped:
            sat_count += 1

        # plant update (Euler)
        dy = (-(y[k - 1] - plant.amb) + plant.K * u_sat) / tau
        y[k] = y[k - 1] + dy * cfg.dt

    u[-1] = u[-2]
    sat_ratio = sat_count / max(1, n_steps)

    return SimResult(t=t, sp=sp, y=y, u=u, sat_ratio=sat_ratio)