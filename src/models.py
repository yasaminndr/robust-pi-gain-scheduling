from dataclasses import dataclass


@dataclass(frozen=True)
class FirstOrderPlant:
    """First-order plant: dy/dt = (-(y - amb) + K*u) / tau"""
    K: float = 1.0
    amb: float = 0.0