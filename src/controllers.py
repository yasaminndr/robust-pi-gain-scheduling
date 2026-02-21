from dataclasses import dataclass


@dataclass
class PIController:
    """PI with saturation + anti-windup (back-calculation)."""
    kp: float
    ki: float
    u_min: float = 0.0
    u_max: float = 1.5
    kaw: float = 1.0

    integ: float = 0.0

    def reset(self) -> None:
        self.integ = 0.0

    def step(self, e: float, dt: float) -> tuple[float, float, bool]:
        """
        Returns:
            u_sat: saturated control
            u_unsat: raw control
            clipped: True if saturation happened
        """
        u_unsat = self.kp * e + self.ki * self.integ
        u_sat = min(max(u_unsat, self.u_min), self.u_max)
        clipped = (u_sat != u_unsat)

        # anti-windup back-calculation
        self.integ += (e + self.kaw * (u_sat - u_unsat)) * dt

        return u_sat, u_unsat, clipped


@dataclass(frozen=True)
class GainSchedule:
    """
    Gain scheduling law:
      Kp(tau) = a / tau
      Ti(tau) = b * tau
      Ki(tau) = Kp / Ti
    """
    a: float
    b: float

    def gains(self, tau: float) -> tuple[float, float]:
        kp = self.a / tau
        ti = self.b * tau
        ki = kp / ti
        return kp, ki