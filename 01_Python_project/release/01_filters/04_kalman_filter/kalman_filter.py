"""Kalman Filter — scalar 1D linear system.

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class KalmanFilter:
    def __init__(self, m: float = 1.0, dt: float = 0.01,
                 q: float = 0.1, r: float = 0.9, p0: float = 10.0):
        self.A = 1.0 + dt
        self.B = dt / m
        self.C = 1.0
        self.Q = q
        self.R = r
        self.x = 0.0
        self.P = p0

    def step(self, measurement: float, control_input: float) -> float:
        x_pred = self.A * self.x + self.B * control_input
        p_pred = self.A ** 2 * self.P + self.Q

        k = p_pred * self.C / (self.C ** 2 * p_pred + self.R)

        self.x = x_pred + k * (measurement - self.C * x_pred)
        self.P = (1 - k * self.C) * p_pred

        return self.x

