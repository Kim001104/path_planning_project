"""Low-Pass Filter — 1차 IIR (Exponential Moving Average).

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class LowPassFilter:
    def __init__(self, alpha: float = 0.9):
        self.alpha = alpha
        self.y: float | None = None

    def step(self, x: float) -> float:
        if self.y is None:
            self.y = x
        else:
            self.y = self.alpha * self.y + (1 - self.alpha) * x

        return self.y

