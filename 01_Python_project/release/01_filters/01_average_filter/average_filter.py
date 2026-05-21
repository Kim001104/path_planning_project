"""Average Filter — recursive cumulative mean.

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class AverageFilter:
    def __init__(self):
        self.n = 0
        self.avg = 0.0

    def step(self, x: float) -> float:
        self.n += 1
        self.avg = self.avg + (x - self.avg) / self.n
        return self.avg

