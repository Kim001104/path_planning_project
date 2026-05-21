"""Moving Average Filter — sliding window arithmetic mean.

과제 명세는 README.md 참조.
"""
from __future__ import annotations

from collections import deque


class MovingAverageFilter:
    def __init__(self, window: int = 10):
        self.window = window
        self.buffer: deque[float] = deque(maxlen=window)

    def step(self, x: float) -> float:
        self.buffer.append(x)
        return sum(self.buffer) / len(self.buffer)

