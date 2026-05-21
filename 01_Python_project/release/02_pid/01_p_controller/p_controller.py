"""P Controller — proportional feedback (memoryless).

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class PController:
    def __init__(self, kp: float):
        self.kp = kp

    def step(self, reference: float, measure: float) -> float:
        error = reference - measure
        return self.kp * error
