"""ConstantSpacePID — 일정 간격 유지 종방향 PID.

과제 명세는 README.md 참조.
"""
from __future__ import annotations


class ConstantSpacePID:
    def __init__(self, kp: float, kd: float, ki: float, dt: float,
                 target_space: float = 20.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.target_space = target_space
        self.prev_error: float | None = None
        self.error_sum: float = 0.0

    def step(self, target_x: float, ego_x: float) -> float:
        error = self.target_space - (target_x - ego_x)  # 반대 부호라서 문제

        if self.prev_error is None:
            d_error = 0.0
        else:
            d_error = (error - self.prev_error) / self.dt

        self.error_sum += error * self.dt

        u = self.kp * error + self.kd * d_error + self.ki * self.error_sum

        self.prev_error = error

        return u
