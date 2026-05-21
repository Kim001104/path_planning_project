"""Frame Transform — Global → Local + 다항식 fit/evaluate.

과제 명세는 README.md 참조.
"""
from __future__ import annotations

import numpy as np


class Global2Local:
    def __init__(self, num_points: int):
        self.n = num_points
        self.local_points = np.zeros((num_points, 2))

    def convert(self, points: np.ndarray, yaw_ego: float,
                x_ego: float, y_ego: float) -> np.ndarray:
        # TODO: global frame points → local frame 변환.
        # 1) 회전각 θ = -yaw_ego  (ego 좌표계로 돌리려면 음수 부호)
        # 2) R(θ) = [[cosθ, -sinθ], [sinθ, cosθ]]
        # 3) 각 점에 대해: local_pt = R · (global_pt - [x_ego, y_ego])
        # 결과를 self.local_points 에 저장하고 반환
        theta = -yaw_ego
        rotation = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ])
        translated = points - np.array([x_ego, y_ego])
        self.local_points = translated @ rotation.T
        return self.local_points


class PolynomialFitting:
    def __init__(self, degree: int, num_points: int):
        self.degree = degree
        self.n = num_points
        self.coeff = np.zeros((degree + 1, 1))

    def fit(self, points: np.ndarray) -> np.ndarray:
        # TODO: least-squares 로 다항식 계수 fitting.
        # 1) A_ij = points[i][0] ** (degree - j)   (행 i, 열 j)
        # 2) b_i = points[i][1]
        # 3) coeff = (Aᵀ A)⁻¹ Aᵀ b   (정상방정식; np.linalg.inv 사용)
        # 결과를 self.coeff 에 저장하고 반환
        a = np.zeros((self.n, self.degree + 1))
        b = np.zeros((self.n, 1))

        for i in range(self.n):
            for j in range(self.degree + 1):
                a[i][j] = points[i][0] ** (self.degree - j)
            b[i][0] = points[i][1]

        self.coeff = np.linalg.inv(a.T @ a) @ a.T @ b
        return self.coeff


class PolynomialValue:
    def __init__(self, degree: int, num_x: int):
        self.degree = degree
        self.n = num_x
        self.y = np.zeros((num_x, 1))
        self.points = np.zeros((num_x, 2))

    def calculate(self, coeff: np.ndarray, x: np.ndarray) -> np.ndarray:
        # TODO: 주어진 coeff (고차 → 저차 순서) 로 x 배열에서 y 값 평가.
        # 각 i 에 대해: y[i] = Σ_j coeff[j] · x[i] ** (degree - j)
        # 결과를 self.y, self.points 에 저장하고 self.y 반환
        for i in range(self.n):
            y_i = 0.0
            for j in range(self.degree + 1):
                y_i += coeff[j][0] * x[i] ** (self.degree - j)
            self.y[i][0] = y_i
            self.points[i] = [x[i], y_i]
        return self.y
