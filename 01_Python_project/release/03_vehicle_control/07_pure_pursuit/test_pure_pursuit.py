"""PurePursuit regression — spec for both solutions and release."""
import sys
from pathlib import Path

import numpy as np

# 05_frame_transform 의 구현을 sys.path 로 직접 import (frame_transform 패턴).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lateral_pipeline_pure_pursuit import LateralPipeline  # 본 폴더 (per-problem 사본)
from pure_pursuit import PurePursuit
from vehicle_lat_pursuit import VehicleLat

DT = 0.1


def test_pure_pursuit_formula_known_inputs():
    """coeff = [0, 0, 0, 1.0] (constant y=1.0), vx=4, L=4, lookahead=1.0
       → d_lh=4, y_lh=1, δ = atan(2·4·1 / (16 + 1 + 1e-3))
    """
    pp = PurePursuit(L=4.0, lookahead_time=1.0)
    coeff = np.array([[0.0], [0.0], [0.0], [1.0]])
    out = pp.step(coeff, vx=4.0)
    expected = float(np.arctan(8.0 / (17.0 + 1e-3)))
    assert abs(out - expected) < 1e-12


def test_stateless():
    """같은 입력 → 같은 출력 (내부 상태 없음)."""
    pp = PurePursuit(L=4.0)
    coeff = np.array([[0.01], [0.02], [0.1], [0.5]])
    out1 = pp.step(coeff, vx=3.0)
    out2 = pp.step(coeff, vx=3.0)
    assert out1 == out2


def test_zero_path_zero_steering():
    pp = PurePursuit(L=4.0)
    coeff = np.zeros((4, 1))
    assert pp.step(coeff, vx=5.0) == 0.0


def test_curved_path_tracking():
    """step(40m)+sin path, vx=3, sim 30s (Y0=2 도로 밖 시작), 평균 |lateral error| < 0.5.
    본 폴더의 lateral_pipeline_pure_pursuit 경유로 검증.
    """
    sim_time = 30.0
    vx = 3.0
    L_vehicle = 4.0
    DEGREE = 3
    NUM_POINT = 5
    X_LOCAL = np.arange(0.0, 5.0, 0.5)
    SAMPLE_XS = np.arange(NUM_POINT) * 1.0

    plant = VehicleLat(dt=DT, vx=vx, L=L_vehicle, Y0=2.0)
    pp = PurePursuit(L=L_vehicle, lookahead_time=1.0)
    pipe = LateralPipeline(
        g2l=Global2Local(NUM_POINT),
        fitter=PolynomialFitting(DEGREE, NUM_POINT),
        ev=PolynomialValue(DEGREE, np.size(X_LOCAL)),
        controller=pp,
        sample_xs=SAMPLE_XS,
        x_local=X_LOCAL,
    )

    def _ref_y(x):
        return np.where(x < 40.0, 0.0, 2.0 * (np.cos((x - 40.0) / 14.0) - 1.0))

    lookahead_x = vx * pp.lookahead_time
    errs = []
    for _ in range(int(sim_time / DT)):
        out = pipe.step(plant.X, plant.Y, plant.Yaw, vx, _ref_y, lookahead_x=lookahead_x)
        plant.step(out.delta, vx)
        ref_at_ego = 0.0 if plant.X < 40.0 else 2.0 * (np.cos((plant.X - 40.0) / 14.0) - 1.0)
        errs.append(abs(plant.Y - ref_at_ego))

    mean_err = float(np.mean(errs))
    assert mean_err < 0.5, f"PurePursuit cos-path mean error: {mean_err}"
