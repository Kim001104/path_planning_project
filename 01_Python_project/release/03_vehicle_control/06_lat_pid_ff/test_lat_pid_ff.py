"""LatPIDFF regression — spec for both solutions and release."""
import sys
from pathlib import Path

import numpy as np

# 05_frame_transform 의 구현을 그대로 import (폴더명이 숫자 prefix 라 sys.path 추가 필요).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lat_pid_ff import LatPIDFF
from lateral_pipeline_pid_ff import LateralPipeline
from vehicle_lat_pid import VehicleLat

DT = 0.1
DEGREE = 3
NUM_POINT = 5
X_LOCAL = np.arange(0.0, 10.0, 0.5)
SAMPLE_XS = np.arange(NUM_POINT) * 1.0  # 0..4m, 1m 간격 (record_gen 과 동일)


def _ref_y(x):
    """직선(첫 40m) → sine(이후) 경로. record_gen 과 동일."""
    L_straight = 40.0
    if np.isscalar(x):
        return 0.0 if x < L_straight else 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0)
    return np.where(x < L_straight, 0.0, 2.0 * (np.cos((x - L_straight) / 14.0) - 1.0))


def _make_pipeline(controller: LatPIDFF) -> LateralPipeline:
    g2l = Global2Local(NUM_POINT)
    fitter = PolynomialFitting(DEGREE, NUM_POINT)
    ev = PolynomialValue(DEGREE, np.size(X_LOCAL))
    return LateralPipeline(g2l, fitter, ev, controller, SAMPLE_XS, X_LOCAL)


def _run_closed_loop(controller: LatPIDFF, vx: float, sim_time: float) -> tuple[list, list, list]:
    """Returns (X_ego, Y_ego, lateral_errors). pipeline 경유."""
    pipe = _make_pipeline(controller)
    plant = VehicleLat(dt=DT, vx=vx, Y0=1.0)
    X_ego, Y_ego, errs = [], [], []
    steps = int(sim_time / DT)
    lookahead_x = vx * controller.lookahead_time
    for _ in range(steps):
        X_ego.append(plant.X)
        Y_ego.append(plant.Y)
        out = pipe.step(plant.X, plant.Y, plant.Yaw, vx, _ref_y, lookahead_x=lookahead_x)
        plant.step(out.delta, vx)
        errs.append(abs(plant.Y - float(_ref_y(plant.X))))
    return X_ego, Y_ego, errs


def test_pid_only_law_when_kff_zero():
    """kff=0 → ff_term 무시. 알려진 coeff (기울기만) + vx 에서 한 step 결과 검증."""
    pid = LatPIDFF(kp=1.0, kd=0.0, ki=0.0, kff=0.0, dt=DT, lookahead_time=0.5)
    # coeff [a,b,c,d] degree 3: y = a*x^3 + b*x^2 + c*x + d
    # 모두 0 빼고 d=2.0 (constant) → polyval at any x = 2.0
    coeff = np.array([[0.0], [0.0], [0.0], [2.0]])
    out = pid.step(coeff, vx=4.0)
    # error=2.0, D=0 (first), sum=0.2, ff_term=0, u = 1*2 + 0 + 0 + 0 = 2.0
    assert abs(out - 2.0) < 1e-12


def test_ff_term_scales_with_vx_squared():
    """kff=1, kp=kd=ki=0, coeff[-3]=0.5 → step 결과 = vx² · 2 · 0.5 = vx²."""
    pid = LatPIDFF(kp=0.0, kd=0.0, ki=0.0, kff=1.0, dt=DT, lookahead_time=0.5)
    coeff = np.array([[0.0], [0.5], [0.0], [0.0]])  # coeff[-3] = 0.5
    # error = polyval at d_lh (0.5*vx), but kp=kd=ki=0 → only ff_term:
    # ff = vx² · 2 · coeff[-3] = vx² · 1.0 = vx²
    out = pid.step(coeff, vx=5.0)
    assert abs(out - 25.0) < 1e-9


def test_first_call_no_derivative_spike():
    pid = LatPIDFF(kp=0.0, kd=10.0, ki=0.0, kff=0.0, dt=DT)
    coeff = np.array([[0.0], [0.0], [0.0], [3.0]])
    # 첫 호출: D=0, kp=ki=kff=0 → step = 0
    assert pid.step(coeff, vx=1.0) == 0.0


def test_low_speed_pid_only_tracks():
    """vx=3, PID-only (kff=0) — 곡률 작아 FF 없이도 OK. pipeline + controller 통합 검증."""
    controller = LatPIDFF(kp=0.2, kd=0.1, ki=0.0, kff=0.0, dt=DT)
    _, _, errs = _run_closed_loop(controller, vx=3.0, sim_time=30.0)
    mean_err = float(np.mean(errs))
    assert mean_err < 0.5, f"low-speed PID-only mean error too large: {mean_err}"


def test_high_speed_ff_beats_no_ff():
    """vx=10, FF (kff 적정값) 가 PID-only 대비 평균 lateral error 30% 이상 감소."""
    pid_only = LatPIDFF(kp=0.2, kd=0.1, ki=0.0, kff=0.0, dt=DT)
    pid_ff = LatPIDFF(kp=0.2, kd=0.1, ki=0.0, kff=0.1, dt=DT)
    _, _, errs_no_ff = _run_closed_loop(pid_only, vx=10.0, sim_time=15.0)
    _, _, errs_ff = _run_closed_loop(pid_ff, vx=10.0, sim_time=15.0)
    mean_no_ff = float(np.mean(errs_no_ff))
    mean_ff = float(np.mean(errs_ff))
    assert mean_ff < 0.85 * mean_no_ff, (
        f"FF should improve >30%; no_ff={mean_no_ff:.4f}, ff={mean_ff:.4f}, "
        f"ratio={mean_ff/mean_no_ff:.3f}"
    )
