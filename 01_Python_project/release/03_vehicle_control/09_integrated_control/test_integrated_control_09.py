"""Integrated control regression — spec for both solutions and release.

09 의 합격 기준 (ego 는 lane2 유지, target 침범 시 timegap 으로 따라감):
  1. speed PID law (단위)
  2. timegap PD law (단위)
  3. LongitudinalDecision latch
  4. LateralController adapter — PP/Stanley/PIDFF 셋 다 .step/.lookahead_x 호출 가능
  5. closed-loop 30s sim — lane2 추종 유지, invasion 후 timegap 으로 target follow, 충돌 X
"""
import sys
from pathlib import Path

import numpy as np

# 05_frame_transform 의 구현을 그대로 import (폴더명이 숫자 prefix 라 sys.path 추가 필요).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "05_frame_transform"))
from control_pipeline import (
    ControlPipeline,
    EgoState,
    LongitudinalDecision,
    Road,
    TargetState,
)
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue
from lateral_controller import LateralController, PurePursuit
from longitudinal_controller import LongitudinalController
from vehicle_combined import VehicleCombined

DT = 0.1
DEGREE = 3
NUM_POINT = 10
X_LOCAL = np.arange(0.0, 13.0, 0.5)
SAMPLE_XS = np.arange(NUM_POINT) * 1.2  # 0..10.8m


def target_state(t: float, road: Road, vx_t: float = 8.0,
                 X0: float = 50.0, t_invasion: float = 10.0,
                 T_invasion: float = 5.0) -> TargetState:
    X = X0 + vx_t * t
    if t < t_invasion:
        Y = road.lane1_center(X)
    elif t < t_invasion + T_invasion:
        phase = (t - t_invasion) / T_invasion
        blend = 0.5 * (1.0 - np.cos(np.pi * phase))
        Y = (1.0 - blend) * road.lane1_center(X) + blend * road.lane2_center(X)
    else:
        Y = road.lane2_center(X)
    return TargetState(X=float(X), Y=float(Y), vx=vx_t)


def test_speed_pid_law():
    ctrl = LongitudinalController(dt=DT, kp_v=1.0, kd_v=0.5, kp_g=0.0, kd_g=0.0, tau_gap=1.5)
    out1 = ctrl.speed_step(v_des=10.0, v_ego=7.0)
    assert abs(out1 - 3.0) < 1e-9
    out2 = ctrl.speed_step(v_des=10.0, v_ego=8.0)
    assert abs(out2 - (-3.0)) < 1e-9


def test_timegap_law():
    ctrl = LongitudinalController(dt=DT, kp_v=0.0, kd_v=0.0, kp_g=0.3, kd_g=1.0, tau_gap=1.5)
    # gap = 20, desired = 15, gap_err = +5, rel_v = -2 → ax = 0.3*5 + 1.0*(-2) = -0.5
    out = ctrl.timegap_step(gap=20.0, v_ego=10.0, v_target=8.0)
    assert abs(out - (-0.5)) < 1e-9


def test_long_decision_latch():
    road = Road(R=200.0)
    dec = LongitudinalDecision(road)
    ego = EgoState(X=0.0, Y=-1.75, Yaw=0.0, vx=10.0)
    t1 = TargetState(X=50.0, Y=float(road.lane1_center(50.0)), vx=8.0)
    assert dec.long_mode(0.0, ego, t1) == "speed"
    t2 = TargetState(X=70.0, Y=float(road.lane2_center(70.0)), vx=8.0)
    assert dec.long_mode(1.0, ego, t2) == "timegap"
    # latch: 다시 lane1 위치로 돌아가도 timegap 유지
    t3 = TargetState(X=80.0, Y=float(road.lane1_center(80.0)), vx=8.0)
    assert dec.long_mode(2.0, ego, t3) == "timegap"


def test_lateral_adapter_pure_pursuit():
    pp = PurePursuit(L=4.0, lookahead_time=1.0)
    lat = LateralController(pp, lookahead_x_fn=lambda vx: vx * pp.lookahead_time)
    coeff = np.array([[0.0], [0.0], [0.0], [1.0]])
    delta = lat.step(coeff, vx=4.0)
    expected = float(np.arctan(2.0 * 4.0 * 1.0 / (16.0 + 1.0 + 1e-3)))
    assert abs(delta - expected) < 1e-12
    assert abs(lat.lookahead_x(vx=4.0) - 4.0) < 1e-12


def test_lateral_adapter_stanley_compat():
    """Stanley (08) 도 동일 adapter 로 wrap 가능 확인."""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "08_stanley"))
    from stanley import Stanley
    s = Stanley(k=1.0)
    lat = LateralController(s, lookahead_x_fn=lambda vx: 0.0)
    coeff = np.array([[0.0], [0.0], [0.1], [0.5]])
    delta = lat.step(coeff, vx=5.0)
    expected = 0.1 + float(np.arctan(0.5 / (5.0 + 1e-3)))
    assert abs(delta - expected) < 1e-12
    assert lat.lookahead_x(vx=5.0) == 0.0


def test_lateral_adapter_lat_pid_ff_compat():
    """LatPIDFF (06) 도 동일 adapter 로 wrap 가능 확인."""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "06_lat_pid_ff"))
    from lat_pid_ff import LatPIDFF
    pid = LatPIDFF(kp=1.0, kd=0.0, ki=0.0, kff=0.0, dt=DT, lookahead_time=0.5)
    lat = LateralController(pid, lookahead_x_fn=lambda vx: vx * pid.lookahead_time)
    coeff = np.array([[0.0], [0.0], [0.0], [2.0]])
    delta = lat.step(coeff, vx=4.0)
    assert abs(delta - 2.0) < 1e-12
    assert abs(lat.lookahead_x(vx=4.0) - 2.0) < 1e-12


def _make_pipeline(road: Road, lat_ctrl: LateralController, long_ctrl: LongitudinalController,
                   v_des: float = 10.0) -> tuple[ControlPipeline, LongitudinalDecision]:
    decision = LongitudinalDecision(road)
    pipe = ControlPipeline(
        g2l=Global2Local(NUM_POINT),
        fitter=PolynomialFitting(DEGREE, NUM_POINT),
        ev=PolynomialValue(DEGREE, int(X_LOCAL.size)),
        lat_ctrl=lat_ctrl,
        long_ctrl=long_ctrl,
        decision=decision,
        ref_y_fn=road.lane2_center,
        sample_xs=SAMPLE_XS,
        x_local=X_LOCAL,
        v_des=v_des,
    )
    return pipe, decision


def test_closed_loop_integration():
    road = Road(R=200.0)
    pp = PurePursuit(L=4.0, lookahead_time=1.0)
    lat_ctrl = LateralController(pp, lookahead_x_fn=lambda vx: vx * pp.lookahead_time)
    long_ctrl = LongitudinalController(dt=DT, kp_v=0.5, kd_v=0.0, kp_g=2.0, kd_g=3.0, tau_gap=1.5)
    pipe, decision = _make_pipeline(road, lat_ctrl, long_ctrl, v_des=10.0)

    plant = VehicleCombined(dt=DT, vx0=10.0, X0=0.0, Y0=float(road.lane2_center(0.0)), Yaw0=0.0)
    sim_time = 45.0
    steps = int(sim_time / DT)
    modes: list[str] = []
    Y_ego_arr: list[float] = []
    X_ego_arr: list[float] = []
    X_tgt_arr: list[float] = []
    vx_ego_arr: list[float] = []
    for i in range(steps):
        t = i * DT
        ego = EgoState(X=plant.X, Y=plant.Y, Yaw=plant.Yaw, vx=plant.vx)
        tgt = target_state(t, road)
        out = pipe.step(t, ego, tgt)
        plant.step(out.delta, out.ax)
        modes.append(out.long_mode)
        Y_ego_arr.append(plant.Y); X_ego_arr.append(plant.X); X_tgt_arr.append(tgt.X)
        vx_ego_arr.append(plant.vx)

    assert decision.invaded, "target invasion 후 timegap mode latch 안 됨"
    assert "timegap" in modes

    n_last = int(5.0 / DT)
    Y_last = np.array(Y_ego_arr[-n_last:])
    X_last = np.array(X_ego_arr[-n_last:])
    lane2_at = road.lane2_center(X_last)
    mean_lat_err = float(np.mean(np.abs(Y_last - lane2_at)))
    assert mean_lat_err < 0.4, f"lane2 추종 평균 lat err 큼: {mean_lat_err:.3f}"

    # sim 마지막 5s — ego.vx 가 v_des=10 보다 작아짐 (timegap 으로 감속 발생).
    # 단 target.vx=8 평형 정확 도달은 검증 X — gain/시뮬 길이 따라 가까이 수렴까지 시간차.
    vx_last_mean = float(np.mean(vx_ego_arr[-n_last:]))
    assert vx_last_mean < 9.5, f"ego 가 감속 안 함: vx_last_mean={vx_last_mean:.2f} (v_des=10)"
    assert vx_last_mean > 7.0, f"ego 가 과도 감속: vx_last_mean={vx_last_mean:.2f} (target vx=8)"

    gap_arr = np.array(X_tgt_arr) - np.array(X_ego_arr)
    assert gap_arr.min() > 2.0, f"min gap 너무 작음 (충돌 위험): {gap_arr.min():.2f}"
