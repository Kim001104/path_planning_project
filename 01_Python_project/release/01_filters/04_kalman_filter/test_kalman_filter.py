"""KalmanFilter regression — spec for both solutions and release."""
import numpy as np
from kalman_filter import KalmanFilter


def test_one_step_numeric():
    kf = KalmanFilter(m=1.0, dt=0.01, q=0.1, r=0.9, p0=10.0)
    out = kf.step(measurement=1.0, control_input=0.0)
    # Hand calc: P_pred = 1.01² · 10 + 0.1 = 10.301
    #            K = 10.301 / (10.301 + 0.9) ≈ 0.91965
    #            x = K · 1.0 ≈ 0.91965
    assert abs(out - 0.91965) < 1e-4


def test_high_measurement_trust():
    kf = KalmanFilter(m=1.0, dt=0.01, q=0.1, r=1e-9, p0=10.0)
    out = kf.step(measurement=5.0, control_input=0.0)
    assert abs(out - 5.0) < 1e-3


def test_high_model_trust():
    kf = KalmanFilter(m=1.0, dt=0.01, q=1e-9, r=1e9, p0=1e-9)
    out = kf.step(measurement=100.0, control_input=0.0)
    assert abs(out) < 1e-3


def test_converges_under_noisy_measurement():
    rng = np.random.default_rng(0)
    truth = 5.0
    kf = KalmanFilter(m=1.0, dt=0.01, q=0.01, r=1.0, p0=10.0)
    # 모델 (A=1.01, B=0.01) 의 drift 를 상쇄하는 feedforward control input.
    # A·truth + B·u = truth  →  u = (1 - A)·truth / B = -truth
    u_ff = -truth
    estimates = []
    for _ in range(10000):
        z = truth + rng.normal(0, 1.0)
        estimates.append(kf.step(z, control_input=u_ff))
    mean_ss = float(np.mean(estimates[-1000:]))
    assert abs(mean_ss - truth) < 0.1
