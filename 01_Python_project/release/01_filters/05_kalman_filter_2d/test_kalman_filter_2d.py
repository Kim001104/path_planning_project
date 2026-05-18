"""KalmanFilter2D regression — spec for both solutions and release."""
import numpy as np
from kalman_filter_2d import KalmanFilter2D


def _cv_matrices(dt=0.1):
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([0.0, dt])
    C = np.array([1.0, 0.0])
    return A, B, C


def test_one_step_numeric_constant_velocity():
    A, B, C = _cv_matrices(dt=0.1)
    Q = np.diag([0.01, 0.05])
    R = 5.0
    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R,
                        x0=np.zeros(2), P0=10.0 * np.eye(2))
    out = kf.step(measurement=1.0, control_input=0.0)
    # Hand calc: P_pred = [[10.11, 1], [1, 10.05]], S = 15.11
    #            K = [10.11, 1] / 15.11 ≈ [0.6691, 0.0662]
    #            state = K · 1.0
    assert abs(out[0] - 0.6691) < 1e-3
    assert abs(out[1] - 0.0662) < 1e-3


def test_high_measurement_trust():
    A, B, C = _cv_matrices()
    Q = np.diag([0.01, 0.05])
    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=1e-9)
    out = kf.step(measurement=5.0, control_input=0.0)
    assert abs(out[0] - 5.0) < 1e-3


def test_high_model_trust():
    A, B, C = _cv_matrices()
    Q = np.diag([1e-9, 1e-9])
    P0 = 1e-9 * np.eye(2)
    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=1e9, P0=P0)
    out = kf.step(measurement=100.0, control_input=0.0)
    assert abs(out[0]) < 1e-3
    assert abs(out[1]) < 1e-3


def test_constant_velocity_tracking():
    rng = np.random.default_rng(0)
    dt = 0.1
    n_steps = 200
    v_truth = 2.0
    A, B, C = _cv_matrices(dt=dt)
    Q = np.diag([0.01, 0.05])
    R = 0.5
    kf = KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R)

    out = np.zeros(2)
    for k in range(n_steps):
        truth_pos = v_truth * k * dt
        z = truth_pos + rng.normal(0, 0.5)
        out = kf.step(measurement=z, control_input=0.0)

    truth_pos_final = v_truth * (n_steps - 1) * dt
    assert abs(out[0] - truth_pos_final) < 0.5
    assert abs(out[1] - v_truth) < 0.3
