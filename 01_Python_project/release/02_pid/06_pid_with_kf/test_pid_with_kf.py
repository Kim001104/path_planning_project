"""PID + KF integration regression — spec for both solutions and release."""
import numpy as np
from closed_loop_kf import closed_loop_step
from kalman_filter_2d import KalmanFilter2D
from pid_controller import PIDController
from plant_kf import Plant

DT = 0.1
SIM_TIME = 60.0
DISTURBANCE = 0.1
NOISE_STD = 0.25
SEED = 42
KP, KD, KI = 2.0, 2.0, 0.5
DAMPING = 1.0
M = 1.0


def _make_plant(seed: int) -> Plant:
    return Plant(
        dt=DT, y0=1.0,
        damping=DAMPING, m=M,
        disturbance=DISTURBANCE,
        measurement_noise_std=NOISE_STD,
        rng=np.random.default_rng(seed),
    )


def _make_kf() -> KalmanFilter2D:
    A = np.array([[1.0, DT], [0.0, 1.0 - DAMPING * DT / M]])
    B = np.array([0.0, DT / M])
    C = np.array([1.0, 0.0])
    Q = np.diag([1e-3, 1e-3])
    R = NOISE_STD ** 2
    x0 = np.array([1.0, 0.0])  # known initial state
    P0 = 10.0 * np.eye(2)
    return KalmanFilter2D(A=A, B=B, C=C, Q=Q, R=R, x0=x0, P0=P0)


def test_returns_4tuple_of_floats():
    plant = _make_plant(SEED)
    kf = _make_kf()
    pid = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    out = closed_loop_step(plant, kf, pid, target=0.0, prev_u=0.0)
    assert isinstance(out, tuple) and len(out) == 4
    for v in out:
        assert isinstance(v, float), f"expected float, got {type(v)}"


def _run_loop() -> tuple[list[float], list[float]]:
    """Driver loop with prev_u tracking. Returns (y_trues, estimation_errors)."""
    plant = _make_plant(SEED)
    kf = _make_kf()
    pid = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    y_trues, errs = [], []
    prev_u = 0.0
    for _ in range(steps):
        y_true, _, y_est, u = closed_loop_step(plant, kf, pid, target=0.0, prev_u=prev_u)
        y_trues.append(y_true)
        errs.append(abs(y_est - y_true))
        prev_u = u
    return y_trues, errs


def test_closed_loop_converges():
    y_trues, _ = _run_loop()
    tail_mean_err = float(np.mean(np.abs(y_trues[len(y_trues) // 2:])))
    assert tail_mean_err < 0.15, f"tail mean |y_true| too large: {tail_mean_err}"


def test_kf_estimation_better_than_raw_noise():
    """KF 추정 오차가 raw measurement 노이즈 std (0.25) 보다 의미있게 작아야."""
    _, errs = _run_loop()
    tail_est_err = float(np.mean(errs[len(errs) // 2:]))
    # raw measurement 의 평균 절대 오차 ~ noise_std * sqrt(2/π) ≈ 0.2
    # KF 가 disturbance 모델 mismatch 가 있어도 절반 이하면 OK
    assert tail_est_err < 0.10, f"KF estimation tail mean error too large: {tail_est_err}"


class _RecordingEstimator:
    """KF 호출 인자를 기록 — closed_loop_step 이 prev_u 를 estimator 에 정확히 전달하는지 검증용."""

    def __init__(self, inner: KalmanFilter2D):
        self.inner = inner
        self.calls: list[tuple[float, float]] = []

    def step(self, measurement: float, control_input: float):
        self.calls.append((measurement, control_input))
        return self.inner.step(measurement, control_input)


def test_estimator_receives_prev_u():
    """closed_loop_step 이 prev_u 를 estimator.step 의 control_input 인자로 그대로 넘겨야."""
    plant = _make_plant(SEED)
    rec = _RecordingEstimator(_make_kf())
    pid = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)

    closed_loop_step(plant, rec, pid, target=0.0, prev_u=7.5)
    closed_loop_step(plant, rec, pid, target=0.0, prev_u=-3.25)

    assert len(rec.calls) == 2, f"expected 2 estimator calls, got {len(rec.calls)}"
    _, u1 = rec.calls[0]
    _, u2 = rec.calls[1]
    assert u1 == 7.5, f"first prev_u not forwarded: got {u1}"
    assert u2 == -3.25, f"second prev_u not forwarded: got {u2}"
