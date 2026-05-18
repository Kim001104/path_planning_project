"""PID + LPF integration regression — spec for both solutions and release."""
import numpy as np
from closed_loop_lpf import closed_loop_step
from low_pass_filter import LowPassFilter
from pid_controller import PIDController
from plant_lpf import Plant

DT = 0.1
SIM_TIME = 60.0
DISTURBANCE = 0.1
NOISE_STD = 0.25
SEED = 42
KP, KD, KI = 2.0, 2.0, 0.5
ALPHA = 0.9


def _make_plant(seed: int) -> Plant:
    return Plant(
        dt=DT, y0=1.0,
        disturbance=DISTURBANCE,
        measurement_noise_std=NOISE_STD,
        rng=np.random.default_rng(seed),
    )


def test_returns_4tuple_of_floats():
    plant = _make_plant(SEED)
    lpf = LowPassFilter(alpha=ALPHA)
    pid = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    out = closed_loop_step(plant, lpf, pid, target=0.0)
    assert isinstance(out, tuple) and len(out) == 4
    for v in out:
        assert isinstance(v, float), f"expected float, got {type(v)}"


def test_filtered_loop_converges():
    plant = _make_plant(SEED)
    lpf = LowPassFilter(alpha=ALPHA)
    pid = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    steps = int(SIM_TIME / DT)
    y_trues = []
    for _ in range(steps):
        y_true, _, _, _ = closed_loop_step(plant, lpf, pid, target=0.0)
        y_trues.append(y_true)
    # 마지막 30초 (steps/2) 의 평균 절대 오차 < 0.15
    tail_mean_err = float(np.mean(np.abs(y_trues[steps // 2:])))
    assert tail_mean_err < 0.15, f"tail mean |y_true| too large: {tail_mean_err}"


def test_lpf_smooths_control_input_vs_raw():
    """필터의 진짜 가치: D 항 노이즈 증폭을 차단해 actuator 출렁임 ↓.

    추적 오차는 raw 와 비슷하지만 (LPF 위상 지연 때문) 제어 입력의 분산은 한 자리수 이상 감소.
    """
    steps = int(SIM_TIME / DT)
    tail_start = steps // 2

    # filtered (student's closed_loop_step)
    plant_f = _make_plant(SEED)
    lpf = LowPassFilter(alpha=ALPHA)
    pid_f = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    u_filtered = []
    for _ in range(steps):
        _, _, _, u = closed_loop_step(plant_f, lpf, pid_f, target=0.0)
        u_filtered.append(u)

    # raw baseline (estimator 우회) — 동일 seed 로 plant 생성
    plant_r = _make_plant(SEED)
    pid_r = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    u_raw = []
    for _ in range(steps):
        m = plant_r.measure()
        u = pid_r.step(0.0, m)
        plant_r.step(u)
        u_raw.append(u)

    std_filtered = float(np.std(u_filtered[tail_start:]))
    std_raw = float(np.std(u_raw[tail_start:]))
    assert std_filtered * 5.0 < std_raw, (
        f"LPF should reduce control std by 5×+ vs raw; "
        f"filtered={std_filtered:.4f}, raw={std_raw:.4f}, ratio={std_raw/std_filtered:.2f}x"
    )
