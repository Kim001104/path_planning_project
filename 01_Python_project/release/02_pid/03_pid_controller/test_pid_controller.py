"""PID Controller regression — spec for both solutions and release."""
from pid_controller import PIDController
from plant_pid import Plant


def test_pd_subset_when_ki_zero():
    pid = PIDController(kp=2.0, kd=3.0, ki=0.0, dt=0.1)
    pid.step(reference=1.0, measure=0.0)        # error1 = 1.0, D = 0 (first call)
    out = pid.step(reference=1.0, measure=0.5)  # error2 = 0.5
    # u = kp*0.5 + kd*((0.5 - 1.0)/0.1) + ki*sum = 1.0 + 3.0*(-5.0) + 0 = -14.0
    assert out == -14.0


def test_first_call_no_derivative_spike():
    pid = PIDController(kp=2.0, kd=10.0, ki=0.0, dt=0.1)
    # first call: prev_error 미정 → D = 0
    assert pid.step(reference=1.0, measure=0.0) == 2.0


def test_integral_accumulates():
    dt = 0.1
    pid = PIDController(kp=0.0, kd=0.0, ki=1.0, dt=dt)
    pid.step(reference=1.0, measure=0.0)  # error_sum = 1.0 * 0.1 = 0.1
    pid.step(reference=2.0, measure=0.0)  # error_sum += 2.0 * 0.1 = 0.3
    assert abs(pid.error_sum - 0.3) < 1e-12


def test_pd_only_leaves_steady_state_error_under_disturbance():
    dt = 0.1
    plant = Plant(dt, y0=1.0, disturbance=0.5)
    controller = PIDController(kp=2.0, kd=1.0, ki=0.0, dt=dt)
    for _ in range(int(30 / dt)):
        plant.step(controller.step(reference=0.0, measure=plant.y))
    # PD-only: |y_ss| = disturbance/kp = 0.25 → > 0.1
    assert abs(plant.y) > 0.1, f"PD-only should leave SS error under disturbance; got y={plant.y}"


def test_pid_converges_under_disturbance():
    dt = 0.1
    plant = Plant(dt, y0=1.0, disturbance=0.5)
    controller = PIDController(kp=2.0, kd=1.0, ki=0.5, dt=dt)
    for _ in range(int(60 / dt)):
        plant.step(controller.step(reference=0.0, measure=plant.y))
    assert abs(plant.y) < 0.05, f"PID should converge under disturbance; got y={plant.y}"
