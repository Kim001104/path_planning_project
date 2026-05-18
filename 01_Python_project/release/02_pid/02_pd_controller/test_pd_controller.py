"""PD Controller regression — spec for both solutions and release."""
from pd_controller import PDController
from plant_pd import Plant


def test_p_subset_when_kd_zero():
    pid = PDController(kp=2.0, kd=0.0, dt=0.1)
    assert pid.step(reference=10.0, measure=4.0) == 12.0
    assert pid.step(reference=0.0, measure=5.0) == -10.0
    assert pid.step(reference=0.0, measure=0.0) == 0.0


def test_first_call_has_no_d_contribution():
    pid = PDController(kp=2.0, kd=5.0, dt=0.1)
    # 첫 호출: prev_error 미정 → D = 0
    assert pid.step(reference=1.0, measure=0.0) == 2.0


def test_derivative_term_formula():
    pid = PDController(kp=2.0, kd=3.0, dt=0.1)
    pid.step(reference=1.0, measure=0.0)        # error1 = 1.0
    out = pid.step(reference=1.0, measure=0.5)  # error2 = 0.5
    # u = kp*0.5 + kd*((0.5 - 1.0)/0.1) = 1.0 + 3.0*(-5.0) = -14.0
    assert out == -14.0


def test_closed_loop_converges():
    dt = 0.1
    plant = Plant(dt, y0=1.0)
    controller = PDController(kp=2.0, kd=1.0, dt=dt)
    for _ in range(int(30 / dt)):
        plant.step(controller.step(reference=0.0, measure=plant.y))
    assert abs(plant.y) < 5e-3, f"closed loop did not converge; final y={plant.y}"
