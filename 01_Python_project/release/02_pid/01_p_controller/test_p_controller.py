"""P Controller regression — spec for both solutions and release."""
from p_controller import PController
from plant import Plant


def test_proportional_law():
    pid = PController(kp=2.0)
    assert pid.step(reference=10.0, measure=4.0) == 12.0
    assert pid.step(reference=0.0, measure=0.0) == 0.0
    assert pid.step(reference=0.0, measure=5.0) == -10.0


def test_zero_gain_outputs_zero():
    pid = PController(kp=0.0)
    assert pid.step(reference=10.0, measure=0.0) == 0.0
    assert pid.step(reference=-7.0, measure=3.0) == 0.0


def test_closed_loop_converges():
    dt = 0.1
    plant = Plant(dt, y0=1.0)
    controller = PController(kp=2.0)
    for _ in range(int(30 / dt)):
        plant.step(controller.step(reference=0.0, measure=plant.y))
    assert abs(plant.y) < 0.05, f"closed loop did not converge; final y={plant.y}"
