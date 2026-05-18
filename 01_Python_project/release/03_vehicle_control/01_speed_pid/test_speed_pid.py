"""Speed PID regression — spec for both solutions and release."""
from speed_pid import SpeedPID
from vehicle_long_speed import VehicleLong


def test_pid_law_one_step():
    pid = SpeedPID(kp=1.0, kd=0.0, ki=0.005, dt=0.1)
    pid.step(reference=30.0, measure=10.0)        # error1 = 20, D=0 (first), sum = 20*0.1 = 2.0
    out = pid.step(reference=30.0, measure=15.0)  # error2 = 15, D=(15-20)/0.1=-50, sum=2+1.5=3.5
    # u = 1.0*15 + 0*(-50) + 0.005*3.5 = 15.0175
    assert abs(out - 15.0175) < 1e-9


def test_first_call_no_derivative_spike():
    pid = SpeedPID(kp=1.0, kd=10.0, ki=0.0, dt=0.1)
    # 첫 호출: prev_error=None → D=0
    assert pid.step(reference=30.0, measure=0.0) == 30.0


def test_speed_tracking_converges():
    dt = 0.1
    sim_time = 50.0
    plant = VehicleLong(dt=dt, vx0=0.0)
    controller = SpeedPID(kp=1.0, kd=0.0, ki=0.005, dt=dt)
    for _ in range(int(sim_time / dt)):
        u = controller.step(reference=30.0, measure=plant.vx)
        plant.step(u)
    assert abs(plant.vx - 30.0) < 0.5, f"vx did not reach reference; final vx={plant.vx}"


def test_acceleration_clipped_by_plant():
    plant = VehicleLong(dt=0.1, vx0=0.0, ax_limit=2.0)
    plant.step(ax_cmd=1000.0)   # huge command
    assert plant.ax == 2.0
    plant.step(ax_cmd=-1000.0)  # huge negative
    assert plant.ax == -2.0
