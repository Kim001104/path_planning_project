"""KinematicLateral PID regression — spec for both solutions and release."""
from kinematic_lateral_pid import KinematicLateralPID
from vehicle_lat_kinematic import VehicleLat


def test_pid_law_one_step():
    pid = KinematicLateralPID(kp=1.0, kd=0.0, ki=0.0, dt=0.1)
    # error = 4.0 - 0.0 = 4.0; D=0 (first); u = 1.0 * 4.0 = 4.0
    assert pid.step(reference_Y=4.0, ego_Y=0.0) == 4.0


def test_first_call_no_derivative_spike():
    pid = KinematicLateralPID(kp=0.0, kd=10.0, ki=0.0, dt=0.1)
    # 첫 호출: D=0
    assert pid.step(reference_Y=4.0, ego_Y=0.0) == 0.0


def test_steering_clipped_by_plant():
    plant = VehicleLat(dt=0.1, vx=3.0)
    plant.step(delta=10.0, vx=3.0)
    assert plant.delta == 0.5
    plant.step(delta=-10.0, vx=3.0)
    assert plant.delta == -0.5


def test_lateral_tracking_converges():
    dt = 0.1
    sim_time = 30.0
    vx = 3.0
    Y_ref = 4.0
    plant = VehicleLat(dt=dt, vx=vx)
    pid = KinematicLateralPID(kp=0.2, kd=0.8, ki=0.0, dt=dt)
    for _ in range(int(sim_time / dt)):
        delta = pid.step(reference_Y=Y_ref, ego_Y=plant.Y)
        plant.step(delta, vx)
    assert abs(plant.Y - Y_ref) < 0.2, f"Y did not reach reference; final Y={plant.Y}"
