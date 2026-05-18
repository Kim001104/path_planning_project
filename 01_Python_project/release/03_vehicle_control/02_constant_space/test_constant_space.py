"""ConstantSpace PID regression — spec for both solutions and release."""
from constant_space_pid import ConstantSpacePID
from vehicle_long_space import VehicleLong


def test_error_definition_one_step():
    pid = ConstantSpacePID(kp=0.5, kd=0.0, ki=0.0, dt=0.1, target_space=20.0)
    # target_x=30, ego_x=5, target_space=20 → error = 30 - 5 - 20 = 5
    # u = kp*5 + 0 + 0 = 2.5
    assert pid.step(target_x=30.0, ego_x=5.0) == 2.5


def test_first_call_no_derivative_spike():
    pid = ConstantSpacePID(kp=0.0, kd=10.0, ki=0.0, dt=0.1, target_space=20.0)
    # 첫 호출: D 기여 0
    assert pid.step(target_x=30.0, ego_x=0.0) == 0.0


def _run_following(kp: float, kd: float, ki: float, sim_time: float = 80.0) -> float:
    dt = 0.1
    target = VehicleLong(dt=dt, m=500.0, Ca=0.0, x0=30.0, vx0=10.0)
    ego = VehicleLong(dt=dt, m=500.0, Ca=0.5, x0=0.0, vx0=10.0)
    controller = ConstantSpacePID(kp=kp, kd=kd, ki=ki, dt=dt, target_space=20.0)
    for _ in range(int(sim_time / dt)):
        u = controller.step(target_x=target.x, ego_x=ego.x)
        ego.step(u)
        target.step(0.0)
    return target.x - ego.x


def test_pd_only_leaves_steady_state_offset():
    """ego drag 가 일정 외란 → PD-only 잔류 오차 ≈ drag/kp = 0.1/0.08 ≈ 1.25m.

    본 커리큘럼은 target control 을 PD 로 유지 — I 항은 이론에서만 언급, 합격 기준 X.
    잔류 오차의 존재 자체가 학습 포인트. 03_time_gap 은 gap_target 이 동적이라
    drag 외란을 자연스럽게 흡수 → PD-only 로 잔류 사라짐.
    """
    gap = _run_following(kp=0.08, kd=0.4, ki=0.0)
    assert abs(gap - 20.0) > 0.5, f"PD-only should leave SS offset; gap={gap}"
