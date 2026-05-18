"""TimeGap PID regression — spec for both solutions and release."""
from time_gap_pid import TimeGapPID
from vehicle_long_tg import VehicleLong


def test_time_gap_formula_one_step():
    pid = TimeGapPID(kp=1.0, kd=0.0, ki=0.0, dt=0.1, time_gap=1.0)
    # target_x=50, ego_x=10, ego_vx=10 → target_space = 10·1 = 10, error = 50-10-10 = 30, u = 30
    assert pid.step(target_x=50.0, ego_x=10.0, ego_vx=10.0) == 30.0


def test_first_call_no_derivative_spike():
    pid = TimeGapPID(kp=0.0, kd=10.0, ki=0.0, dt=0.1)
    # 첫 호출: D=0
    assert pid.step(target_x=50.0, ego_x=10.0, ego_vx=10.0) == 0.0


def test_stationary_target_converges_to_time_gap():
    """정속 target — 정상상태에서 ego 가 1초 간격으로 추종.

    초기 조건 = ACC 인계 시점: 이미 정상 time-gap 거리 (gap0 = ego_vx · time_gap = 10m).
    """
    dt = 0.1
    sim_time = 50.0
    target = VehicleLong(dt=dt, Ca=0.0, x0=10.0, vx0=10.0)
    ego = VehicleLong(dt=dt, Ca=0.5, x0=0.0, vx0=10.0)
    pid = TimeGapPID(kp=0.3, kd=1.0, ki=0.0, dt=dt, time_gap=1.0)
    for _ in range(int(sim_time / dt)):
        u = pid.step(target_x=target.x, ego_x=ego.x, ego_vx=ego.vx)
        ego.step(u)
        target.step(0.0)
    final_time_gap = (target.x - ego.x) / ego.vx
    assert abs(final_time_gap - 1.0) < 0.2, (
        f"time_gap did not settle near 1.0s; final={final_time_gap}s"
    )


def _maneuvering_a_t(t: float) -> float:
    if t < 20:
        return 0.0
    elif t < 40:
        return 1.5
    elif t < 60:
        return -1.5
    else:
        return 0.0


def test_maneuvering_target_no_collision_and_reconverges():
    """가/감속 하는 target — 충돌 없고 정속 복귀 후 1초 gap 재수렴.

    초기 조건 = ACC 인계 시점: ego_vx == target_vx, gap0 = ego_vx · time_gap = 10m
    (이미 정상 time-gap 거리, transient 없이 maneuver 만 흡수). 게인은 정지 시나리오와 동일.
    """
    dt = 0.1
    sim_time = 80.0
    target = VehicleLong(dt=dt, Ca=0.0, x0=10.0, vx0=10.0)
    ego = VehicleLong(dt=dt, Ca=0.5, x0=0.0, vx0=10.0)
    pid = TimeGapPID(kp=0.3, kd=1.0, ki=0.0, dt=dt, time_gap=1.0)
    min_gap = float("inf")
    for i in range(int(sim_time / dt)):
        t = i * dt
        u = pid.step(target_x=target.x, ego_x=ego.x, ego_vx=ego.vx)
        ego.step(u)
        target.step(_maneuvering_a_t(t))
        gap = target.x - ego.x
        if gap < min_gap:
            min_gap = gap
    assert min_gap > 0.0, f"collision occurred; min gap = {min_gap}m"
    final_time_gap = (target.x - ego.x) / ego.vx
    assert abs(final_time_gap - 1.0) < 0.5, (
        f"time_gap did not re-converge; final={final_time_gap}s"
    )
