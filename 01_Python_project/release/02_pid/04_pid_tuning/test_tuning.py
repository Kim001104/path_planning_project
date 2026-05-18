"""PID Tuning regression — verifies student-supplied KP/KD/KI close the loop."""
from pid_controller import PIDController
from plant_pid_tuning import Plant
from tuning import KD, KI, KP

# 본 과제 plant 파라미터 (변경 금지)
DT = 0.1
SIM_TIME = 60.0
DISTURBANCE = 0.3
ACTUATION_GAIN = 0.5
Y0 = 1.0


def test_gains_are_positive():
    assert KP > 0, "KP 가 양수여야 합니다"
    assert KD > 0, "KD 가 양수여야 합니다"
    assert KI > 0, "KI 가 양수여야 합니다 (외란 보상 위해 필요)"


def test_closed_loop_converges():
    plant = Plant(DT, y0=Y0, disturbance=DISTURBANCE, actuation_gain=ACTUATION_GAIN)
    controller = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    for _ in range(int(SIM_TIME / DT)):
        plant.step(controller.step(reference=0.0, measure=plant.y))
    assert abs(plant.y) < 0.1, f"폐루프 수렴 실패; final y={plant.y}"


def test_control_input_does_not_blow_up():
    plant = Plant(DT, y0=Y0, disturbance=DISTURBANCE, actuation_gain=ACTUATION_GAIN)
    controller = PIDController(kp=KP, kd=KD, ki=KI, dt=DT)
    max_abs_u = 0.0
    for _ in range(int(SIM_TIME / DT)):
        u = controller.step(reference=0.0, measure=plant.y)
        max_abs_u = max(max_abs_u, abs(u))
        plant.step(u)
    assert max_abs_u < 50.0, f"제어 입력 폭주; max|u|={max_abs_u:.2f}"
