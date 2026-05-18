"""PID Tuning demo — closed loop with student's KP/KD/KI on the harder plant (plotly).

실행 전 `tuning.py` 의 KP/KD/KI 값을 결정해야 합니다 (모두 0 이면 시스템이 발산).
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from pid_controller import PIDController
from plant_pid_tuning import Plant
from plotly.subplots import make_subplots
from tuning import KD, KI, KP


def main() -> None:
    dt = 0.1
    sim_time = 60.0
    steps = int(sim_time / dt)

    plant = Plant(dt, y0=1.0, disturbance=0.3, actuation_gain=0.5)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    controller = PIDController(kp=KP, kd=KD, ki=KI, dt=dt)

    t = np.zeros(steps)
    y = np.zeros(steps)
    u_arr = np.zeros(steps)
    for i in range(steps):
        t[i] = i * dt
        y[i] = plant.y
        u = controller.step(reference=0.0, measure=plant.y)
        u_arr[i] = u
        plant.step(u)

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("position", "control u"),
                        vertical_spacing=0.08)
    fig.add_hline(y=0.0, line=dict(color="black", dash="dash"),
                  annotation_text="reference", row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=y, mode="lines", name="position",
                             line=dict(color="red", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=u_arr, mode="lines", name="control u",
                             line=dict(color="blue", width=1),
                             showlegend=False), row=2, col=1)
    fig.update_xaxes(title_text="time (s)", row=2, col=1)
    fig.update_yaxes(title_text="position", row=1, col=1)
    fig.update_yaxes(title_text="u", row=2, col=1)
    fig.update_layout(
        template="plotly_white",
        title=f"PID Tuning — KP={KP}, KD={KD}, KI={KI} on harder plant "
              f"(actuation_gain=0.5, disturbance=0.3)",
    )
    fig.show()


if __name__ == "__main__":
    main()
