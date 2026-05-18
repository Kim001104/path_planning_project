"""PID Controller demo — PD vs PID under constant disturbance (plotly).

실행 전 `pid_controller.py` 의 `# TODO` 를 구현해야 동작합니다.
구현 전이면 첫 `step(...)` 호출에서 `NotImplementedError` 발생.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from pid_controller import PIDController
from plant_pid import Plant
from plotly.subplots import make_subplots


def _run(kp: float, kd: float, ki: float, dt: float, sim_time: float,
         disturbance: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    steps = int(sim_time / dt)
    plant = Plant(dt, y0=1.0, disturbance=disturbance)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    controller = PIDController(kp=kp, kd=kd, ki=ki, dt=dt)
    t = np.zeros(steps)
    y = np.zeros(steps)
    u_arr = np.zeros(steps)
    for i in range(steps):
        t[i] = i * dt
        y[i] = plant.y
        u = controller.step(reference=0.0, measure=plant.y)
        u_arr[i] = u
        plant.step(u)
    return t, y, u_arr


def main() -> None:
    dt = 0.1
    sim_time = 60.0
    disturbance = 0.5

    t_pd, y_pd, u_pd = _run(kp=0.0, kd=0.0, ki=0.0, dt=dt,
                            sim_time=sim_time, disturbance=disturbance)
    t_pid, y_pid, u_pid = _run(kp=0.0, kd=0.0, ki=0.0, dt=dt,
                               sim_time=sim_time, disturbance=disturbance)

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("position", "control u"),
                        vertical_spacing=0.08)
    fig.add_hline(y=0.0, line=dict(color="black", dash="dash"),
                  annotation_text="reference", row=1, col=1)
    fig.add_trace(go.Scatter(x=t_pd, y=y_pd, mode="lines", name="PD (Ki=0)",
                             line=dict(color="gray", width=2, dash="dot")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_pid, y=y_pid, mode="lines", name="PID (Ki=0.5)",
                             line=dict(color="red", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_pd, y=u_pd, mode="lines", name="u (PD)",
                             line=dict(color="gray", width=1, dash="dot"),
                             showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_pid, y=u_pid, mode="lines", name="u (PID)",
                             line=dict(color="red", width=1),
                             showlegend=False), row=2, col=1)
    fig.update_xaxes(title_text="time (s)", row=2, col=1)
    fig.update_yaxes(title_text="position", row=1, col=1)
    fig.update_yaxes(title_text="u", row=2, col=1)
    fig.update_layout(template="plotly_white",
                      title=f"PD vs PID — closed-loop with constant disturbance ({disturbance})")
    fig.show()


if __name__ == "__main__":
    main()
