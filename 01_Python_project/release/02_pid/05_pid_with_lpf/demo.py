"""PID + LPF demo — true vs measured vs filtered estimate, closed-loop (plotly).

실행 전 `closed_loop_lpf.py` 의 `# TODO` 를 구현해야 동작합니다.
구현 전이면 첫 호출에서 `NotImplementedError` 발생.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from closed_loop_lpf import closed_loop_step
from low_pass_filter import LowPassFilter
from pid_controller import PIDController
from plant_lpf import Plant
from plotly.subplots import make_subplots


def main() -> None:
    dt = 0.1
    sim_time = 60.0
    steps = int(sim_time / dt)

    plant = Plant(
        dt=dt, y0=1.0,
        disturbance=0.1,
        measurement_noise_std=0.25,
        rng=np.random.default_rng(seed=42),
    )
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    lpf = LowPassFilter(alpha=0.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    pid = PIDController(kp=0.0, kd=0.0, ki=0.0, dt=dt)

    t = np.zeros(steps)
    y_true = np.zeros(steps)
    y_measure = np.zeros(steps)
    y_estimate = np.zeros(steps)
    u_arr = np.zeros(steps)
    for i in range(steps):
        t[i] = i * dt
        yt, ym, ye, u = closed_loop_step(plant, lpf, pid, target=0.0)
        y_true[i], y_measure[i], y_estimate[i], u_arr[i] = yt, ym, ye, u

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("position (true / measured / LPF estimate)",
                                        "control u"),
                        vertical_spacing=0.08)
    fig.add_hline(y=0.0, line=dict(color="black", dash="dash"),
                  annotation_text="reference", row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=y_measure, mode="markers", name="measured (noisy)",
                             marker=dict(color="red", size=3, opacity=0.5)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=y_true, mode="lines", name="true",
                             line=dict(color="blue", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=y_estimate, mode="lines", name="LPF estimate",
                             line=dict(color="cyan", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=u_arr, mode="lines", name="control u",
                             line=dict(color="purple", width=1),
                             showlegend=False), row=2, col=1)
    fig.update_xaxes(title_text="time (s)", row=2, col=1)
    fig.update_yaxes(title_text="position", row=1, col=1)
    fig.update_yaxes(title_text="u", row=2, col=1)
    fig.update_layout(
        template="plotly_white",
        title="PID + LPF — closed-loop with noisy measurement (R=0.25, disturbance=0.1)",
    )
    fig.show()


if __name__ == "__main__":
    main()
