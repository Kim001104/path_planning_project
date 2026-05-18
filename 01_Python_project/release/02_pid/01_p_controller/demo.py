"""P Controller demo — closed-loop step response y0=1.0 → ref=0.0 (plotly).

실행 전 `p_controller.py` 의 `# TODO` 를 구현해야 동작합니다.
구현 전이면 첫 `step(...)` 호출에서 `NotImplementedError` 발생.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go
from p_controller import PController
from plant import Plant


def main() -> None:
    dt = 0.1
    sim_time = 30.0
    steps = int(sim_time / dt)

    plant = Plant(dt, y0=1.0)
    # [튜닝] 게인/파라미터 값을 바꿔 응답 변화 비교 — test_*.py 의 값은 변경 X (합격 기준)
    controller = PController(kp=0.0)
    target = 0.0

    t = np.zeros(steps)
    y = np.zeros(steps)
    for i in range(steps):
        t[i] = i * dt
        y[i] = plant.y
        plant.step(controller.step(target, plant.y))

    fig = go.Figure()
    fig.add_hline(y=target, line=dict(color="black", dash="dash"), annotation_text="reference")
    fig.add_trace(
        go.Scatter(x=t, y=y, mode="lines", name="position",
                   line=dict(color="red", width=2))
    )
    fig.update_layout(
        title="P Controller — closed-loop step response (Kp=2.0)",
        xaxis_title="time (s)",
        yaxis_title="position",
        template="plotly_white",
    )
    fig.show()


if __name__ == "__main__":
    main()
