# 과제 — PD Controller (비례 + 미분 제어)

## 목표
P 제어기에 미분(D) 항을 더해 응답을 빠르게 하고 오버슈트를 줄인다.
"이전 오차를 기억하는" 첫 stateful 제어기 — 내부 상태 갱신 패턴을 익힌다.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
class PDController:
    def __init__(self, kp: float, kd: float, dt: float): ...
    def step(self, reference: float, measure: float) -> float
```

- `dt` 는 D 항의 미분 정규화에 필요 (단위: 초).
- 내부 상태로 `prev_error` 를 보관 — 첫 호출 시점엔 `None` (이전 오차 없음).

## 구현 위치
`01_Python_project_refactored/release/02_pid/02_pd_controller/pd_controller.py` 의 `step` 메소드 안 `# TODO:` 블록.

## 실행

> 환경 셋업 (1회) 과 명령 실행 규칙은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project_refactored/release/02_pid/02_pd_controller/ -v
```

데모 (시각 확인, 선택 — 구현 후):
```bash
uv run python 01_Python_project_refactored/release/02_pid/02_pd_controller/demo.py
```
→ 기본 브라우저에 P-only vs PD 비교 plotly 그래프 (위치 패널 + 제어 입력 패널).

## 합격 기준 (`pytest` 통과)
1. **Kd=0 시 P 동작** — `kd=0` 이면 `step` 결과가 P 제어기와 동일 (`Kp · 오차`)
2. **첫 호출 D=0** — `kd ≠ 0` 이어도 첫 호출에선 D 기여 0 (`Kp · 오차` 만 반환)
3. **D 항 식** — 두 번째 호출부터 `Kd · (오차 − 이전오차) / dt` 가 정상 반영
4. **폐루프 수렴 (PD 가 P 보다 빠름)** — `Kp=2.0, Kd=1.0, dt=0.1`, 초기 1.0, 목표 0.0, 30 초 시뮬 후 위치 절대값 < 5e-3 (P-only 로는 못 넘기는 기준)

## 힌트
- 일반 형태: `u = Kp · 오차 + Kd · 오차의 시간 미분` (`오차 = 목표 − 현재`)
- 미분 근사: `(오차 − 이전오차) / dt`
- **첫 호출**: 이전 오차가 없으므로 D 기여를 0 으로 두고 P 항만 반환. 그 다음 `prev_error` 를 현재 오차로 설정.
- **호출 끝**: `self.prev_error` 를 잊지 말고 갱신해야 다음 호출에서 D 가 정상 동작.

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
(공통 제약은 [`../../README.md`](../../README.md) 참조)

- **`plant_pd.py` 절대 수정 금지** — 검증 환경(plant)의 일부.
- D 항을 위한 `prev_error` 외에 추가 내부 상태(예: 적분합)는 본 과제 범위 밖.
