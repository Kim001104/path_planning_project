"""LowPassFilter regression — spec for both solutions and release."""
import numpy as np
from low_pass_filter import LowPassFilter


def test_first_sample_passes_through():
    lpf = LowPassFilter(alpha=0.9)
    assert lpf.step(3.14) == 3.14


def test_alpha_zero_is_pass_through():
    lpf = LowPassFilter(alpha=0.0)
    for x in [1.0, 2.0, 3.0, 4.0]:
        assert lpf.step(x) == x


def test_alpha_one_holds_first_value():
    lpf = LowPassFilter(alpha=1.0)
    first = lpf.step(7.0)
    for _ in range(20):
        assert lpf.step(99.0) == first


def test_constant_signal_is_stable():
    lpf = LowPassFilter(alpha=0.9)
    out = 0.0
    for _ in range(50):
        out = lpf.step(2.0)
    assert abs(out - 2.0) < 1e-9


def test_noise_variance_reduced():
    rng = np.random.default_rng(0)
    lpf = LowPassFilter(alpha=0.9)
    samples = rng.normal(loc=5.0, scale=1.0, size=10000)
    out = np.array([lpf.step(s) for s in samples])
    assert np.std(out[1000:]) < 0.5


def test_steady_state_mean_unbiased():
    """정상상태 평균이 truth 와 일치 — 분산만 줄이는 trivial gaming (`return 0`) 차단."""
    rng = np.random.default_rng(0)
    lpf = LowPassFilter(alpha=0.9)
    samples = rng.normal(loc=5.0, scale=1.0, size=10000)
    out = np.array([lpf.step(s) for s in samples])
    mean_ss = float(np.mean(out[1000:]))
    assert abs(mean_ss - 5.0) < 0.1
