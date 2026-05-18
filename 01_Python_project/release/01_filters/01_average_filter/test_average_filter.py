"""AverageFilter regression — spec for both solutions and release."""
import numpy as np
from average_filter import AverageFilter


def test_first_sample_passes_through():
    af = AverageFilter()
    assert af.step(3.14) == 3.14


def test_constant_signal_is_stable():
    af = AverageFilter()
    out = 0.0
    for _ in range(10):
        out = af.step(2.0)
    assert out == 2.0


def test_converges_to_true_mean():
    rng = np.random.default_rng(0)
    af = AverageFilter()
    out = 0.0
    for s in rng.normal(loc=5.0, scale=1.0, size=10000):
        out = af.step(s)
    assert abs(out - 5.0) < 0.05
