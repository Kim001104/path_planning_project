"""MovingAverageFilter regression — spec for both solutions and release."""
from moving_average_filter import MovingAverageFilter


def test_first_sample_passes_through():
    maf = MovingAverageFilter(window=10)
    assert maf.step(3.14) == 3.14


def test_partial_fill_uses_count():
    maf = MovingAverageFilter(window=5)
    out = 0.0
    for x in [1.0, 2.0, 3.0]:
        out = maf.step(x)
    assert abs(out - 2.0) < 1e-9


def test_window_full_average():
    maf = MovingAverageFilter(window=5)
    out = 0.0
    for x in [1.0, 2.0, 3.0, 4.0, 5.0]:
        out = maf.step(x)
    assert abs(out - 3.0) < 1e-9


def test_old_values_forgotten():
    maf = MovingAverageFilter(window=3)
    out = 0.0
    for x in [1.0, 1.0, 1.0, 5.0, 5.0, 5.0]:
        out = maf.step(x)
    assert abs(out - 5.0) < 1e-9


def test_constant_signal_is_stable():
    maf = MovingAverageFilter(window=4)
    out = 0.0
    for _ in range(10):
        out = maf.step(2.0)
    assert out == 2.0
