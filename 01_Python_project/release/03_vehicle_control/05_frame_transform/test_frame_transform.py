"""Frame Transform regression — spec for both solutions and release."""
import numpy as np
from frame_transform import Global2Local, PolynomialFitting, PolynomialValue


def test_global2local_identity():
    points = np.array([[1.0, 2.0], [3.0, 4.0]])
    g2l = Global2Local(num_points=2)
    out = g2l.convert(points, yaw_ego=0.0, x_ego=0.0, y_ego=0.0)
    assert np.allclose(out, points)


def test_global2local_translation_only():
    points = np.array([[10.0, 5.0], [20.0, 15.0]])
    g2l = Global2Local(num_points=2)
    out = g2l.convert(points, yaw_ego=0.0, x_ego=10.0, y_ego=5.0)
    assert np.allclose(out, [[0.0, 0.0], [10.0, 10.0]])


def test_global2local_rotation_90():
    """Yaw=π/2 (북향). Global (1,0) 점 → ego frame 에선 오른쪽 → local (0, -1)."""
    points = np.array([[1.0, 0.0]])
    g2l = Global2Local(num_points=1)
    out = g2l.convert(points, yaw_ego=np.pi / 2, x_ego=0.0, y_ego=0.0)
    assert np.allclose(out, [[0.0, -1.0]], atol=1e-12)


def test_polynomial_fit_exact_cubic():
    """y = 2x³ - x² + 3x - 1 의 4 개 sample 을 fit → 계수 정확."""
    coeff_truth = np.array([[2.0], [-1.0], [3.0], [-1.0]])
    xs = np.array([0.0, 1.0, 2.0, 3.0])
    ys = 2 * xs**3 - xs**2 + 3 * xs - 1
    points = np.column_stack([xs, ys])
    fitter = PolynomialFitting(degree=3, num_points=4)
    coeff = fitter.fit(points)
    assert np.allclose(coeff, coeff_truth, atol=1e-9)


def test_polynomial_fit_overdetermined():
    """degree 2, 점 5 개 (정확히 parabola 위) → coeff 정확."""
    coeff_truth = np.array([[1.5], [-2.0], [4.0]])
    xs = np.array([-2.0, -1.0, 0.0, 1.0, 2.0])
    ys = 1.5 * xs**2 - 2 * xs + 4
    points = np.column_stack([xs, ys])
    fitter = PolynomialFitting(degree=2, num_points=5)
    coeff = fitter.fit(points)
    assert np.allclose(coeff, coeff_truth, atol=1e-9)


def test_polynomial_value_constant():
    """coeff = [0, 0, 0, 7] (degree 3) → 모든 x 에서 y = 7."""
    coeff = np.array([[0.0], [0.0], [0.0], [7.0]])
    xs = np.array([-1.0, 0.0, 1.0, 5.0])
    ev = PolynomialValue(degree=3, num_x=4)
    y = ev.calculate(coeff, xs)
    assert np.allclose(y, np.full((4, 1), 7.0))


def test_polynomial_value_linear():
    """coeff = [0, 0, 2, -1] (degree 3) → y = 2x - 1."""
    coeff = np.array([[0.0], [0.0], [2.0], [-1.0]])
    xs = np.array([0.0, 1.0, 2.0, 3.0])
    ev = PolynomialValue(degree=3, num_x=4)
    y = ev.calculate(coeff, xs)
    expected = 2 * xs - 1
    assert np.allclose(y.flatten(), expected)
