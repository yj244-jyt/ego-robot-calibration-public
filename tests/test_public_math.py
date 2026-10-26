import numpy as np

from public_math.metrics import coverage_ratio, summary_stats, wilson_interval
from public_math.se3 import exp, inverse, log


def test_se3_exp_log_roundtrip():
    twist = np.array([0.1, -0.2, 0.3, 0.2, -0.1, 0.4])
    transform = exp(twist)
    np.testing.assert_allclose(exp(log(transform)), transform, atol=1e-10)
    np.testing.assert_allclose(transform @ inverse(transform), np.eye(4), atol=1e-10)


def test_report_metrics_and_coverage_interval():
    stats = summary_stats([1.0, 2.0, 3.0])
    assert stats["rmse"] > stats["mean"]
    assert coverage_ratio(95, 100) == 0.95
    lower, upper = wilson_interval(95, 100)
    assert 0.0 < lower < 0.95 < upper < 1.0
