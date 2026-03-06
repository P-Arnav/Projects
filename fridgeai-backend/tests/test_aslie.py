import math
import pytest
from services.aslie import p_spoil, rsl, compute
from core.config import THETA


def test_p_spoil_range():
    for t in [0, 1, 5, 10, 30, 100]:
        ps = p_spoil(t, temp=4.0)
        assert 0.0 <= ps <= 1.0, f"P_spoil out of range at t={t}"


def test_p_spoil_increases_with_time():
    values = [p_spoil(t, temp=4.0) for t in range(0, 60, 5)]
    assert values == sorted(values), "P_spoil should be monotonically non-decreasing"


def test_p_spoil_increases_with_temp():
    ps_cold = p_spoil(10, temp=2.0)
    ps_warm = p_spoil(10, temp=10.0)
    assert ps_warm > ps_cold


def test_rsl_converges():
    """RSL binary search should return a value where P_spoil(t+RSL) ≈ THETA."""
    remaining = rsl(t_elapsed=0, shelf_life=7, temp=4.0)
    assert remaining > 0, "RSL should be positive for fresh item"
    ps_at_expiry = p_spoil(remaining, temp=4.0)
    assert abs(ps_at_expiry - THETA) < 0.01, f"RSL did not converge near theta: got {ps_at_expiry:.4f}"


def test_rsl_zero_for_expired():
    """An item well past expiry should have RSL = 0."""
    remaining = rsl(t_elapsed=200, shelf_life=7, temp=4.0)
    assert remaining == 0.0


def test_compute_returns_tuple():
    ps, remaining = compute(t_elapsed=3, temp=4.0, shelf_life=7)
    assert isinstance(ps, float)
    assert isinstance(remaining, float)
    assert 0.0 <= ps <= 1.0
    assert remaining >= 0.0


def test_sigmoid_no_overflow():
    """Very large/small inputs should not raise."""
    ps_large = p_spoil(1000, temp=30.0)
    ps_small = p_spoil(0, temp=-10.0)
    assert 0.0 <= ps_large <= 1.0
    assert 0.0 <= ps_small <= 1.0
