import pytest
from services.fapf import score, consumption_prior


def test_score_range():
    """Score should be in [-0.2, 0.8] for valid inputs."""
    for ps in [0.0, 0.5, 1.0]:
        for cost in [0.0, 0.5, 1.0]:
            s = score(ps, cost, "dairy", day_of_week=0)
            assert -0.21 <= s <= 0.81, f"Score {s} out of expected range"


def test_higher_p_spoil_raises_score():
    s_low = score(0.1, 0.5, "dairy", day_of_week=0)
    s_high = score(0.9, 0.5, "dairy", day_of_week=0)
    assert s_high > s_low


def test_higher_cost_raises_score():
    s_cheap = score(0.5, 0.0, "dairy", day_of_week=0)
    s_expensive = score(0.5, 1.0, "dairy", day_of_week=0)
    assert s_expensive > s_cheap


def test_consumption_prior_all_categories():
    categories = ["dairy", "protein", "meat", "vegetable", "fruit", "fish", "cooked", "beverage"]
    for cat in categories:
        for dow in range(7):
            p = consumption_prior(cat, dow)
            assert 0.0 <= p <= 1.0, f"P_consume out of range for {cat}, dow={dow}"


def test_unknown_category_defaults():
    s = score(0.5, 0.5, "unknown_food", day_of_week=0)
    assert isinstance(s, float)
