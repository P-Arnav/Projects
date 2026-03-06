"""
FAPF — Freshness-Aware Prioritization Framework
================================================
S(i) = α₁·P_spoil + α₂·Cost_norm − α₃·P_consume
α₁=0.50  α₂=0.30  α₃=0.20

P_consume is a static (category, day_of_week) lookup for prototype.
"""

from datetime import datetime, timezone

# [day_of_week 0=Mon … 6=Sun] → P_consume per category
_CONSUME_PRIOR: dict[str, list[float]] = {
    "dairy":     [0.60, 0.50, 0.50, 0.50, 0.60, 0.70, 0.70],
    "protein":   [0.50, 0.50, 0.50, 0.60, 0.60, 0.70, 0.70],
    "meat":      [0.40, 0.40, 0.50, 0.50, 0.60, 0.70, 0.60],
    "vegetable": [0.60, 0.60, 0.60, 0.60, 0.60, 0.50, 0.50],
    "fruit":     [0.70, 0.60, 0.60, 0.60, 0.60, 0.60, 0.60],
    "fish":      [0.40, 0.40, 0.50, 0.50, 0.60, 0.70, 0.50],
    "cooked":    [0.50, 0.60, 0.60, 0.60, 0.60, 0.50, 0.50],
    "beverage":  [0.50, 0.50, 0.50, 0.50, 0.50, 0.60, 0.60],
}
_DEFAULT_PRIOR = [0.50] * 7


def consumption_prior(category: str, day_of_week: int | None = None) -> float:
    if day_of_week is None:
        day_of_week = datetime.now(tz=timezone.utc).weekday()
    row = _CONSUME_PRIOR.get(category, _DEFAULT_PRIOR)
    return row[day_of_week % 7]


def score(
    p_spoil: float,
    cost_norm: float,
    category: str,
    day_of_week: int | None = None,
) -> float:
    """Priority score in roughly [−0.2, 0.8]. Higher → more urgent."""
    p_consume = consumption_prior(category, day_of_week)
    return 0.5 * p_spoil + 0.3 * cost_norm - 0.2 * p_consume
