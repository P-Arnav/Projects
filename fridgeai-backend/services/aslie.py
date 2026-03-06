"""
ASLIE — Adaptive Shelf-Life Inference Engine
============================================
Pure math module. No I/O, no async. Independently unit-testable.

P_spoil(i, t) = sigmoid(B0 + B1*t + B2*T_n + B3*C_n + B4*H_n)

  T_n, C_n, H_n are temp / category / humidity normalised to [0,1]
  using TEMP_NORM, CAT_NORM, HUMID_NORM reference ranges in config.py.

  B0, B2, B3, B4 were fitted from the Mendeley Multi-Parameter Fruit
  Spoilage IoT dataset (features: Temp + Humidity; CO2/Light excluded).
  B1 was recalibrated so dairy at 4 degC / 50% RH reaches P_spoil=0.75
  in approx 7 days (the dataset has no elapsed-time column).

RSL = binary-search for t* where P_spoil(t*) >= theta, capped at the
      item's declared shelf_life to honour category-specific expiry rules.
"""

import math
from core.config import (
    BETA_0, BETA_1, BETA_2, BETA_3, BETA_4, THETA,
    TEMP_NORM, HUMID_NORM, CAT_NORM,
)


def _sigmoid(x: float) -> float:
    x = max(-500.0, min(500.0, x))
    return 1.0 / (1.0 + math.exp(-x))


def _normalise(x: float, xmin: float, xmax: float) -> float:
    return (x - xmin) / (xmax - xmin)


def _log_odds(
    t: float,
    temp: float,
    category_enc: int = 0,
    humidity: float = 50.0,
) -> float:
    temp_n  = _normalise(temp,         TEMP_NORM[0],  TEMP_NORM[1])
    humid_n = _normalise(humidity,     HUMID_NORM[0], HUMID_NORM[1])
    cat_n   = _normalise(category_enc, CAT_NORM[0],   CAT_NORM[1])
    return BETA_0 + BETA_1 * t + BETA_2 * temp_n + BETA_3 * cat_n + BETA_4 * humid_n


def p_spoil(
    t: float,
    temp: float,
    category_enc: int = 0,
    humidity: float = 50.0,
) -> float:
    """Spoilage probability in [0, 1] at elapsed time t (days)."""
    return _sigmoid(_log_odds(t, temp, category_enc, humidity))


def rsl(
    t_elapsed: float,
    shelf_life: int,
    temp: float,
    category_enc: int = 0,
    humidity: float = 50.0,
) -> float:
    """
    Remaining shelf life in days.

    Uses a 32-iteration binary search to find t* where P_spoil >= THETA,
    then caps the result at the item's declared shelf_life so that
    category-specific expiry rules are always respected.
    """
    lo = t_elapsed
    hi = t_elapsed + shelf_life * 3  # safety ceiling for binary search

    for _ in range(32):
        mid = (lo + hi) / 2.0
        if _sigmoid(_log_odds(mid, temp, category_enc, humidity)) >= THETA:
            hi = mid
        else:
            lo = mid

    formula_rsl = max(0.0, lo - t_elapsed)

    # Hard cap: never exceed what the declared shelf_life allows
    official_rsl = max(0.0, float(shelf_life) - t_elapsed)
    return min(formula_rsl, official_rsl)


def compute(
    t_elapsed: float,
    temp: float,
    shelf_life: int,
    category_enc: int = 0,
    humidity: float = 50.0,
) -> tuple[float, float]:
    """Return (P_spoil, RSL) for an item."""
    ps        = p_spoil(t_elapsed, temp, category_enc, humidity)
    remaining = rsl(t_elapsed, shelf_life, temp, category_enc, humidity)
    return ps, remaining
