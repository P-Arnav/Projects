"""
paif.py — Proactive Actionable Intelligence Framework (PAIF)
Generates a single, time-sensitive action recommendation from ASLIE outputs.
"""

_FREEZABLE = {"meat", "fish", "protein", "cooked"}


def recommend(p_spoil: float, rsl: float, category: str) -> str | None:
    """
    Return a human-readable action string, or None when no action is needed.

    Priority order (highest urgency first):
      1. Discard         — P_spoil > 0.90 or RSL <= 0
      2. Freeze now      — RSL < 1 d, freezable category
      3. Use today       — RSL < 1 d
      4. Freeze / 24 h   — RSL < 2 d, freezable
      5. Use within 24 h — RSL < 2 d
      6. Cook first      — P_spoil > 0.80, freezable
      7. Use this first  — P_spoil > 0.80
      8. Plan to use     — P_spoil > 0.50
      9. None            — safe, no action required
    """
    freezable = category in _FREEZABLE

    if p_spoil > 0.90 or rsl <= 0:
        return "Discard — likely spoiled"

    if rsl < 1:
        return "Freeze now" if freezable else "Use today"

    if rsl < 2:
        return "Freeze or use within 24 h" if freezable else "Use within 24 h"

    if p_spoil > 0.80:
        return "Cook immediately or freeze" if freezable else "Use this first"

    if p_spoil > 0.50:
        return "Plan to use soon"

    return None
