"""
corrections.py — Adaptive Learning Loop
Reads user feedback to compute per-category shelf-life correction offsets.
The offset is applied to shelf_life in scorer.py before ASLIE runs.
"""


async def get_correction(conn, category: str) -> float:
    """
    Return the average shelf_life correction (days) for a category.
    Positive = items last longer than declared; negative = shorter.
    Returns 0.0 when no feedback exists for the category.
    """
    val = await conn.fetchval(
        "SELECT AVG(correction) FROM feedback WHERE category = $1",
        category,
    )
    return val if val is not None else 0.0
