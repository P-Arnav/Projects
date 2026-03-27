from __future__ import annotations
from typing import Optional

from fastapi import APIRouter, Depends
import asyncpg
from pydantic import BaseModel

from core.database import db_dependency

router = APIRouter(prefix="/restock", tags=["restock"])


class RestockSuggestion(BaseModel):
    name: str
    category: str
    reason: str
    priority: str  # "urgent" | "low_stock"
    current_qty: Optional[int] = None
    rsl: Optional[float] = None
    p_spoil: Optional[float] = None


@router.get("", response_model=list[RestockSuggestion])
async def get_restock_suggestions(conn: asyncpg.Connection = Depends(db_dependency)):
    """
    Suggest items to restock based on current pantry state:
    - Urgent: RSL < 2 days and P_spoil > 0.5 (will go bad before you can restock)
    - Low stock: quantity == 1 and P_spoil > 0.4 (running out while spoiling)
    """
    rows = await conn.fetch(
        "SELECT name, category, quantity, rsl, p_spoil FROM items"
    )

    suggestions: list[RestockSuggestion] = []
    seen: set[str] = set()

    for row in rows:
        name = row["name"]
        category = row["category"]
        qty = row["quantity"]
        rsl = row["rsl"]
        p_spoil = row["p_spoil"]

        if name.lower() in seen:
            continue

        if rsl is not None and rsl < 2.0 and p_spoil is not None and p_spoil > 0.5:
            suggestions.append(RestockSuggestion(
                name=name,
                category=category,
                reason=f"Expires in {rsl:.1f} day(s) — replace soon",
                priority="urgent",
                current_qty=qty,
                rsl=rsl,
                p_spoil=p_spoil,
            ))
            seen.add(name.lower())

        elif qty == 1 and p_spoil is not None and p_spoil > 0.4:
            suggestions.append(RestockSuggestion(
                name=name,
                category=category,
                reason="Only 1 left and spoiling",
                priority="low_stock",
                current_qty=qty,
                rsl=rsl,
                p_spoil=p_spoil,
            ))
            seen.add(name.lower())

    # Sort: urgent first, then by highest P_spoil
    suggestions.sort(key=lambda s: (0 if s.priority == "urgent" else 1, -(s.p_spoil or 0)))
    return suggestions
