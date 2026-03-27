from __future__ import annotations
import os
import logging
from datetime import datetime, timezone
from typing import Optional
from uuid import uuid4

from fastapi import APIRouter, Depends
import asyncpg
import httpx
from pydantic import BaseModel

from core.database import db_dependency
from websocket.manager import manager

router = APIRouter(prefix="/recipes", tags=["recipes"])
logger = logging.getLogger(__name__)

SPOONACULAR_BASE = "https://api.spoonacular.com"
SPOONACULAR_KEY = os.getenv("SPOONACULAR_API_KEY", "d0cd5334fea44828b7525009b40b0e0a")


class Recipe(BaseModel):
    meal_id: str
    name: str
    thumbnail: Optional[str] = None
    used_ingredients: list[str] = []
    missed_ingredients: list[str] = []


class CookRequest(BaseModel):
    item_ids: list[str] = []


@router.get("/suggestions", response_model=list[Recipe])
async def get_recipe_suggestions(conn: asyncpg.Connection = Depends(db_dependency)):
    """Return recipe suggestions based on current pantry items (via Spoonacular)."""
    rows = await conn.fetch(
        "SELECT DISTINCT name FROM items WHERE (p_spoil IS NULL OR p_spoil < 0.9) ORDER BY p_spoil DESC NULLS LAST"
    )

    if not rows:
        return []

    # Top 5 ingredient names
    ingredients = ",".join(row["name"].lower() for row in rows[:5])

    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            resp = await client.get(
                f"{SPOONACULAR_BASE}/recipes/findByIngredients",
                params={
                    "ingredients": ingredients,
                    "number": 5,
                    "ranking": 2,   # minimize missing ingredients
                    "ignorePantry": True,
                    "apiKey": SPOONACULAR_KEY,
                },
            )
            resp.raise_for_status()
            data = resp.json()
    except Exception as exc:
        logger.warning("Spoonacular request failed: %s", exc)
        return []

    recipes: list[Recipe] = []
    for item in data:
        recipes.append(Recipe(
            meal_id=str(item["id"]),
            name=item["title"],
            thumbnail=item.get("image"),
            used_ingredients=[i["name"] for i in item.get("usedIngredients", [])],
            missed_ingredients=[i["name"] for i in item.get("missedIngredients", [])],
        ))

    return recipes


@router.post("/{meal_id}/cook")
async def cook_recipe(
    meal_id: str,
    body: CookRequest,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    """Record cooking a recipe — decrements quantity of the specified pantry items by 1."""
    now = datetime.now(tz=timezone.utc).isoformat()
    consumed: list[str] = []

    for item_id in body.item_ids:
        row = await conn.fetchrow(
            "SELECT item_id, name, category, quantity FROM items WHERE item_id = $1", item_id
        )
        if row is None:
            continue

        new_qty = row["quantity"] - 1
        if new_qty <= 0:
            await conn.execute("DELETE FROM items WHERE item_id = $1", item_id)
            await manager.broadcast({
                "event": "ITEM_DELETED",
                "timestamp": now,
                "data": {"item_id": item_id, "reason": "consumed"},
            })
        else:
            await conn.execute(
                "UPDATE items SET quantity = $1, updated_at = $2 WHERE item_id = $3",
                new_qty, now, item_id,
            )
            await manager.broadcast({
                "event": "ITEM_UPDATED",
                "timestamp": now,
                "data": {"item_id": item_id, "changed_fields": {"quantity": new_qty}},
            })

        # Record consumption history
        hist_id = str(uuid4())
        await conn.execute(
            """INSERT INTO consumption_history
               (id, item_id, item_name, category, quantity_consumed, reason, p_spoil_at_removal, consumed_at)
               VALUES ($1,$2,$3,$4,$5,'cooked',$6,$7)""",
            hist_id, item_id, row["name"], row["category"], 1, None, now,
        )
        consumed.append(item_id)

    return {"meal_id": meal_id, "consumed": consumed}
