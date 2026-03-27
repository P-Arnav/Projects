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

SPOONACULAR_API_KEY = os.getenv("SPOONACULAR_API_KEY", "")
SPOONACULAR_BASE = "https://api.spoonacular.com"


class Recipe(BaseModel):
    meal_id: str
    name: str
    thumbnail: Optional[str] = None
    used_ingredients: list[str] = []
    missed_ingredients: list[str] = []


class RecipeDetails(BaseModel):
    meal_id: str
    source_url: Optional[str] = None
    ready_in_minutes: Optional[int] = None
    servings: Optional[int] = None
    steps: list[str] = []


class CookRequest(BaseModel):
    item_ids: list[str] = []


@router.get("/suggestions", response_model=list[Recipe])
async def get_recipe_suggestions(conn: asyncpg.Connection = Depends(db_dependency)):
    """Return recipe suggestions based on current pantry items (via Spoonacular)."""
    rows = await conn.fetch(
        """SELECT name, MAX(COALESCE(p_spoil, 0)) AS ps
           FROM items
           WHERE (p_spoil IS NULL OR p_spoil < 0.9)
           GROUP BY name
           ORDER BY ps DESC"""
    )

    if not rows:
        return []

    ingredient_list = [row["name"] for row in rows[:8]]
    ingredients_str = ",".join(ingredient_list)

    try:
        async with httpx.AsyncClient(timeout=15.0) as client:
            resp = await client.get(
                f"{SPOONACULAR_BASE}/recipes/findByIngredients",
                params={
                    "ingredients": ingredients_str,
                    "number": 5,
                    "ranking": 1,
                    "ignorePantry": True,
                    "apiKey": SPOONACULAR_API_KEY,
                },
            )
            resp.raise_for_status()
            data = resp.json()
    except Exception as exc:
        logger.error("Spoonacular suggestions request failed: %s", exc, exc_info=True)
        return []

    recipes: list[Recipe] = []
    for item in data:
        recipes.append(Recipe(
            meal_id=str(item["id"]),
            name=item.get("title", "Unknown Recipe"),
            thumbnail=item.get("image"),
            used_ingredients=[i["name"] for i in item.get("usedIngredients", [])],
            missed_ingredients=[i["name"] for i in item.get("missedIngredients", [])],
        ))

    return recipes


@router.get("/{meal_id}/details", response_model=RecipeDetails)
async def get_recipe_details(meal_id: str):
    """Fetch step-by-step cooking instructions from Spoonacular."""
    try:
        async with httpx.AsyncClient(timeout=15.0) as client:
            resp = await client.get(
                f"{SPOONACULAR_BASE}/recipes/{meal_id}/information",
                params={"apiKey": SPOONACULAR_API_KEY},
            )
            resp.raise_for_status()
            data = resp.json()
    except Exception as exc:
        logger.error("Spoonacular details request failed: %s", exc, exc_info=True)
        return RecipeDetails(meal_id=meal_id)

    steps: list[str] = []
    for instruction_block in data.get("analyzedInstructions", []):
        for step in instruction_block.get("steps", []):
            steps.append(step.get("step", ""))

    return RecipeDetails(
        meal_id=meal_id,
        source_url=data.get("sourceUrl"),
        ready_in_minutes=data.get("readyInMinutes"),
        servings=data.get("servings"),
        steps=steps,
    )


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
