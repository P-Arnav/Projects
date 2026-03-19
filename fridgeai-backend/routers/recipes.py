from __future__ import annotations
import logging
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends
import aiosqlite
import httpx
from pydantic import BaseModel

from core.database import db_dependency
from websocket.manager import manager

router = APIRouter(prefix="/recipes", tags=["recipes"])
logger = logging.getLogger(__name__)

MEALDB_BASE = "https://www.themealdb.com/api/json/v1/1"


class Recipe(BaseModel):
    meal_id: str
    name: str
    thumbnail: Optional[str] = None
    category: Optional[str] = None
    area: Optional[str] = None
    instructions: Optional[str] = None
    ingredients: list[str] = []


class CookRequest(BaseModel):
    item_ids: list[str] = []


@router.get("/suggestions", response_model=list[Recipe])
async def get_recipe_suggestions(db: aiosqlite.Connection = Depends(db_dependency)):
    """Return recipe suggestions based on current pantry items (via TheMealDB)."""
    cur = await db.execute(
        "SELECT DISTINCT name FROM items WHERE (P_spoil IS NULL OR P_spoil < 0.9) ORDER BY P_spoil DESC"
    )
    rows = await cur.fetchall()

    if not rows:
        return []

    # Collect up to 6 ingredient names to query
    ingredients = [row["name"].lower() for row in rows[:6]]

    meal_map: dict[str, dict] = {}
    async with httpx.AsyncClient(timeout=8.0) as client:
        for ing in ingredients:
            try:
                resp = await client.get(f"{MEALDB_BASE}/filter.php", params={"i": ing})
                if resp.status_code == 200:
                    meals = resp.json().get("meals") or []
                    for m in meals[:3]:
                        meal_map[m["idMeal"]] = m
            except Exception:
                continue

    if not meal_map:
        return []

    recipes: list[Recipe] = []
    async with httpx.AsyncClient(timeout=8.0) as client:
        for meal_id in list(meal_map.keys())[:8]:
            try:
                resp = await client.get(f"{MEALDB_BASE}/lookup.php", params={"i": meal_id})
                if resp.status_code != 200:
                    continue
                meals = resp.json().get("meals") or []
                if not meals:
                    continue
                m = meals[0]
                ings = []
                for i in range(1, 21):
                    ing = (m.get(f"strIngredient{i}") or "").strip()
                    measure = (m.get(f"strMeasure{i}") or "").strip()
                    if ing:
                        ings.append(f"{measure} {ing}".strip() if measure else ing)
                instructions = (m.get("strInstructions") or "").strip()
                recipes.append(Recipe(
                    meal_id=m["idMeal"],
                    name=m["strMeal"],
                    thumbnail=m.get("strMealThumb"),
                    category=m.get("strCategory"),
                    area=m.get("strArea"),
                    instructions=instructions[:500] if instructions else None,
                    ingredients=ings,
                ))
            except Exception:
                continue

    return recipes


@router.post("/{meal_id}/cook")
async def cook_recipe(
    meal_id: str,
    body: CookRequest,
    db: aiosqlite.Connection = Depends(db_dependency),
):
    """Record cooking a recipe — decrements quantity of the specified pantry items by 1."""
    now = datetime.now(tz=timezone.utc).isoformat()
    consumed: list[str] = []

    for item_id in body.item_ids:
        cur = await db.execute(
            "SELECT item_id, name, quantity FROM items WHERE item_id = ?", [item_id]
        )
        row = await cur.fetchone()
        if row is None:
            continue

        new_qty = row["quantity"] - 1
        if new_qty <= 0:
            await db.execute("DELETE FROM items WHERE item_id = ?", [item_id])
            await manager.broadcast({
                "event": "ITEM_DELETED",
                "timestamp": now,
                "data": {"item_id": item_id, "reason": "consumed"},
            })
        else:
            await db.execute(
                "UPDATE items SET quantity = ?, updated_at = ? WHERE item_id = ?",
                [new_qty, now, item_id],
            )
            await manager.broadcast({
                "event": "ITEM_UPDATED",
                "timestamp": now,
                "data": {"item_id": item_id, "changed_fields": {"quantity": new_qty}},
            })
        consumed.append(item_id)

    await db.commit()
    return {"meal_id": meal_id, "consumed": consumed}
