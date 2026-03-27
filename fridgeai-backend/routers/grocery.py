from __future__ import annotations
from datetime import datetime, timezone
from typing import Optional
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, status
import asyncpg
from pydantic import BaseModel

from core.database import db_dependency
from models.item import ItemRead
from services import settle_timer
from websocket.manager import manager

_SHELF_LIFE_DEFAULTS: dict[str, int] = {
    "dairy": 7, "protein": 7, "meat": 3, "vegetable": 6,
    "fruit": 7, "fish": 2, "cooked": 4, "beverage": 7,
}

router = APIRouter(prefix="/grocery", tags=["grocery"])


class GroceryItemCreate(BaseModel):
    name: str
    category: str = "vegetable"
    quantity: int = 1
    source: str = "manual"  # manual | restock | recipe


class GroceryItemRead(BaseModel):
    grocery_id: str
    name: str
    category: str
    quantity: int
    checked: bool
    source: str
    created_at: str

    @classmethod
    def from_row(cls, row) -> "GroceryItemRead":
        return cls(
            grocery_id=row["grocery_id"],
            name=row["name"],
            category=row["category"],
            quantity=row["quantity"],
            checked=bool(row["checked"]),
            source=row["source"],
            created_at=row["created_at"],
        )


class GroceryItemUpdate(BaseModel):
    checked: Optional[bool] = None
    quantity: Optional[int] = None


@router.get("", response_model=list[GroceryItemRead])
async def list_grocery(conn: asyncpg.Connection = Depends(db_dependency)):
    rows = await conn.fetch("SELECT * FROM grocery_items ORDER BY created_at DESC")
    return [GroceryItemRead.from_row(r) for r in rows]


@router.post("", response_model=GroceryItemRead, status_code=status.HTTP_201_CREATED)
async def add_grocery(
    body: GroceryItemCreate,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    grocery_id = str(uuid4())
    now = datetime.now(tz=timezone.utc).isoformat()

    await conn.execute(
        "INSERT INTO grocery_items (grocery_id, name, category, quantity, checked, source, created_at) "
        "VALUES ($1, $2, $3, $4, 0, $5, $6)",
        grocery_id, body.name, body.category, body.quantity, body.source, now,
    )

    row = await conn.fetchrow("SELECT * FROM grocery_items WHERE grocery_id = $1", grocery_id)
    item = GroceryItemRead.from_row(row)

    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": item.model_dump(),
    })
    return item


@router.patch("/{grocery_id}", response_model=GroceryItemRead)
async def update_grocery(
    grocery_id: str,
    body: GroceryItemUpdate,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    existing = await conn.fetchrow("SELECT grocery_id FROM grocery_items WHERE grocery_id = $1", grocery_id)
    if existing is None:
        raise HTTPException(status_code=404, detail="Grocery item not found")

    updates = body.model_dump(exclude_none=True)
    if not updates:
        raise HTTPException(status_code=422, detail="No fields to update")

    keys = list(updates.keys())
    vals = list(updates.values())
    set_clause = ", ".join(f"{k} = ${i+1}" for i, k in enumerate(keys))
    await conn.execute(
        f"UPDATE grocery_items SET {set_clause} WHERE grocery_id = ${len(keys)+1}",
        *vals, grocery_id,
    )

    row = await conn.fetchrow("SELECT * FROM grocery_items WHERE grocery_id = $1", grocery_id)
    item = GroceryItemRead.from_row(row)

    now = datetime.now(tz=timezone.utc).isoformat()
    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": item.model_dump(),
    })
    return item


@router.post("/{grocery_id}/add-to-fridge", response_model=ItemRead)
async def add_to_fridge(
    grocery_id: str,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    """Promote a grocery list item into the fridge inventory and mark it as checked."""
    row = await conn.fetchrow("SELECT * FROM grocery_items WHERE grocery_id = $1", grocery_id)
    if row is None:
        raise HTTPException(status_code=404, detail="Grocery item not found")

    item_id = str(uuid4())
    now = datetime.now(tz=timezone.utc).isoformat()
    shelf_life = _SHELF_LIFE_DEFAULTS.get(row["category"], 7)

    await conn.execute(
        """INSERT INTO items
           (item_id, name, category, quantity, entry_time, shelf_life,
            location, estimated_cost, storage_temp, humidity,
            confidence_tier, updated_at)
           VALUES ($1,$2,$3,$4,$5,$6,'',0.0,4.0,50.0,'LOW',$7)""",
        item_id, row["name"], row["category"], row["quantity"],
        now, shelf_life, now,
    )

    # Mark grocery item as checked
    await conn.execute(
        "UPDATE grocery_items SET checked = 1 WHERE grocery_id = $1", grocery_id
    )

    item_row = await conn.fetchrow("SELECT * FROM items WHERE item_id = $1", item_id)
    item = ItemRead.from_row(item_row)

    settle_timer.schedule(item_id)

    await manager.broadcast({
        "event": "ITEM_INSERTED",
        "timestamp": now,
        "data": item.model_dump(),
    })

    # Broadcast updated grocery item
    updated_grocery = await conn.fetchrow("SELECT * FROM grocery_items WHERE grocery_id = $1", grocery_id)
    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": GroceryItemRead.from_row(updated_grocery).model_dump(),
    })

    return item


# Must be defined BEFORE /{grocery_id} to avoid path conflict
@router.delete("/checked", status_code=status.HTTP_204_NO_CONTENT)
async def clear_checked(conn: asyncpg.Connection = Depends(db_dependency)):
    """Delete all checked grocery items."""
    await conn.execute("DELETE FROM grocery_items WHERE checked = 1")
    now = datetime.now(tz=timezone.utc).isoformat()
    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": {"cleared_checked": True},
    })


@router.delete("/{grocery_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_grocery(
    grocery_id: str,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    existing = await conn.fetchrow("SELECT grocery_id FROM grocery_items WHERE grocery_id = $1", grocery_id)
    if existing is None:
        raise HTTPException(status_code=404, detail="Grocery item not found")

    await conn.execute("DELETE FROM grocery_items WHERE grocery_id = $1", grocery_id)

    now = datetime.now(tz=timezone.utc).isoformat()
    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": {"grocery_id": grocery_id, "deleted": True},
    })
