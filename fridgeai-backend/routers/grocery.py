from __future__ import annotations
from datetime import datetime, timezone
from typing import Optional
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, status
import aiosqlite
from pydantic import BaseModel

from core.database import db_dependency
from websocket.manager import manager

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
async def list_grocery(db: aiosqlite.Connection = Depends(db_dependency)):
    cur = await db.execute("SELECT * FROM grocery_items ORDER BY created_at DESC")
    rows = await cur.fetchall()
    return [GroceryItemRead.from_row(r) for r in rows]


@router.post("", response_model=GroceryItemRead, status_code=status.HTTP_201_CREATED)
async def add_grocery(
    body: GroceryItemCreate,
    db: aiosqlite.Connection = Depends(db_dependency),
):
    grocery_id = str(uuid4())
    now = datetime.now(tz=timezone.utc).isoformat()

    await db.execute(
        "INSERT INTO grocery_items (grocery_id, name, category, quantity, checked, source, created_at) "
        "VALUES (?, ?, ?, ?, 0, ?, ?)",
        [grocery_id, body.name, body.category, body.quantity, body.source, now],
    )
    await db.commit()

    cur = await db.execute("SELECT * FROM grocery_items WHERE grocery_id = ?", [grocery_id])
    row = await cur.fetchone()
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
    db: aiosqlite.Connection = Depends(db_dependency),
):
    cur = await db.execute("SELECT * FROM grocery_items WHERE grocery_id = ?", [grocery_id])
    if await cur.fetchone() is None:
        raise HTTPException(status_code=404, detail="Grocery item not found")

    updates = body.model_dump(exclude_none=True)
    if not updates:
        raise HTTPException(status_code=422, detail="No fields to update")

    set_clause = ", ".join(f"{k} = ?" for k in updates)
    values = list(updates.values()) + [grocery_id]
    await db.execute(f"UPDATE grocery_items SET {set_clause} WHERE grocery_id = ?", values)
    await db.commit()

    cur = await db.execute("SELECT * FROM grocery_items WHERE grocery_id = ?", [grocery_id])
    row = await cur.fetchone()
    item = GroceryItemRead.from_row(row)

    now = datetime.now(tz=timezone.utc).isoformat()
    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": item.model_dump(),
    })
    return item


# Must be defined BEFORE /{grocery_id} to avoid path conflict
@router.delete("/checked", status_code=status.HTTP_204_NO_CONTENT)
async def clear_checked(db: aiosqlite.Connection = Depends(db_dependency)):
    """Delete all checked grocery items."""
    await db.execute("DELETE FROM grocery_items WHERE checked = 1")
    await db.commit()
    now = datetime.now(tz=timezone.utc).isoformat()
    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": {"cleared_checked": True},
    })


@router.delete("/{grocery_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_grocery(
    grocery_id: str,
    db: aiosqlite.Connection = Depends(db_dependency),
):
    cur = await db.execute("SELECT grocery_id FROM grocery_items WHERE grocery_id = ?", [grocery_id])
    if await cur.fetchone() is None:
        raise HTTPException(status_code=404, detail="Grocery item not found")

    await db.execute("DELETE FROM grocery_items WHERE grocery_id = ?", [grocery_id])
    await db.commit()

    now = datetime.now(tz=timezone.utc).isoformat()
    await manager.broadcast({
        "event": "GROCERY_UPDATED",
        "timestamp": now,
        "data": {"grocery_id": grocery_id, "deleted": True},
    })
