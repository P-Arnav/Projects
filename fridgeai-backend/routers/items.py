from __future__ import annotations
from datetime import datetime, timezone
from typing import Optional
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, status
import aiosqlite

from core.database import db_dependency
from models.item import ItemCreate, ItemRead, ItemUpdate
from services import settle_timer
from websocket.manager import manager

router = APIRouter(prefix="/items", tags=["items"])

# Fields whose change warrants rescoring
_SCORE_FIELDS = {"shelf_life", "storage_temp", "humidity"}


@router.post("", response_model=ItemRead, status_code=status.HTTP_201_CREATED)
async def create_item(body: ItemCreate, db: aiosqlite.Connection = Depends(db_dependency)):
    item_id = str(uuid4())
    now = datetime.now(tz=timezone.utc).isoformat()

    await db.execute(
        """INSERT INTO items
           (item_id, name, category, quantity, entry_time, shelf_life,
            location, estimated_cost, storage_temp, humidity,
            confidence_tier, updated_at)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 'LOW', ?)""",
        [
            item_id, body.name, body.category, body.quantity, now,
            body.shelf_life, body.location, body.estimated_cost,
            body.storage_temp, body.humidity, now,
        ],
    )
    await db.commit()

    cur = await db.execute("SELECT * FROM items WHERE item_id = ?", [item_id])
    row = await cur.fetchone()
    item = ItemRead.from_row(row)

    settle_timer.schedule(item_id)

    await manager.broadcast({
        "event": "ITEM_INSERTED",
        "timestamp": now,
        "data": item.model_dump(),
    })

    return item


@router.get("", response_model=list[ItemRead])
async def list_items(
    category: Optional[str] = None,
    updated_since: Optional[str] = None,
    db: aiosqlite.Connection = Depends(db_dependency),
):
    query = "SELECT * FROM items WHERE 1=1"
    params: list = []

    if category:
        query += " AND category = ?"
        params.append(category)
    if updated_since:
        query += " AND updated_at >= ?"
        params.append(updated_since)

    cur = await db.execute(query, params)
    rows = await cur.fetchall()
    return [ItemRead.from_row(r) for r in rows]


@router.get("/{item_id}", response_model=ItemRead)
async def get_item(item_id: str, db: aiosqlite.Connection = Depends(db_dependency)):
    cur = await db.execute("SELECT * FROM items WHERE item_id = ?", [item_id])
    row = await cur.fetchone()
    if row is None:
        raise HTTPException(status_code=404, detail="Item not found")
    return ItemRead.from_row(row)


@router.patch("/{item_id}", response_model=ItemRead)
async def update_item(
    item_id: str,
    body: ItemUpdate,
    db: aiosqlite.Connection = Depends(db_dependency),
):
    cur = await db.execute("SELECT * FROM items WHERE item_id = ?", [item_id])
    if await cur.fetchone() is None:
        raise HTTPException(status_code=404, detail="Item not found")

    updates = body.model_dump(exclude_none=True)
    if not updates:
        raise HTTPException(status_code=422, detail="No fields to update")

    now = datetime.now(tz=timezone.utc).isoformat()
    updates["updated_at"] = now

    set_clause = ", ".join(f"{k} = ?" for k in updates)
    values = list(updates.values()) + [item_id]
    await db.execute(f"UPDATE items SET {set_clause} WHERE item_id = ?", values)
    await db.commit()

    # Rescore if any score-relevant field changed
    if _SCORE_FIELDS & set(body.model_dump(exclude_none=True)):
        settle_timer.schedule(item_id)

    cur = await db.execute("SELECT * FROM items WHERE item_id = ?", [item_id])
    row = await cur.fetchone()
    item = ItemRead.from_row(row)

    await manager.broadcast({
        "event": "ITEM_UPDATED",
        "timestamp": now,
        "data": {"item_id": item_id, "changed_fields": body.model_dump(exclude_none=True)},
    })

    return item


@router.delete("/{item_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_item(
    item_id: str,
    reason: str = "consumed",
    db: aiosqlite.Connection = Depends(db_dependency),
):
    cur = await db.execute("SELECT item_id FROM items WHERE item_id = ?", [item_id])
    if await cur.fetchone() is None:
        raise HTTPException(status_code=404, detail="Item not found")

    settle_timer.cancel(item_id)

    await db.execute("DELETE FROM items WHERE item_id = ?", [item_id])
    await db.commit()

    await manager.broadcast({
        "event": "ITEM_DELETED",
        "timestamp": datetime.now(tz=timezone.utc).isoformat(),
        "data": {"item_id": item_id, "reason": reason},
    })
