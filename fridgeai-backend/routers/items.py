from __future__ import annotations
from datetime import datetime, timezone
from typing import Optional
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, status
import asyncpg

from core.database import db_dependency
from models.item import ItemCreate, ItemRead, ItemUpdate
from models.feedback import FeedbackCreate, FeedbackRead
from services import settle_timer
from websocket.manager import manager

router = APIRouter(prefix="/items", tags=["items"])

# Fields whose change warrants rescoring
_SCORE_FIELDS = {"shelf_life", "storage_temp", "humidity"}


@router.post("", response_model=ItemRead, status_code=status.HTTP_201_CREATED)
async def create_item(body: ItemCreate, conn: asyncpg.Connection = Depends(db_dependency)):
    item_id = str(uuid4())
    now = datetime.now(tz=timezone.utc).isoformat()

    await conn.execute(
        """INSERT INTO items
           (item_id, name, category, quantity, entry_time, shelf_life,
            location, estimated_cost, storage_temp, humidity,
            confidence_tier, updated_at)
           VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,'LOW',$11)""",
        item_id, body.name, body.category, body.quantity, now,
        body.shelf_life, body.location, body.estimated_cost,
        body.storage_temp, body.humidity, now,
    )

    row = await conn.fetchrow("SELECT * FROM items WHERE item_id = $1", item_id)
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
    conn: asyncpg.Connection = Depends(db_dependency),
):
    query = "SELECT * FROM items WHERE 1=1"
    params: list = []

    if category:
        params.append(category)
        query += f" AND category = ${len(params)}"
    if updated_since:
        params.append(updated_since)
        query += f" AND updated_at >= ${len(params)}"

    rows = await conn.fetch(query, *params)
    return [ItemRead.from_row(r) for r in rows]


@router.get("/{item_id}", response_model=ItemRead)
async def get_item(item_id: str, conn: asyncpg.Connection = Depends(db_dependency)):
    row = await conn.fetchrow("SELECT * FROM items WHERE item_id = $1", item_id)
    if row is None:
        raise HTTPException(status_code=404, detail="Item not found")
    return ItemRead.from_row(row)


@router.patch("/{item_id}", response_model=ItemRead)
async def update_item(
    item_id: str,
    body: ItemUpdate,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    existing = await conn.fetchrow("SELECT item_id FROM items WHERE item_id = $1", item_id)
    if existing is None:
        raise HTTPException(status_code=404, detail="Item not found")

    updates = body.model_dump(exclude_none=True)
    if not updates:
        raise HTTPException(status_code=422, detail="No fields to update")

    now = datetime.now(tz=timezone.utc).isoformat()
    updates["updated_at"] = now

    keys = list(updates.keys())
    vals = list(updates.values())
    set_clause = ", ".join(f"{k} = ${i+1}" for i, k in enumerate(keys))
    await conn.execute(
        f"UPDATE items SET {set_clause} WHERE item_id = ${len(keys)+1}",
        *vals, item_id,
    )

    # Rescore if any score-relevant field changed
    if _SCORE_FIELDS & set(body.model_dump(exclude_none=True)):
        settle_timer.schedule(item_id)

    row = await conn.fetchrow("SELECT * FROM items WHERE item_id = $1", item_id)
    item = ItemRead.from_row(row)

    await manager.broadcast({
        "event": "ITEM_UPDATED",
        "timestamp": now,
        "data": {"item_id": item_id, "changed_fields": body.model_dump(exclude_none=True)},
    })

    return item


@router.post("/{item_id}/feedback", response_model=FeedbackRead, status_code=status.HTTP_201_CREATED)
async def submit_feedback(
    item_id: str,
    body: FeedbackCreate,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    row = await conn.fetchrow("SELECT * FROM items WHERE item_id = $1", item_id)
    if row is None:
        raise HTTPException(status_code=404, detail="Item not found")

    declared = row["shelf_life"]

    if body.shelf_life_actual is not None:
        actual = body.shelf_life_actual
    else:
        # still_good=True: item has lasted at least elapsed_days + 1
        entry_time = datetime.fromisoformat(row["entry_time"])
        elapsed = (datetime.now(tz=timezone.utc) - entry_time.replace(tzinfo=timezone.utc)).total_seconds() / 86400.0
        actual = max(float(declared), elapsed + 1.0)

    correction = actual - declared

    feedback_id = str(uuid4())
    now = datetime.now(tz=timezone.utc).isoformat()
    await conn.execute(
        """INSERT INTO feedback
           (feedback_id, item_id, category, shelf_life_declared, shelf_life_actual, correction, created_at)
           VALUES ($1,$2,$3,$4,$5,$6,$7)""",
        feedback_id, item_id, row["category"], declared, actual, correction, now,
    )

    # Rescore item immediately with the updated correction applied
    settle_timer.schedule(item_id)

    return FeedbackRead(
        feedback_id=feedback_id,
        item_id=item_id,
        category=row["category"],
        shelf_life_declared=declared,
        shelf_life_actual=actual,
        correction=correction,
        created_at=now,
    )


@router.delete("/{item_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_item(
    item_id: str,
    reason: str = "consumed",
    conn: asyncpg.Connection = Depends(db_dependency),
):
    row = await conn.fetchrow("SELECT * FROM items WHERE item_id = $1", item_id)
    if row is None:
        raise HTTPException(status_code=404, detail="Item not found")

    settle_timer.cancel(item_id)

    # Determine effective reason for history
    effective_reason = reason
    if reason == "consumed" and row["p_spoil"] is not None and row["p_spoil"] > 0.8:
        effective_reason = "wasted"

    # Record consumption history
    hist_id = str(uuid4())
    now = datetime.now(tz=timezone.utc).isoformat()
    await conn.execute(
        """INSERT INTO consumption_history
           (id, item_id, item_name, category, quantity_consumed, reason, p_spoil_at_removal, consumed_at)
           VALUES ($1,$2,$3,$4,$5,$6,$7,$8)""",
        hist_id, item_id, row["name"], row["category"], row["quantity"],
        effective_reason, row["p_spoil"], now,
    )

    await conn.execute("DELETE FROM items WHERE item_id = $1", item_id)

    await manager.broadcast({
        "event": "ITEM_DELETED",
        "timestamp": now,
        "data": {"item_id": item_id, "reason": reason},
    })
