from __future__ import annotations
from typing import Optional

from fastapi import APIRouter, Depends
import aiosqlite

from core.database import db_dependency
from models.alerts import AlertRead

router = APIRouter(prefix="/alerts", tags=["alerts"])


@router.get("", response_model=list[AlertRead])
async def list_alerts(
    since: Optional[str] = None,
    limit: int = 100,
    db: aiosqlite.Connection = Depends(db_dependency),
):
    query = "SELECT * FROM alerts"
    params: list = []

    if since:
        query += " WHERE created_at >= ?"
        params.append(since)

    query += " ORDER BY created_at DESC LIMIT ?"
    params.append(limit)

    cur = await db.execute(query, params)
    rows = await cur.fetchall()
    return [AlertRead.from_row(r) for r in rows]
