from __future__ import annotations
from typing import Optional

from fastapi import APIRouter, Depends
import asyncpg

from core.database import db_dependency
from models.alerts import AlertRead

router = APIRouter(prefix="/alerts", tags=["alerts"])


@router.get("", response_model=list[AlertRead])
async def list_alerts(
    since: Optional[str] = None,
    limit: int = 100,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    query = "SELECT * FROM alerts"
    params: list = []

    if since:
        params.append(since)
        query += f" WHERE created_at >= ${len(params)}"

    params.append(limit)
    query += f" ORDER BY created_at DESC LIMIT ${len(params)}"

    rows = await conn.fetch(query, *params)
    return [AlertRead.from_row(r) for r in rows]
