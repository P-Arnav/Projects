from fastapi import APIRouter, Depends
import asyncpg

from core.database import db_dependency
from websocket.manager import manager
from services.settle_timer import pending_count

router = APIRouter(prefix="/status", tags=["status"])


@router.get("")
async def get_status(conn: asyncpg.Connection = Depends(db_dependency)):
    item_count = await conn.fetchval("SELECT COUNT(*) FROM items")

    return {
        "status": "ok",
        "ws_clients": manager.client_count,
        "item_count": item_count,
        "pending_timers": pending_count(),
    }
