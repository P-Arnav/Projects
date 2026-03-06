from fastapi import APIRouter, Depends
import aiosqlite

from core.database import db_dependency
from websocket.manager import manager
from services.settle_timer import pending_count

router = APIRouter(prefix="/status", tags=["status"])


@router.get("")
async def get_status(db: aiosqlite.Connection = Depends(db_dependency)):
    cur = await db.execute("SELECT COUNT(*) FROM items")
    row = await cur.fetchone()
    item_count = row[0]

    return {
        "status": "ok",
        "ws_clients": manager.client_count,
        "item_count": item_count,
        "pending_timers": pending_count(),
    }
