import contextlib
import aiosqlite
from core.config import DB_PATH

CREATE_ITEMS = """
CREATE TABLE IF NOT EXISTS items (
    item_id        TEXT PRIMARY KEY,
    name           TEXT NOT NULL,
    category       TEXT NOT NULL,
    quantity       INTEGER NOT NULL DEFAULT 1,
    entry_time     TEXT NOT NULL,
    shelf_life     INTEGER NOT NULL,
    location       TEXT NOT NULL DEFAULT '',
    estimated_cost REAL NOT NULL DEFAULT 0.0,
    storage_temp   REAL NOT NULL DEFAULT 4.0,
    humidity       REAL NOT NULL DEFAULT 50.0,
    P_spoil        REAL,
    RSL            REAL,
    fapf_score     REAL,
    confidence_tier TEXT NOT NULL DEFAULT 'LOW',
    updated_at     TEXT NOT NULL
)
"""

CREATE_ALERTS = """
CREATE TABLE IF NOT EXISTS alerts (
    alert_id   TEXT PRIMARY KEY,
    item_id    TEXT NOT NULL,
    item_name  TEXT NOT NULL,
    alert_type TEXT NOT NULL,
    P_spoil    REAL,
    RSL        REAL,
    message    TEXT NOT NULL,
    created_at TEXT NOT NULL
)
"""


async def init_db() -> None:
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(CREATE_ITEMS)
        await db.execute(CREATE_ALERTS)
        await db.commit()


@contextlib.asynccontextmanager
async def get_db():
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        yield db


async def db_dependency():
    """FastAPI dependency — yields an open aiosqlite connection."""
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        yield db
