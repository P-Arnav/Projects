"""
conftest.py — runs before any test module is imported.

Sets DB_PATH to a temp file (not :memory:) so a single aiosqlite connection
can share state between init_db() and request handlers.
Also clears all tables before each test for isolation.
"""

import os
import atexit
import tempfile

# Must be set before any project import so core.config picks it up
_fd, _db_path = tempfile.mkstemp(suffix=".sqlite")
os.close(_fd)
atexit.register(os.unlink, _db_path)

os.environ["DB_PATH"] = _db_path
os.environ["SETTLE_DELAY_SECONDS"] = "9999"  # prevent auto-scoring in CRUD tests

import pytest
from core.database import init_db, get_db


@pytest.fixture(autouse=True)
async def clean_db():
    """Ensure tables exist and are empty before each test."""
    await init_db()
    async with get_db() as db:
        await db.execute("DELETE FROM items")
        await db.execute("DELETE FROM alerts")
        await db.commit()
    yield
