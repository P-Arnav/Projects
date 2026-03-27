"""
conftest.py — runs before any test module is imported.

Sets DATABASE_URL from the environment (or a test Supabase URL) so asyncpg
can connect. Clears all tables before each test for isolation.
"""

import os

os.environ.setdefault("DATABASE_URL", os.getenv("TEST_DATABASE_URL", os.getenv("DATABASE_URL", "")))
os.environ["SETTLE_DELAY_SECONDS"] = "9999"  # prevent auto-scoring in CRUD tests

import pytest
from core.database import init_db, close_db, get_db


@pytest.fixture(autouse=True)
async def clean_db():
    """Ensure tables exist and are empty before each test."""
    await init_db()
    async with get_db() as conn:
        await conn.execute("DELETE FROM items")
        await conn.execute("DELETE FROM alerts")
    yield
    await close_db()
