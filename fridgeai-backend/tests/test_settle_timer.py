"""
Settle timer integration tests.

Uses AsyncClient + ASGITransport so settle tasks run in the SAME event loop
as the test — asyncio.sleep() yields to them naturally.
"""

import asyncio
import pytest
import pytest_asyncio
from datetime import datetime, timezone, timedelta
from httpx import AsyncClient, ASGITransport

import services.settle_timer as st
from main import app
from core.database import get_db
from services.scorer import run_for_item
from websocket.manager import manager


@pytest_asyncio.fixture
async def client():
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c


@pytest_asyncio.fixture(autouse=True)
async def reset_timers():
    """Cancel leftover tasks between tests."""
    yield
    for task in list(st._pending.values()):
        task.cancel()
    st._pending.clear()


async def test_timer_fires_and_scores(client, monkeypatch):
    """After delay elapses, P_spoil/RSL/fapf_score should be populated."""
    monkeypatch.setattr(st, "SETTLE_DELAY_SECONDS", 1)

    resp = await client.post("/items", json={
        "name": "Milk",
        "category": "dairy",
        "shelf_life": 7,
        "storage_temp": 4.0,
        "humidity": 50.0,
    })
    assert resp.status_code == 201
    item_id = resp.json()["item_id"]

    # Wait for the 1-second timer + small buffer
    await asyncio.sleep(2.5)

    resp = await client.get(f"/items/{item_id}")
    data = resp.json()
    assert data["P_spoil"] is not None, "P_spoil should be set after settle"
    assert data["RSL"] is not None
    assert data["fapf_score"] is not None


async def test_timer_cancelled_on_delete(client, monkeypatch):
    """Deleting an item before timer fires should produce no ITEM_SCORED event."""
    monkeypatch.setattr(st, "SETTLE_DELAY_SECONDS", 2)

    scored_ids: list[str] = []
    original_broadcast = manager.broadcast

    async def capture(msg):
        if msg.get("event") == "ITEM_SCORED":
            scored_ids.append(msg["data"]["item_id"])
        await original_broadcast(msg)

    monkeypatch.setattr(manager, "broadcast", capture)

    resp = await client.post("/items", json={
        "name": "Chicken",
        "category": "meat",
        "shelf_life": 3,
    })
    item_id = resp.json()["item_id"]

    # Delete immediately — before the 2-second timer fires
    await client.delete(f"/items/{item_id}")

    # Wait past what the timer would have fired
    await asyncio.sleep(3.5)

    assert item_id not in scored_ids, "Deleted item should not have been scored"


async def test_critical_alert_fired(client, monkeypatch):
    """Manually back-date entry_time so ASLIE sees a very old item → CRITICAL_ALERT."""
    alerts_received: list[dict] = []
    original_broadcast = manager.broadcast

    async def capture(msg):
        if msg.get("event") == "ALERT_FIRED":
            alerts_received.append(msg["data"])
        await original_broadcast(msg)

    monkeypatch.setattr(manager, "broadcast", capture)

    resp = await client.post("/items", json={
        "name": "Old Fish",
        "category": "fish",
        "shelf_life": 1,
        "storage_temp": 25.0,
        "humidity": 90.0,
    })
    item_id = resp.json()["item_id"]

    # Backdate so ASLIE sees 10 days elapsed → P_spoil >> 0.80
    past = (datetime.now(tz=timezone.utc) - timedelta(days=10)).isoformat()
    async with get_db() as conn:
        await conn.execute("UPDATE items SET entry_time=$1 WHERE item_id=$2", past, item_id)

    await run_for_item(item_id)

    critical = [a for a in alerts_received if a["alert_type"] == "CRITICAL_ALERT"]
    assert len(critical) >= 1, f"Expected CRITICAL_ALERT, got: {alerts_received}"
    assert critical[0]["item_id"] == item_id
