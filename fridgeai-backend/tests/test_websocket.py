import pytest
import pytest_asyncio
from httpx import AsyncClient, ASGITransport
from fastapi.testclient import TestClient

from main import app


async def test_broadcast_item_inserted():
    """ITEM_INSERTED should be broadcast when POST /items is called."""
    received: list[dict] = []

    with TestClient(app) as sync_client:
        with sync_client.websocket_connect("/ws") as ws:
            ws.receive_json()  # WELCOME

            sync_client.post("/items", json={
                "name": "Yogurt",
                "category": "dairy",
                "shelf_life": 10,
            })

            msg = ws.receive_json()
            received.append(msg)

    assert any(m["event"] == "ITEM_INSERTED" for m in received)
    inserted = next(m for m in received if m["event"] == "ITEM_INSERTED")
    assert inserted["data"]["name"] == "Yogurt"


async def test_broadcast_item_deleted():
    received: list[dict] = []

    with TestClient(app) as sync_client:
        with sync_client.websocket_connect("/ws") as ws:
            ws.receive_json()  # WELCOME

            resp = sync_client.post("/items", json={
                "name": "Tofu",
                "category": "protein",
                "shelf_life": 5,
            })
            item_id = resp.json()["item_id"]
            ws.receive_json()  # ITEM_INSERTED

            sync_client.delete(f"/items/{item_id}?reason=consumed")
            msg = ws.receive_json()
            received.append(msg)

    assert received[0]["event"] == "ITEM_DELETED"
    assert received[0]["data"]["item_id"] == item_id
    assert received[0]["data"]["reason"] == "consumed"


async def test_welcome_message():
    with TestClient(app) as sync_client:
        with sync_client.websocket_connect("/ws?client_type=web") as ws:
            msg = ws.receive_json()
            assert msg["event"] == "WELCOME"
            assert "connected_clients" in msg["data"]
