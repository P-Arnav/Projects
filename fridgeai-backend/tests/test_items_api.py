import pytest
import pytest_asyncio
from httpx import AsyncClient, ASGITransport

from main import app


@pytest_asyncio.fixture
async def client():
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c


async def test_create_item(client):
    resp = await client.post("/items", json={
        "name": "Milk",
        "category": "dairy",
        "shelf_life": 7,
        "quantity": 2,
    })
    assert resp.status_code == 201
    data = resp.json()
    assert data["name"] == "Milk"
    assert data["category"] == "dairy"
    assert data["confidence_tier"] == "LOW"
    assert data["P_spoil"] is None


async def test_get_item(client):
    resp = await client.post("/items", json={"name": "Eggs", "category": "protein", "shelf_life": 21})
    item_id = resp.json()["item_id"]

    resp = await client.get(f"/items/{item_id}")
    assert resp.status_code == 200
    assert resp.json()["item_id"] == item_id


async def test_list_items(client):
    await client.post("/items", json={"name": "Chicken", "category": "meat", "shelf_life": 3})
    resp = await client.get("/items")
    assert resp.status_code == 200
    assert len(resp.json()) >= 1


async def test_list_items_category_filter(client):
    await client.post("/items", json={"name": "Salmon", "category": "fish", "shelf_life": 2})
    resp = await client.get("/items?category=fish")
    assert resp.status_code == 200
    for item in resp.json():
        assert item["category"] == "fish"


async def test_patch_item(client):
    resp = await client.post("/items", json={"name": "Cheese", "category": "dairy", "shelf_life": 14})
    item_id = resp.json()["item_id"]

    resp = await client.patch(f"/items/{item_id}", json={"quantity": 3})
    assert resp.status_code == 200
    assert resp.json()["quantity"] == 3


async def test_delete_item(client):
    resp = await client.post("/items", json={"name": "Broccoli", "category": "vegetable", "shelf_life": 5})
    item_id = resp.json()["item_id"]

    resp = await client.delete(f"/items/{item_id}")
    assert resp.status_code == 204

    resp = await client.get(f"/items/{item_id}")
    assert resp.status_code == 404


async def test_get_nonexistent_item(client):
    resp = await client.get("/items/does-not-exist")
    assert resp.status_code == 404


async def test_status_endpoint(client):
    resp = await client.get("/status")
    assert resp.status_code == 200
    data = resp.json()
    assert data["status"] == "ok"
    assert "item_count" in data
    assert "ws_clients" in data
