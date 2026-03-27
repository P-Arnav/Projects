import sys
from fastapi.testclient import TestClient
from main import app
from io import BytesIO
from PIL import Image

def test_routes():
    client = TestClient(app)

    print("Testing /ocr/scan endpoint...")
    img = Image.new('RGB', (100, 100))
    buffer = BytesIO()
    img.save(buffer, format='JPEG')
    buffer.seek(0)
    
    response = client.post("/ocr/scan", files={"file": ("test.jpg", buffer, "image/jpeg")})
    print("OCR Scan Status:", response.status_code)
    print("OCR Scan Response:", response.json())
    assert response.status_code == 200
    
    print("\nTesting /items endpoint...")
    item_res = client.post("/items", json={
        "name": "milk", "category": "Dairy", "quantity": 1, 
        "shelf_life": 5, "estimated_cost": 2.0, "storage_temp": 4, "humidity": 50, "location": ""
    })
    print("Items Add Status:", item_res.status_code)
    
    print("\nTesting /recipes endpoint...")
    resp_recipes = client.get("/recipes")
    print("Recipes Status:", resp_recipes.status_code)
    try:
        data = resp_recipes.json()
        print("Recipes List Length:", len(data) if isinstance(data, list) else data.get("error"))
    except Exception as e:
        print("Cannot parse json:", e)

if __name__ == "__main__":
    test_routes()
