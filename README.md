# FridgeAI

A real-time food waste reduction system for smart fridges. Tracks pantry inventory, predicts spoilage using a logistic regression model (ASLIE), prioritises items to use first (FAPF), and suggests recipes based on what you have.

---

## Features

- **Inventory** — Add items manually, via barcode scan, or via Grounding DINO camera detection
- **Spoilage scoring** — ASLIE model predicts spoilage probability (P_spoil) and remaining shelf life (RSL) per item
- **Alerts** — Real-time toast notifications when items hit critical/warning thresholds
- **Analytics** — FAPF priority table and 7-day spoilage forecast chart
- **Recipes** — Suggests recipes from TheMealDB based on current pantry contents; marks items as consumed when you cook
- **Grocery list** — Manual list with check-off, auto-populated from restock suggestions
- **Restock suggestions** — Flags items expiring within 2 days or running low
- **Receipt scanning** — OCR a photo of a grocery receipt to bulk-add items (requires Tesseract)

---

## Requirements

- Python 3.10+
- Node.js 18+
- (Optional) Tesseract OCR — for receipt scanning
- (Optional) CUDA-capable GPU — for faster Grounding DINO vision scans

---

## Setup

### Backend

```bash
cd fridgeai-backend
pip install -r requirements.txt
```

Create a `.env` file in `fridgeai-backend/`:

```
DB_PATH=db/fridgeai.sqlite
SETTLE_DELAY_SECONDS=1800
```

Set `SETTLE_DELAY_SECONDS=5` for fast testing (skips the 30-minute settle timer).

Start the server:

```bash
py -m uvicorn main:app --reload --port 8000
```

### Frontend

```bash
cd fridgeai-frontend
npm install
npm run dev
```

Opens at [http://localhost:5173](http://localhost:5173). The Vite dev server proxies all API calls to the backend at port 8000.

---

## Using the App

### Inventory tab

- **Add Item** — manually enter a name, category, quantity, cost, and storage conditions
- **Scan** — opens the camera modal with two modes:
  - *Detect Items* — uses Grounding DINO to identify food items in frame; press Space to capture
  - *Scan Barcode* — uses BarcodeDetector (or QuaggaJS fallback) to look up items by barcode
- Items display a risk bar, spoilage probability, and remaining shelf life once scored (after the settle delay)

### Alerts tab

Scrollable log of all fired alerts — critical (P_spoil > 0.80), warning (P_spoil > 0.50), and use-today (RSL < 1 day).

### Analytics tab

- **Priority table** — items ranked by FAPF score (higher = use first)
- **Spoilage forecast** — 7-day SVG chart showing predicted spoilage curve
- **Restock suggestions** — items flagged as urgent (expiring soon) or low stock

### Recipes tab

Fetches recipe suggestions from TheMealDB based on your current pantry items. Click **Show details** to see ingredients and instructions. Click **Cook This Recipe** to automatically decrement matched pantry items by 1.

### Grocery List tab

- Add items manually with name, category, and quantity
- Check off items as you shop
- **Clear all checked** removes completed items
- Items added from restock suggestions are tagged with their source

---

## Receipt Scanning

Requires Tesseract OCR installed on your system:

- Windows: [https://github.com/UB-Mannheim/tesseract/wiki](https://github.com/UB-Mannheim/tesseract/wiki)
- Then: `pip install pytesseract`

Take a clear photo of a grocery receipt and upload it via the receipt button in the Inventory view. The backend extracts food item names, infers categories, and returns them for you to confirm before adding.

---

## Running Tests

```bash
cd fridgeai-backend
pytest tests/ -v
```

26 tests covering ASLIE scoring, FAPF ranking, items API, WebSocket events, and the settle timer.

---

## Architecture

```
fridgeai-backend/
├── main.py                  # FastAPI app entry point
├── core/                    # Config and database setup
├── models/                  # Pydantic schemas + MobileNetV3 weights
├── services/                # ASLIE, FAPF, scoring orchestration, settle timer
├── routers/                 # REST endpoints (items, alerts, grocery, recipes, restock, receipt, vision)
├── websocket/               # WebSocket connection manager and router
└── scripts/                 # ASLIE coefficient fitting script

fridgeai-frontend/
├── src/
│   ├── App.jsx              # Global state (useReducer) + WebSocket setup
│   ├── api.js               # REST helpers + WS singleton
│   ├── views/               # Inventory, Alerts, Analytics, Recipes, GroceryList
│   └── components/          # ItemCard, AddItemModal, ScanModal, AlertBanner, ReceiptModal
└── vite.config.js           # Dev proxy to backend
```

### ASLIE Spoilage Model

Logistic regression predicting spoilage probability from time elapsed, temperature, humidity, and food category:

```
P_spoil = sigmoid(β₀ + β₁·t + β₂·T_n + β₃·C_n + β₄·H_n)
```

Fitted on the Mendeley Multi-Parameter Fruit Spoilage IoT Dataset (10,995 readings, 79% accuracy, ROC-AUC 0.86).

### FAPF Priority Score

```
S(i) = 0.5·P_spoil + 0.3·Cost_norm − 0.2·P_consume
```

Higher score = use this item first.
