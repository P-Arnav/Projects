# FridgeAI — Project Context for Claude

## Project Overview
FridgeAI is a real-time food waste reduction system for smart fridges. It consists of:
- A FastAPI backend with ASLIE scoring, FAPF ranking, WebSocket sync, and vision endpoints
- A React + Vite frontend dashboard (inventory, alerts, analytics)
- A Grounding DINO + MobileNetV3 vision pipeline for item detection and spoilage classification
- A standalone webcam test/detection script

## Repository Structure

```
C:\Users\cloro\anthro_tester\
├── CLAUDE.md                        # this file
├── fridgeai-backend/                # FastAPI backend
├── fridgeai-frontend/               # React + Vite frontend
├── grounding_dino_test.py           # standalone Grounding DINO test script
├── check_webcam.py                  # webcam connectivity test
├── fridge_model.py                  # Blender 3D fridge model script
└── generate_report.py               # Word document generator (python-docx)
```

## Running the Project

```bash
# Backend (from fridgeai-backend/)
pip install -r requirements.txt
py -m uvicorn main:app --reload --port 8000

# Frontend (from fridgeai-frontend/)
npm install
npm run dev                          # opens on localhost:5173

# Vision test script
python grounding_dino_test.py        # defaults to webcam, camera index 0
python grounding_dino_test.py --image path/to/image.jpg
```

## Backend — fridgeai-backend/

### Key Files
| File | Purpose |
|------|---------|
| `main.py` | FastAPI app, lifespan (init_db + recover_on_startup), router registration |
| `core/config.py` | All constants: ASLIE coefficients, thresholds, DB path, settle delay |
| `core/database.py` | aiosqlite setup, db_dependency, init_db() |
| `models/item.py` | ItemCreate, ItemRead, ItemUpdate Pydantic models |
| `models/alert.py` | AlertRead Pydantic model |
| `services/aslie.py` | ASLIE engine — P_spoil, RSL computation |
| `services/fapf.py` | FAPF scoring — S(i) = 0.5·P_spoil + 0.3·Cost_norm − 0.2·P_consume |
| `services/scorer.py` | Orchestrates ASLIE + FAPF, writes to DB, broadcasts WS events, fires alerts |
| `services/settle_timer.py` | Per-item asyncio tasks, 30-min settle delay, startup recovery |
| `routers/items.py` | CRUD endpoints for /items |
| `routers/alerts.py` | GET /alerts |
| `routers/status.py` | GET /status |
| `routers/lookup.py` | GET /lookup/shelf-life/{category}, GET /lookup/barcode/{barcode} |
| `routers/vision.py` | POST /vision/scan — Grounding DINO + MobileNetV3 spoilage |
| `websocket/manager.py` | ConnectionManager singleton, broadcast() |
| `websocket/ws_router.py` | WS endpoint /ws?client_type=web |
| `scripts/fit_aslie.py` | Fits ASLIE β coefficients from Mendeley dataset |
| `models/spoilage_mobilenetv3.pth` | Trained MobileNetV3 spoilage classifier weights |

### ASLIE Model
```
P_spoil(i, t) = sigmoid(β₀ + β₁·t + β₂·T_n + β₃·C_n + β₄·H_n)

β₀ = -37.9506  (intercept, fitted from Mendeley dataset)
β₁ =   3.40    (time in days, recalibrated heuristic)
β₂ =  17.0408  (temperature, normalised)
β₃ =  -0.0282  (category encoding, normalised)
β₄ =  25.9930  (humidity, normalised)
θ  =   0.75    (spoilage decision threshold)

Normalisation ranges:
  TEMP_NORM  = (0.0, 30.0)   degC
  HUMID_NORM = (0.0, 100.0)  %
  CAT_NORM   = (1.0,  8.0)   ordinal

RSL = min(binary_search(P_spoil >= θ), shelf_life - t_elapsed)
```

### Category Encodings
```
dairy=1, protein=2, meat=3, vegetable=4, fruit=5, fish=6, cooked=7, beverage=8
```

### Default Shelf Lives (days)
```
dairy=7, protein=7, meat=3, vegetable=6, fruit=7, fish=2, cooked=4, beverage=7
```

### Alert Thresholds
```
ALERT_CRITICAL  = 0.80  (P_spoil)
ALERT_WARNING   = 0.50  (P_spoil)
ALERT_USE_TODAY = 1.0   (RSL in days)
SETTLE_DELAY    = 1800  seconds (30 min), override with SETTLE_DELAY_SECONDS env var
```

### WebSocket Events
| Event | Payload |
|-------|---------|
| `ITEM_INSERTED` | Full ItemRead dict |
| `ITEM_SCORED` | item_id, P_spoil, RSL, fapf_score, confidence_tier |
| `ITEM_UPDATED` | item_id, changed_fields dict |
| `ITEM_DELETED` | item_id, reason |
| `ALERT_FIRED` | AlertRead dict |

### Vision Endpoint
- `POST /vision/scan` — accepts multipart JPEG upload
- Runs Grounding DINO base model (lazy-loaded, cached in app.state)
- Crops each bounding box, runs MobileNetV3 spoilage classifier
- Returns `ScanResult { items: DetectedItem[], capture_id }`
- `DetectedItem` has: name, category, shelf_life, confidence, count, spoilage_detected, spoilage_confidence
- Spoilage threshold: 0.5
- Model path: `models/spoilage_mobilenetv3.pth`
- Grounding DINO model: `IDEA-Research/grounding-dino-base`

## Frontend — fridgeai-frontend/

### Key Files
| File | Purpose |
|------|---------|
| `src/App.jsx` | useReducer global state, WS setup, nav tabs |
| `src/api.js` | REST helpers + WebSocket singleton with auto-reconnect |
| `src/constants.js` | CSS colour palette, CATEGORIES list, riskColor/riskLabel helpers |
| `src/views/Inventory.jsx` | Item grid, category filter, Scan/Add buttons |
| `src/views/Alerts.jsx` | Scrollable alert log |
| `src/views/Analytics.jsx` | FAPF priority table + 7-day SVG spoilage forecast |
| `src/components/ItemCard.jsx` | Risk bar, RSL countdown, P_spoil, tier badge |
| `src/components/AddItemModal.jsx` | Manual entry form + barcode lookup + shelf-life defaults |
| `src/components/ScanModal.jsx` | Grounding DINO camera scan + QuaggaJS barcode scanning |
| `src/components/AlertBanner.jsx` | Top-right toast stack, auto-dismiss 5s |
| `vite.config.js` | Proxy: /items /alerts /status /lookup /vision /ws → localhost:8000 |

### Colour Palette
```js
bg:       '#070d1a'    surface:  '#0c1628'    surface2: '#101e33'
border:   '#1a2e4a'    border2:  '#243d5c'
teal:     '#00d4aa'    blue:     '#3b9eff'    text:     '#e8f0fe'
muted:    '#4a6080'    critical: '#ff4d6d'    warn:     '#fbbf24'    safe: '#34d399'
```

### Risk Tiers
```
P_spoil > 0.80  → CRITICAL (red)
P_spoil > 0.50  → USE SOON (yellow)
P_spoil ≤ 0.50  → SAFE (green)
P_spoil = null  → PENDING (muted)
```

### Global State (useReducer)
```js
{ items, alerts, toasts, wsStatus, view }

WS event → reducer action:
  ITEM_INSERTED → ADD_ITEM
  ITEM_SCORED   → UPDATE_ITEM
  ITEM_UPDATED  → UPDATE_ITEM
  ITEM_DELETED  → REMOVE_ITEM
  ALERT_FIRED   → ADD_ALERT + ADD_TOAST
```

### ScanModal — Camera Setup
- External USB camera: index 0 (USB Camera 0c45:6366)
- Laptop camera:       index 1 (HP True Vision FHD)
- OMEN Cam:            index 2
- Resolution: 1280×720 ideal
- Two modes: Detect Items (Grounding DINO) and Scan Barcode (QuaggaJS)
- Barcode: uses native BarcodeDetector if available, falls back to QuaggaJS (@ericblade/quagga2)
- Auto-adds detected items immediately, skips spoiled items

## Vision Scripts

### grounding_dino_test.py
- Model: `IDEA-Research/grounding-dino-base` (cached in ~/.cache/huggingface/hub)
- Camera: index 0 (external USB)
- Threshold: 0.5 minimum
- Controls: SPACE = capture + detect, R = reset session counter, Q = quit
- Session counter overlaid on live feed

### check_webcam.py
- Camera: index 0 (external USB)
- Camera settings: AUTO_EXPOSURE=0.25, EXPOSURE=-8, BRIGHTNESS=80, CONTRAST=50

## Datasets

### Mendeley Multi-Parameter Fruit Spoilage IoT Dataset
- 10,995 readings, features: Temp (21-27°C), Humidity (71-95%), Class: Good/Bad
- Fruits: Banana, Orange, Pineapple, Tomato
- Used to fit ASLIE β₀, β₂, β₃, β₄ via LogisticRegression
- Fitting script: `scripts/fit_aslie.py`
- Performance: 79% accuracy, ROC-AUC 0.86

### Fresh and Rotten Fruits Dataset (Sriram, Kaggle)
- ~13,500 images, 6 classes × 2 labels (fresh/rotten)
- Used to fine-tune MobileNetV3-Small spoilage classifier
- Training: 15 epochs, Adam lr=1e-3, BCEWithLogitsLoss, WeightedRandomSampler
- Performance: 100% accuracy on test set (2,698 images)
- Weights: `models/spoilage_mobilenetv3.pth`

## Dependencies

### Backend (requirements.txt)
```
fastapi, uvicorn, aiosqlite, pydantic, numpy, httpx, websockets,
python-dotenv, python-multipart, pillow
+ torch, transformers (vision — optional, for /vision/scan)
+ torchvision (spoilage classifier)
```

### Frontend (package.json)
```
react, react-dom, vite, @vitejs/plugin-react
@zxing/browser, @zxing/library, @ericblade/quagga2
```

### Vision test scripts
```
torch, torchvision, transformers, pillow, opencv-python
```

### Report generator
```
python-docx
```

## Environment Variables (.env in fridgeai-backend/)
```
DB_PATH=db/fridgeai.sqlite
SETTLE_DELAY_SECONDS=1800    # set to 5 for fast testing
```

## Tests
```bash
cd fridgeai-backend
pytest tests/ -v
```
26 tests covering: ASLIE, FAPF, items API, WebSocket, settle timer.
Test isolation: temp SQLite file via conftest.py + clean_db autouse fixture.
