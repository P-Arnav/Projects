from __future__ import annotations
import asyncio
import base64
import io
import json
import logging
import os
import re
from typing import Optional

import httpx
from fastapi import APIRouter, File, HTTPException, UploadFile
from PIL import Image, ImageEnhance, ImageFilter
from pydantic import BaseModel

from routers.lookup import get_item_shelf_life, get_item_cost

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/receipt", tags=["receipt"])

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")

try:
    import pytesseract
    _TESSERACT_AVAILABLE = True
except ImportError:
    _TESSERACT_AVAILABLE = False

try:
    import easyocr as _easyocr_lib
    import numpy as np
    _EASYOCR_AVAILABLE = True
except ImportError:
    _EASYOCR_AVAILABLE = False

_easyocr_reader = None


def _get_easyocr_reader():
    global _easyocr_reader
    if _easyocr_reader is None:
        _easyocr_reader = _easyocr_lib.Reader(['en'], gpu=False, verbose=False)
    return _easyocr_reader


# ── Category keyword map ─────────────────────────────────────────────────────
_CATEGORY_KEYWORDS: dict[str, list[str]] = {
    "dairy":     ["milk", "cheese", "yogurt", "yoghurt", "butter", "cream", "curd", "paneer", "ghee"],
    "meat":      ["chicken", "beef", "pork", "lamb", "mutton", "sausage", "bacon", "ham", "turkey"],
    "fish":      ["fish", "salmon", "tuna", "shrimp", "prawn", "crab", "lobster", "sardine", "mackerel"],
    "vegetable": ["tomato", "potato", "onion", "carrot", "spinach", "broccoli", "pepper", "lettuce",
                  "cabbage", "capsicum", "beans", "peas", "corn", "cauliflower", "cucumber", "celery",
                  "mushroom", "zucchini", "eggplant", "garlic", "ginger"],
    "fruit":     ["apple", "banana", "orange", "mango", "grape", "strawberry", "lemon", "lime",
                  "pear", "peach", "pineapple", "watermelon", "cherry", "berry", "kiwi", "avocado"],
    "beverage":  ["juice", "water", "soda", "cola", "coffee", "tea", "drink", "beer", "wine",
                  "lemonade", "smoothie"],
    "cooked":    ["bread", "pasta", "rice", "soup", "sauce", "noodle", "chapati", "roti", "tortilla",
                  "cereal", "oat", "flour", "sugar", "salt", "oil", "vinegar"],
    "protein":   ["egg", "tofu", "dal", "lentil", "almond", "walnut", "peanut", "nut", "seed",
                  "chickpea", "kidney bean"],
}

_SKIP_WORDS = {
    "total", "subtotal", "tax", "discount", "change", "cash", "card",
    "receipt", "thank", "welcome", "date", "time", "invoice", "order",
    "bill", "store", "phone", "address", "qty", "price", "amount",
    "item", "description", "barcode", "savings", "reward", "points",
}


class ParsedItem(BaseModel):
    name: str
    category: str
    quantity: int
    shelf_life: int
    price: Optional[float] = None


class ReceiptScanResult(BaseModel):
    items: list[ParsedItem]
    raw_text: str
    ocr_available: bool
    ocr_engine: str  # "gemini" | "tesseract" | "easyocr" | "none"


def _categorize(name: str) -> str:
    name_l = name.lower()
    for cat, keywords in _CATEGORY_KEYWORDS.items():
        if any(kw in name_l for kw in keywords):
            return cat
    return "vegetable"


def _is_food(name: str) -> bool:
    name_l = name.lower()
    return any(kw in name_l for keywords in _CATEGORY_KEYWORDS.values() for kw in keywords)


def _parse_receipt_text(text: str) -> list[ParsedItem]:
    """Parse plain OCR text into items using regex heuristics (local OCR fallback)."""
    items: list[ParsedItem] = []
    seen: set[str] = set()

    for line in text.splitlines():
        line = line.strip()
        if len(line) < 3:
            continue

        price: Optional[float] = None
        price_m = re.search(r'[\$₹£€]?\s*(\d+\.\d{2})\s*$', line)
        if price_m:
            price = float(price_m.group(1))
            line = line[:price_m.start()].strip()

        if any(skip in line.lower() for skip in _SKIP_WORDS):
            continue
        if not re.search(r'[a-zA-Z]{3,}', line):
            continue

        name_m = re.match(r"^([A-Za-z][A-Za-z\s\-\.'&]*)", line)
        if not name_m:
            continue
        name = name_m.group(1).strip()
        if len(name) < 3:
            continue

        name_key = name.lower()
        if name_key in seen or not _is_food(name):
            continue

        seen.add(name_key)
        cat = _categorize(name)
        items.append(ParsedItem(
            name=name.title(),
            category=cat,
            quantity=1,
            shelf_life=get_item_shelf_life(name, cat),
            price=price if price is not None else get_item_cost(name) or None,
        ))

    return items


def _items_from_gpt_json(raw: list[dict]) -> list[ParsedItem]:
    """Convert Gemini JSON output into ParsedItem list."""
    items: list[ParsedItem] = []
    seen: set[str] = set()
    for entry in raw:
        name = str(entry.get("name", "")).strip()
        if not name or name.lower() in seen:
            continue
        seen.add(name.lower())
        cat = _categorize(name)
        qty = int(entry.get("quantity", 1) or 1)
        price_val = entry.get("price")
        price = float(price_val) if price_val is not None else get_item_cost(name) or None
        items.append(ParsedItem(
            name=name.title(),
            category=cat,
            quantity=max(1, qty),
            shelf_life=get_item_shelf_life(name, cat),
            price=price,
        ))
    return items


async def _ocr_with_gemini(image_bytes: bytes) -> tuple[list[ParsedItem], str]:
    """Send receipt image to Gemini Vision and return structured items + raw text."""
    b64 = base64.b64encode(image_bytes).decode()

    prompt = (
        "You are a receipt parser. Extract only grocery/food items from this receipt image. "
        "Return a JSON array (no markdown, no explanation) where each element has: "
        '{"name": string, "quantity": integer, "price": float or null}. '
        "Ignore tax, total, subtotal, store name, and non-food items. "
        "If quantity is ambiguous, default to 1."
    )

    payload = {
        "contents": [
            {
                "parts": [
                    {"text": prompt},
                    {"inline_data": {"mime_type": "image/jpeg", "data": b64}},
                ]
            }
        ],
        "generationConfig": {"maxOutputTokens": 1024, "temperature": 0.1},
    }

    async with httpx.AsyncClient(timeout=30.0) as client:
        resp = await client.post(
            f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key={GEMINI_API_KEY}",
            headers={"Content-Type": "application/json"},
            json=payload,
        )
        resp.raise_for_status()
        content = resp.json()["candidates"][0]["content"]["parts"][0]["text"].strip()

    # Strip markdown code fences if present
    content = re.sub(r"^```(?:json)?\s*", "", content)
    content = re.sub(r"\s*```$", "", content)

    raw_data: list[dict] = json.loads(content)
    items = _items_from_gpt_json(raw_data)
    raw_text = "\n".join(f"{i['name']} x{i.get('quantity',1)} ${i.get('price','?')}" for i in raw_data)
    return items, raw_text


class ParseTextRequest(BaseModel):
    text: str


@router.post("/parse-text", response_model=ReceiptScanResult)
async def parse_receipt_text_endpoint(body: ParseTextRequest):
    """Parse raw OCR text (from client-side Tesseract.js) into food items."""
    items = _parse_receipt_text(body.text)
    return ReceiptScanResult(
        items=items,
        raw_text=body.text,
        ocr_available=True,
        ocr_engine="tesseract-js",
    )


@router.post("/scan", response_model=ReceiptScanResult)
async def scan_receipt(file: UploadFile = File(...)):
    data = await file.read()
    try:
        img = Image.open(io.BytesIO(data)).convert("RGB")
    except Exception:
        raise HTTPException(status_code=400, detail="Invalid image file")

    # ── Path 1: Gemini Vision (cloud) ─────────────────────────────────────────
    if GEMINI_API_KEY:
        try:
            # Re-encode as JPEG for the API (max 1600px wide)
            img.thumbnail((1600, 1600), Image.LANCZOS)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=90)
            items, raw_text = await _ocr_with_gemini(buf.getvalue())
            return ReceiptScanResult(
                items=items, raw_text=raw_text,
                ocr_available=True, ocr_engine="gemini",
            )
        except Exception as exc:
            logger.warning("Gemini OCR failed, falling back to local: %s", exc)

    # ── Path 2: Local OCR fallback (Tesseract → EasyOCR) ─────────────────────
    if not _TESSERACT_AVAILABLE and not _EASYOCR_AVAILABLE:
        raise HTTPException(
            status_code=503,
            detail=(
                "No OCR engine available. Set GEMINI_API_KEY for cloud OCR (free at aistudio.google.com), "
                "or install Tesseract (https://github.com/UB-Mannheim/tesseract/wiki) + pip install pytesseract, "
                "or run: pip install easyocr"
            ),
        )

    grey = img.convert("L")
    sharpened = ImageEnhance.Contrast(grey).enhance(2.0).filter(ImageFilter.SHARPEN)
    loop = asyncio.get_event_loop()
    text = ""
    engine_used = "none"

    if _TESSERACT_AVAILABLE:
        try:
            _img = sharpened
            text = await loop.run_in_executor(
                None, lambda: pytesseract.image_to_string(_img, config="--psm 6")
            )
            engine_used = "tesseract"
        except Exception as exc:
            logger.warning("Tesseract failed: %s", exc)

    if not text.strip() and _EASYOCR_AVAILABLE:
        try:
            reader = _get_easyocr_reader()
            _arr = np.array(sharpened)
            results = await loop.run_in_executor(None, lambda: reader.readtext(_arr))
            text = "\n".join(r[1] for r in results)
            engine_used = "easyocr"
        except Exception as exc:
            raise HTTPException(status_code=500, detail=f"OCR failed: {exc}")

    if not text.strip():
        raise HTTPException(status_code=422, detail="OCR produced no output — try a clearer image")

    items = _parse_receipt_text(text)
    return ReceiptScanResult(items=items, raw_text=text, ocr_available=True, ocr_engine=engine_used)
