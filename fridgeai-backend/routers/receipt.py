from __future__ import annotations
import asyncio
import io
import re
import logging
from typing import Optional

import numpy as np
from fastapi import APIRouter, File, HTTPException, UploadFile
from PIL import Image, ImageEnhance, ImageFilter
from pydantic import BaseModel

from routers.lookup import get_item_shelf_life, get_item_cost

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/receipt", tags=["receipt"])

try:
    import pytesseract
    _TESSERACT_AVAILABLE = True
except ImportError:
    _TESSERACT_AVAILABLE = False

try:
    import easyocr as _easyocr_lib
    _EASYOCR_AVAILABLE = True
except ImportError:
    _EASYOCR_AVAILABLE = False

_easyocr_reader = None


def _get_easyocr_reader():
    global _easyocr_reader
    if _easyocr_reader is None:
        _easyocr_reader = _easyocr_lib.Reader(['en'], gpu=False, verbose=False)
    return _easyocr_reader

# Keyword → category mapping for receipt parsing
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
    ocr_engine: str  # "tesseract" | "easyocr" | "none"


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
    items: list[ParsedItem] = []
    seen: set[str] = set()

    for line in text.splitlines():
        line = line.strip()
        if len(line) < 3:
            continue

        # Extract trailing price
        price: Optional[float] = None
        price_m = re.search(r'[\$₹£€]?\s*(\d+\.\d{2})\s*$', line)
        if price_m:
            price = float(price_m.group(1))
            line = line[:price_m.start()].strip()

        # Skip non-food lines
        if any(skip in line.lower() for skip in _SKIP_WORDS):
            continue

        # Need at least 3 consecutive letters
        if not re.search(r'[a-zA-Z]{3,}', line):
            continue

        # Extract leading alphabetic token as name
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


@router.post("/scan", response_model=ReceiptScanResult)
async def scan_receipt(file: UploadFile = File(...)):
    if not _TESSERACT_AVAILABLE and not _EASYOCR_AVAILABLE:
        raise HTTPException(
            status_code=503,
            detail=(
                "No OCR engine available. Either install Tesseract "
                "(https://github.com/UB-Mannheim/tesseract/wiki) + pip install pytesseract, "
                "or run: pip install easyocr"
            ),
        )

    data = await file.read()
    try:
        img = Image.open(io.BytesIO(data)).convert("RGB")
    except Exception:
        raise HTTPException(status_code=400, detail="Invalid image file")

    # Preprocess: greyscale → high contrast → sharpen for better OCR
    grey = img.convert("L")
    enhanced = ImageEnhance.Contrast(grey).enhance(2.0)
    sharpened = enhanced.filter(ImageFilter.SHARPEN)

    text = ""
    engine_used = "none"

    loop = asyncio.get_event_loop()

    if _TESSERACT_AVAILABLE:
        try:
            _img = sharpened
            text = await loop.run_in_executor(
                None, lambda: pytesseract.image_to_string(_img, config="--psm 6")
            )
            engine_used = "tesseract"
        except Exception as exc:
            logger.warning("Tesseract failed, falling back to easyocr: %s", exc)

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
