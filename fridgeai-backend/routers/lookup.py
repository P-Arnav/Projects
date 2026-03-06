"""
lookup.py — Path 2 (category shelf-life defaults) + Path 3 (Open Food Facts barcode lookup)
"""

from __future__ import annotations
from typing import Optional
import httpx
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/lookup", tags=["lookup"])

# ── Path 2: Default shelf life by category ──────────────────────────────────
# Sources: USDA FoodKeeper, FDA guidelines, Open Food Facts typical values
DEFAULT_SHELF_LIFE: dict[str, int] = {
    "dairy":     7,
    "protein":   7,
    "meat":      3,
    "vegetable": 6,
    "fruit":     7,
    "fish":      2,
    "cooked":    4,
    "beverage":  7,
}


# ── Item-specific shelf lives (fridge storage, days) ────────────────────────
# Overrides category defaults for common grocery items.
# Sources: USDA FoodKeeper, FDA guidelines, StillTasty
ITEM_SHELF_LIFE: dict[str, int] = {
    # dairy
    "milk":          7,
    "yogurt":       14,
    "cheese":       14,
    "cheddar":      21,
    "mozzarella":    7,
    "parmesan":     30,
    "cream cheese": 14,
    "sour cream":   14,
    "butter":       21,
    "heavy cream":   7,
    "cream":         7,
    "cottage cheese": 7,
    # protein / eggs
    "eggs":         21,
    "tofu":          5,
    "hummus":        7,
    "tempeh":        7,
    # meat
    "chicken":       2,
    "beef":          2,
    "ground beef":   2,
    "steak":         3,
    "pork":          3,
    "ham":           5,
    "bacon":         7,
    "deli meat":     5,
    "sausage":       2,
    "turkey":        2,
    # fish
    "fish":          2,
    "salmon":        2,
    "tuna":          2,
    "shrimp":        2,
    "cod":           2,
    # vegetables
    "carrot":       21,
    "broccoli":      5,
    "spinach":       5,
    "lettuce":       7,
    "tomato":        5,
    "cucumber":      7,
    "bell pepper":   7,
    "pepper":        7,
    "celery":       14,
    "onion":         7,
    "mushroom":      5,
    "kale":          7,
    "zucchini":      7,
    "cauliflower":   7,
    "potato":       14,
    "garlic":       21,
    "corn":          3,
    "asparagus":     4,
    "green beans":   5,
    "cabbage":      14,
    "beet":         14,
    "avocado":       3,
    # fruits
    "apple":        21,
    "orange":       21,
    "banana":        5,
    "strawberry":    5,
    "blueberry":     7,
    "grape":         7,
    "lemon":        21,
    "lime":         21,
    "mango":         7,
    "pear":          7,
    "peach":         5,
    "watermelon":    7,
    "pineapple":     5,
    "kiwi":         14,
    "plum":          5,
    "cherry":        5,
    "raspberry":     3,
    "melon":         7,
    # cooked / prepared
    "leftovers":     4,
    "soup":          4,
    "bread":         7,
    "sauce":         7,
    # beverages
    "juice":         7,
    "orange juice":  7,
    "apple juice":  10,
}


def get_item_shelf_life(name: str, category: str) -> int:
    """Return shelf life in days, preferring item-specific over category default."""
    name_lower = name.lower()
    # Exact match first
    if name_lower in ITEM_SHELF_LIFE:
        return ITEM_SHELF_LIFE[name_lower]
    # Keyword match (longest key wins to prefer "ground beef" over "beef")
    matched_key = None
    for key in sorted(ITEM_SHELF_LIFE, key=len, reverse=True):
        if key in name_lower:
            matched_key = key
            break
    if matched_key:
        return ITEM_SHELF_LIFE[matched_key]
    return DEFAULT_SHELF_LIFE.get(category, 7)


class ShelfLifeResponse(BaseModel):
    category:    str
    shelf_life:  int   # days
    source:      str


@router.get("/shelf-life/{category}", response_model=ShelfLifeResponse)
async def get_shelf_life(category: str):
    """Return the default shelf life in days for a given category."""
    cat = category.lower()
    if cat not in DEFAULT_SHELF_LIFE:
        raise HTTPException(
            status_code=404,
            detail=f"Unknown category '{category}'. "
                   f"Valid: {', '.join(DEFAULT_SHELF_LIFE)}"
        )
    return ShelfLifeResponse(
        category=cat,
        shelf_life=DEFAULT_SHELF_LIFE[cat],
        source="USDA FoodKeeper / FDA guidelines",
    )


class ItemShelfLifeResponse(BaseModel):
    name:        str
    category:    str
    shelf_life:  int
    source:      str


@router.get("/item/{name}", response_model=ItemShelfLifeResponse)
async def get_item_shelf_life_endpoint(name: str):
    """Return shelf life and category for a specific grocery item name."""
    name_lower = name.lower().strip()
    category   = _map_category([name_lower])
    shelf_life = get_item_shelf_life(name_lower, category)
    source     = "item-specific (USDA FoodKeeper)" if name_lower in ITEM_SHELF_LIFE or any(k in name_lower for k in ITEM_SHELF_LIFE) else "category default"
    return ItemShelfLifeResponse(
        name=name,
        category=category,
        shelf_life=shelf_life,
        source=source,
    )


# ── Path 3: Open Food Facts barcode lookup ──────────────────────────────────

# Map Open Food Facts category tags to ASLIE categories
_OFF_TAG_MAP: dict[str, str] = {
    # dairy
    "en:dairies": "dairy", "en:dairy": "dairy", "en:milks": "dairy",
    "en:cheeses": "dairy", "en:yogurts": "dairy", "en:butters": "dairy",
    "en:creams": "dairy", "en:fermented-milks": "dairy",
    "en:ice-creams": "dairy", "en:desserts": "dairy",
    # protein / eggs / legumes / nuts
    "en:eggs": "protein", "en:egg-based-foods": "protein",
    "en:plant-based-foods-and-beverages": "protein",
    "en:legumes": "protein", "en:beans": "protein", "en:lentils": "protein",
    "en:nuts": "protein", "en:seeds": "protein", "en:tofu": "protein",
    "en:meat-alternatives": "protein",
    # meat
    "en:meats": "meat", "en:poultry": "meat", "en:beef": "meat",
    "en:pork": "meat", "en:sausages": "meat", "en:deli-meats": "meat",
    "en:lamb": "meat", "en:mutton": "meat", "en:goat": "meat",
    "en:chicken": "meat", "en:turkey": "meat",
    # fish
    "en:fish": "fish", "en:seafood": "fish", "en:fishes": "fish",
    "en:tuna": "fish", "en:salmon": "fish", "en:shrimps": "fish",
    "en:prawns": "fish", "en:crustaceans": "fish",
    # vegetable
    "en:vegetables": "vegetable", "en:fresh-vegetables": "vegetable",
    "en:salads": "vegetable", "en:tomatoes": "vegetable",
    "en:leafy-vegetables": "vegetable", "en:root-vegetables": "vegetable",
    "en:mushrooms": "vegetable",
    # fruit
    "en:fruits": "fruit", "en:fresh-fruits": "fruit",
    "en:berries": "fruit", "en:citrus": "fruit",
    "en:dried-fruits": "fruit", "en:tropical-fruits": "fruit",
    # cooked / prepared / condiments / spices — all shelf-stable processed foods
    "en:prepared-meals": "cooked", "en:sandwiches": "cooked",
    "en:soups": "cooked", "en:ready-to-eat": "cooked",
    "en:sauces": "cooked", "en:condiments": "cooked",
    "en:spices": "cooked", "en:seasonings": "cooked",
    "en:herbs": "cooked", "en:spice-mixes": "cooked",
    "en:marinades": "cooked", "en:dressings": "cooked",
    "en:pastes": "cooked", "en:pickles": "cooked",
    "en:canned-foods": "cooked", "en:snacks": "cooked",
    "en:baked-goods": "cooked", "en:breads": "cooked",
    "en:cereals": "cooked", "en:pasta": "cooked",
    "en:rice": "cooked", "en:grains": "cooked",
    "en:chips": "cooked", "en:crackers": "cooked",
    "en:chocolates": "cooked", "en:sweets": "cooked",
    "en:jams": "cooked", "en:spreads": "cooked",
    "en:oils": "cooked", "en:vinegars": "cooked",
    "en:powders": "cooked",
    # beverage
    "en:beverages": "beverage", "en:juices": "beverage",
    "en:sodas": "beverage", "en:waters": "beverage",
    "en:plant-based-beverages": "beverage",
    "en:teas": "beverage", "en:coffees": "beverage",
    "en:energy-drinks": "beverage", "en:alcoholic-beverages": "beverage",
    "en:milk-drinks": "beverage",
}

OFF_API = "https://world.openfoodfacts.org/api/v2/product/{barcode}.json"
OFF_FIELDS = "product_name,categories_tags,product_quantity,quantity,image_url"


def _map_category(tags: list[str]) -> str:
    """Return the best ASLIE category from a list of OFF category tags."""
    for tag in tags:
        cat = _OFF_TAG_MAP.get(tag.lower())
        if cat:
            return cat
    # Fallback: scan for partial keyword matches
    joined = " ".join(tags).lower()
    for keyword, cat in [
        ("dairy", "dairy"), ("milk", "dairy"), ("cheese", "dairy"), ("yogurt", "dairy"), ("cream", "dairy"),
        ("meat", "meat"), ("chicken", "meat"), ("beef", "meat"), ("pork", "meat"),
        ("lamb", "meat"), ("mutton", "meat"), ("poultry", "meat"),
        ("turkey", "meat"), ("bacon", "meat"), ("ham", "meat"), ("sausage", "meat"),
        ("fish", "fish"), ("salmon", "fish"), ("tuna", "fish"), ("cod", "fish"),
        ("seafood", "fish"), ("prawn", "fish"), ("shrimp", "fish"),
        ("vegetable", "vegetable"), ("salad", "vegetable"), ("mushroom", "vegetable"),
        ("carrot", "vegetable"), ("broccoli", "vegetable"), ("lettuce", "vegetable"),
        ("tomato", "vegetable"), ("onion", "vegetable"), ("potato", "vegetable"),
        ("garlic", "vegetable"), ("kale", "vegetable"), ("asparagus", "vegetable"),
        ("cauliflower", "vegetable"), ("beet", "vegetable"),
        ("pepper", "vegetable"), ("capsicum", "vegetable"), ("chili", "vegetable"),
        ("green chili", "vegetable"), ("ginger", "vegetable"), ("spinach", "vegetable"),
        ("cucumber", "vegetable"), ("zucchini", "vegetable"), ("eggplant", "vegetable"),
        ("cabbage", "vegetable"), ("celery", "vegetable"), ("corn", "vegetable"),
        ("pea", "vegetable"), ("bean sprout", "vegetable"),
        ("fruit", "fruit"), ("apple", "fruit"), ("orange", "fruit"), ("banana", "fruit"),
        ("berry", "fruit"), ("grape", "fruit"), ("lemon", "fruit"), ("lime", "fruit"),
        ("mango", "fruit"), ("pear", "fruit"), ("peach", "fruit"), ("plum", "fruit"),
        ("cherry", "fruit"), ("melon", "fruit"), ("kiwi", "fruit"),
        ("pineapple", "fruit"), ("avocado", "fruit"),
        ("drink", "beverage"), ("juice", "beverage"), ("water", "beverage"), ("tea", "beverage"), ("coffee", "beverage"),
        ("spice", "cooked"), ("seasoning", "cooked"), ("condiment", "cooked"), ("sauce", "cooked"),
        ("powder", "cooked"), ("masala", "cooked"), ("herb", "cooked"), ("pickle", "cooked"),
        ("snack", "cooked"), ("cereal", "cooked"), ("bread", "cooked"), ("pasta", "cooked"),
        ("rice", "cooked"), ("grain", "cooked"), ("oil", "cooked"), ("jam", "cooked"),
        ("prepared", "cooked"), ("meal", "cooked"), ("canned", "cooked"),
        ("egg", "protein"), ("legume", "protein"), ("bean", "protein"), ("lentil", "protein"), ("nut", "protein"),
    ]:
        if keyword in joined:
            return cat
    return "cooked"   # best guess for unrecognised processed/packaged products


class BarcodeResult(BaseModel):
    barcode:     str
    name:        Optional[str]
    category:    str
    shelf_life:  int           # days (from DEFAULT_SHELF_LIFE for the mapped category)
    source:      str
    off_found:   bool


@router.get("/barcode/{barcode}", response_model=BarcodeResult)
async def lookup_barcode(barcode: str):
    """
    Look up a product by barcode via Open Food Facts and return
    pre-filled name, category, and default shelf life.
    """
    url = OFF_API.format(barcode=barcode)
    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            resp = await client.get(url, params={"fields": OFF_FIELDS},
                                    headers={"User-Agent": "FridgeAI/1.0"})
            data = resp.json()
    except Exception as exc:
        raise HTTPException(status_code=502, detail=f"Open Food Facts unreachable: {exc}")

    if data.get("status") != 1:
        # Product not found — return defaults so the form still works
        return BarcodeResult(
            barcode=barcode,
            name=None,
            category="protein",
            shelf_life=DEFAULT_SHELF_LIFE["protein"],
            source="defaults (barcode not in Open Food Facts)",
            off_found=False,
        )

    product   = data.get("product", {})
    name      = product.get("product_name") or None
    tags      = product.get("categories_tags") or []
    category  = _map_category(tags)
    shelf_life = DEFAULT_SHELF_LIFE.get(category, 7)

    return BarcodeResult(
        barcode=barcode,
        name=name,
        category=category,
        shelf_life=shelf_life,
        source="Open Food Facts",
        off_found=True,
    )
