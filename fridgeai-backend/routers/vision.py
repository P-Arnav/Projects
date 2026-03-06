"""
vision.py — POST /vision/scan  (Grounding DINO + MobileNetV3 spoilage)

Accepts a JPEG/PNG image upload, runs zero-shot object detection,
crops each bounding box and classifies it as fresh/spoiled via a
fine-tuned MobileNetV3-Small. All models lazy-loaded on first request.
"""

from __future__ import annotations
import io
import logging
from pathlib import Path
from uuid import uuid4

from fastapi import APIRouter, HTTPException, Request, UploadFile, File
from pydantic import BaseModel

from routers.lookup import _map_category, get_item_shelf_life

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/vision", tags=["vision"])

MODEL_ID       = "IDEA-Research/grounding-dino-base"
THRESHOLD      = 0.5
SPOILAGE_MODEL = Path(__file__).parent.parent / "models" / "spoilage_mobilenetv3.pth"
SPOILAGE_THRESHOLD = 0.5   # P(spoiled) >= this → flagged


# ── Pydantic models ──────────────────────────────────────────────────────────

class DetectedItem(BaseModel):
    name:                str
    category:            str
    shelf_life:          int
    confidence:          float
    count:               int
    spoilage_detected:   bool
    spoilage_confidence: float   # avg P(spoiled) across bounding boxes for this label


class ScanResult(BaseModel):
    items:      list[DetectedItem]
    capture_id: str


# ── Lazy loaders ─────────────────────────────────────────────────────────────

def _get_dino(app_state):
    if getattr(app_state, "vision_model", None) is not None:
        return app_state.vision_processor, app_state.vision_model, app_state.vision_device

    try:
        import torch
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
    except ImportError as e:
        raise HTTPException(status_code=503, detail=f"Vision deps missing: {e}")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    logger.info("Loading Grounding DINO '%s' on %s…", MODEL_ID, device.upper())
    processor = AutoProcessor.from_pretrained(MODEL_ID)
    model     = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL_ID).to(device)
    app_state.vision_processor = processor
    app_state.vision_model     = model
    app_state.vision_device    = device
    logger.info("Grounding DINO loaded.")
    return processor, model, device


def _get_spoilage(app_state, device):
    if getattr(app_state, "spoilage_model", None) is not None:
        return app_state.spoilage_model

    try:
        import torch
        import torch.nn as nn
        from torchvision import models
    except ImportError:
        return None   # spoilage scoring optional

    if not SPOILAGE_MODEL.exists():
        logger.warning("Spoilage model not found at %s — skipping.", SPOILAGE_MODEL)
        return None

    net = models.mobilenet_v3_small(weights=None)
    net.classifier = nn.Sequential(
        nn.Linear(576, 256),
        nn.Hardswish(),
        nn.Dropout(0.2),
        nn.Linear(256, 1),
    )
    net.load_state_dict(torch.load(SPOILAGE_MODEL, map_location=device))
    net.eval().to(device)
    app_state.spoilage_model = net
    logger.info("Spoilage classifier loaded.")
    return net


# ── Spoilage inference on a single crop ──────────────────────────────────────

def _spoilage_score(crop, spoilage_net, device) -> float:
    """Return P(spoiled) in [0,1] for a PIL crop."""
    import torch
    from torchvision import transforms

    tf = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])
    tensor = tf(crop).unsqueeze(0).to(device)
    with torch.no_grad():
        logit = spoilage_net(tensor).squeeze()
        return float(torch.sigmoid(logit).cpu())


# ── Grocery prompt ────────────────────────────────────────────────────────────

GROCERY_PROMPT = (
    "milk . eggs . apple . orange . banana . yogurt . cheese . butter . "
    "cream . juice . water . soda . chicken . beef . fish . carrot . "
    "tomato . lettuce . broccoli . onion . potato . garlic . pepper . "
    "strawberry . grape . lemon . bread . leftovers . container . bottle . "
    "jar . can . sauce . condiment . spice . powder . chocolate . snack ."
)


# ── Endpoint ─────────────────────────────────────────────────────────────────

@router.post("/scan", response_model=ScanResult)
async def scan_image(request: Request, file: UploadFile = File(...)):
    """
    1. Run Grounding DINO to detect grocery items + bounding boxes.
    2. Crop each box and run MobileNetV3 spoilage classifier.
    3. Return grouped items with category, shelf life, and spoilage flag.
    """
    try:
        from PIL import Image
        import torch
    except ImportError as e:
        raise HTTPException(status_code=503, detail=f"Missing dependency: {e}")

    processor, dino, device = _get_dino(request.app.state)
    spoilage_net            = _get_spoilage(request.app.state, device)

    raw = await file.read()
    try:
        pil_image = Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid image: {e}")

    # ── Grounding DINO detection ──
    inputs = processor(images=pil_image, text=GROCERY_PROMPT, return_tensors="pt").to(device)
    with torch.no_grad():
        outputs = dino(**inputs)

    results = processor.post_process_grounded_object_detection(
        outputs, inputs.input_ids,
        threshold=THRESHOLD, text_threshold=THRESHOLD,
        target_sizes=[pil_image.size[::-1]],
    )[0]

    labels = results["labels"]
    scores = results["scores"].cpu().tolist()
    boxes  = results["boxes"].cpu().tolist()

    if not labels:
        return ScanResult(items=[], capture_id=str(uuid4()))

    # ── Group detections by label, run spoilage per box ──
    w, h = pil_image.size
    label_data: dict[str, dict] = {}

    for label, score, box in zip(labels, scores, boxes):
        if label not in label_data:
            label_data[label] = {"scores": [], "spoilage_scores": []}
        label_data[label]["scores"].append(score)

        if spoilage_net is not None:
            x1, y1, x2, y2 = (
                max(0, int(box[0])), max(0, int(box[1])),
                min(w, int(box[2])), min(h, int(box[3])),
            )
            if x2 > x1 and y2 > y1:
                crop = pil_image.crop((x1, y1, x2, y2))
                label_data[label]["spoilage_scores"].append(_spoilage_score(crop, spoilage_net, device))

    # ── Build response ──
    detected: list[DetectedItem] = []
    for label, data in label_data.items():
        category   = _map_category([label])
        shelf_life = get_item_shelf_life(label, category)
        sp_scores  = data["spoilage_scores"]
        avg_sp     = round(sum(sp_scores) / len(sp_scores), 3) if sp_scores else 0.0

        detected.append(DetectedItem(
            name                = label,
            category            = category,
            shelf_life          = shelf_life,
            confidence          = round(sum(data["scores"]) / len(data["scores"]), 3),
            count               = len(data["scores"]),
            spoilage_detected   = avg_sp >= SPOILAGE_THRESHOLD,
            spoilage_confidence = avg_sp,
        ))

    detected.sort(key=lambda x: (-x.count, -x.confidence))
    return ScanResult(items=detected, capture_id=str(uuid4()))
