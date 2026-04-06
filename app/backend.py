"""
FastAPI backend for the Multimodal Chest X-ray Diagnosis system.

Run with:
    uvicorn app.backend:app --reload --host 0.0.0.0 --port 8000
"""

import base64
import io
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from torchvision import transforms

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from data.preprocess import NIH_CLASSES
from models.model import MultimodalDiagnosticModel

# ── Config ────────────────────────────────────────────────────────────────────

CHECKPOINT_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "best.pth")
VISUAL_BACKBONE = "densenet"
SHARED_DIM      = 512
NUM_HEADS       = 8
MAX_TEXT_LENGTH = 128
# Per-class optimal thresholds (Youden's J statistic on test set)
PER_CLASS_THRESHOLDS = {
    "Atelectasis":        0.5841,
    "Cardiomegaly":       0.6209,
    "Effusion":           0.5710,
    "Infiltration":       0.6670,
    "Mass":               0.5026,
    "Nodule":             0.5446,
    "Pneumonia":          0.5275,
    "Pneumothorax":       0.3712,
    "Consolidation":      0.6139,
    "Edema":              0.6373,
    "Emphysema":          0.4923,
    "Fibrosis":           0.2731,
    "Pleural_Thickening": 0.6237,
    "Hernia":             0.0456,
}

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ── Image transform ───────────────────────────────────────────────────────────

_TRANSFORM = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])

_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
_STD  = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)


def denormalise(tensor: torch.Tensor) -> np.ndarray:
    img = tensor.cpu() * _STD + _MEAN
    return img.clamp(0, 1).permute(1, 2, 0).numpy()


# ── Load model once at startup ────────────────────────────────────────────────

print(f"Loading model from {CHECKPOINT_PATH} on {DEVICE}...")
model = MultimodalDiagnosticModel(
    visual_backbone=VISUAL_BACKBONE,
    shared_dim=SHARED_DIM,
    num_heads=NUM_HEADS,
    max_text_length=MAX_TEXT_LENGTH,
).to(DEVICE)

ckpt = torch.load(CHECKPOINT_PATH, map_location=DEVICE)
model.load_state_dict(ckpt["model"])
model.eval()
print(f"Model loaded (epoch {ckpt.get('epoch', '?')})")

# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI(title="Chest X-ray Diagnosis API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def build_history(age: int, gender: str, view: str, followup: int) -> str:
    """Build a synthetic clinical history string from patient metadata."""
    gender_str = "male" if gender.upper() == "M" else "female"
    view_str = view.upper() if view.upper() in ("PA", "AP") else "unknown view"
    history = f"Patient is a {age}-year-old {gender_str}. Chest X-ray taken in {view_str} view. "
    if followup == 0:
        history += "This is the initial presentation."
    else:
        history += f"Follow-up visit number {followup}."
    return history


@app.post("/predict")
async def predict(
    file: UploadFile = File(...),
    age: int = Form(default=50),
    gender: str = Form(default="M"),
    view: str = Form(default="PA"),
    followup: int = Form(default=0),
):
    # ── Build clinical history from patient metadata ───────────────────────
    history = build_history(age, gender, view, followup)

    # ── Read & preprocess image ────────────────────────────────────────────
    img_bytes = await file.read()
    pil_img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
    img_tensor = _TRANSFORM(pil_img).unsqueeze(0).to(DEVICE)

    # ── Tokenise history ───────────────────────────────────────────────────
    encoding = model.tokenize([history], DEVICE)

    # ── Inference ──────────────────────────────────────────────────────────
    with torch.no_grad():
        out = model(
            images=img_tensor,
            input_ids=encoding["input_ids"],
            attention_mask=encoding["attention_mask"],
            token_type_ids=encoding.get("token_type_ids"),
        )

    probs   = out["probs"][0].cpu().numpy()
    score   = float(out["score"][0, 0].item())
    attn_tv = out["attn_tv"][0].cpu()   # (N_t, N_v)

    # ── Build predictions list (top-1 by margin above per-class threshold)
    margins = []
    for cls_name, prob in zip(NIH_CLASSES, probs):
        thresh = PER_CLASS_THRESHOLDS.get(cls_name, 0.5)
        margin = float(prob) - thresh
        margins.append((cls_name, float(prob), thresh, margin))

    # Find the disease with the highest margin above its threshold
    best = max(margins, key=lambda x: x[3])
    best_disease = best[0] if best[3] > 0 else None  # only if actually above threshold

    predictions = []
    for cls_name, prob, thresh, margin in margins:
        predictions.append({
            "disease":   cls_name,
            "prob":      round(prob, 4),
            "threshold": round(thresh, 4),
            "predicted": cls_name == best_disease,
        })
    predictions.sort(key=lambda x: x["prob"], reverse=True)

    # ── Generate attention heatmap ─────────────────────────────────────────
    attn_map = attn_tv.mean(dim=0)
    n_v = attn_map.shape[0]
    h_patches = int(n_v ** 0.5)
    attn_2d = attn_map.view(h_patches, h_patches)
    attn_up = F.interpolate(
        attn_2d.unsqueeze(0).unsqueeze(0),
        size=(224, 224),
        mode="bilinear",
        align_corners=False,
    ).squeeze().numpy()
    attn_up = (attn_up - attn_up.min()) / (attn_up.max() - attn_up.min() + 1e-8)

    img_np = denormalise(img_tensor[0])

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    axes[0].imshow(img_np)
    axes[0].set_title("Input X-ray")
    axes[0].axis("off")
    axes[1].imshow(img_np)
    axes[1].imshow(attn_up, alpha=0.5, cmap="jet")
    axes[1].set_title(f"Attention Map  (compatibility={score:.3f})")
    axes[1].axis("off")
    plt.tight_layout()

    buf = io.BytesIO()
    plt.savefig(buf, format="png", dpi=120)
    plt.close()
    buf.seek(0)
    heatmap_b64 = base64.b64encode(buf.read()).decode("utf-8")

    # ── Top history tokens ─────────────────────────────────────────────────
    token_importance = attn_tv.mean(dim=1)
    tokenizer = model.text_encoder.tokenizer
    token_ids = encoding["input_ids"][0].cpu()
    tokens = tokenizer.convert_ids_to_tokens(token_ids.tolist())
    valid = [
        {"token": t, "weight": round(float(token_importance[i].item()), 6)}
        for i, t in enumerate(tokens)
        if t not in ("[PAD]", "<pad>", "[CLS]", "[SEP]")
    ]
    valid.sort(key=lambda x: x["weight"], reverse=True)
    top_tokens = valid[:10]

    return JSONResponse({
        "predictions":       predictions,
        "compatibility":     round(score, 4),
        "heatmap_base64":    heatmap_b64,
        "top_history_tokens": top_tokens,
    })


@app.get("/health")
def health():
    return {"status": "ok", "device": str(DEVICE)}


# Frontend is served separately via Vite dev server (port 3000)
