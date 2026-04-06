# =============================================================================
# MV Project — Per-Class Demonstration Notebook
# Kaggle kernel: run each cell top to bottom
# =============================================================================

# %% [markdown]
# ## Setup
# Adds the project code (uploaded as a Kaggle dataset) to the Python path.

# %% Cell 1 — Path setup
import sys, os, glob, shutil

PROJECT_DIR = "/kaggle/input/datasets/bleachiful/mv-project/MV_project"
MODEL_DIR   = "/kaggle/input/models/bleachiful/mv-model/other/default/1"
DATA_ROOT   = "/kaggle/input/datasets/organizations/nih-chest-xrays/data"

# ── Copy models/ and data/ into /kaggle/working/ so they are importable ──────
for folder in ("models", "data"):
    src = os.path.join(PROJECT_DIR, folder)
    dst = os.path.join("/kaggle/working", folder)
    if os.path.isdir(dst):
        print(f"Already present : {dst}")
    elif os.path.isdir(src):
        shutil.copytree(src, dst)
        print(f"Copied {src} → {dst}")
    else:
        raise FileNotFoundError(f"Source not found: {src}")

# Resolve checkpoint (.pth file inside the model directory)
pth_files = sorted(glob.glob(os.path.join(MODEL_DIR, "*.pth")))
if not pth_files:
    raise FileNotFoundError(f"No .pth file found in {MODEL_DIR}")
CHECKPOINT = pth_files[0]
print(f"\nCheckpoint : {CHECKPOINT}")
print(f"Data root  : {DATA_ROOT}")


# %% Cell 2 — Imports
import warnings
warnings.filterwarnings("ignore")

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import torch
import torch.nn.functional as F
from PIL import Image
from torchvision import transforms

from data.preprocess  import NIH_CLASSES, normalize_labels
from data.dataset     import build_clinical_history
from models.model     import MultimodalDiagnosticModel

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Device: {DEVICE}")


# %% Cell 3 — Image transform (must match training)
TRANSFORM = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])

_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
_STD  = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)

def denorm(tensor):
    """Reverse ImageNet normalisation for display."""
    return (tensor.cpu() * _STD + _MEAN).clamp(0, 1).permute(1, 2, 0).numpy()


# %% Cell 4 — Load model
import torch

raw = torch.load(CHECKPOINT, map_location=DEVICE)

# ── Step 1: extract state_dict regardless of save format ─────────────────────
if isinstance(raw, dict) and "model" in raw:
    state_dict = raw["model"]
    print(f"Format : dict['model']  epoch={raw.get('epoch','?')}  auc={raw.get('auc', raw.get('best_auc','?'))}")
elif isinstance(raw, dict) and any(k.startswith(("visual_encoder","text_encoder","fusion","consistency")) for k in raw):
    state_dict = raw
    print("Format : raw state_dict")
elif hasattr(raw, "state_dict"):
    state_dict = raw.state_dict()
    print("Format : full model object")
else:
    raise ValueError(f"Unrecognised checkpoint. Top-level keys: {list(raw.keys())[:10]}")

# ── Step 2: auto-detect backbone from checkpoint keys ────────────────────────
ckpt_keys = list(state_dict.keys())
if any("visual_encoder.backbone" in k for k in ckpt_keys):
    backbone = "vit"
elif any("visual_encoder.features" in k for k in ckpt_keys):
    backbone = "densenet"
else:
    backbone = "densenet"   # fallback
    print("WARNING: could not detect backbone from keys, defaulting to densenet")

print(f"Backbone detected: {backbone}")
print(f"Total keys in checkpoint: {len(ckpt_keys)}")

# ── Step 3: build model with the correct backbone ────────────────────────────
model = MultimodalDiagnosticModel(
    visual_backbone=backbone,
    shared_dim=512,
    num_heads=8,
).to(DEVICE)

# ── Step 4: load weights ──────────────────────────────────────────────────────
missing, unexpected = model.load_state_dict(state_dict, strict=False)
print(f"Missing keys   : {len(missing)}   {missing[:3] if missing else ''}")
print(f"Unexpected keys: {len(unexpected)} {unexpected[:3] if unexpected else ''}")

model.eval()

# ── Step 5: sanity-check that the classification head has non-random weights ──
head_weight = state_dict.get("head_fused.weight", state_dict.get("head_img.weight"))
if head_weight is not None:
    print(f"\nhead_fused.weight — mean={head_weight.float().mean():.6f}  std={head_weight.float().std():.6f}")
    print("std should be ~0.01–0.05 if trained; ~0.04–0.07 if random init (bad)")
else:
    print("WARNING: classification head key not found in checkpoint")


# %% Cell 4b — Patch cross-modal fusion to fix NaN from unmasked text padding
# Root cause: BERT pads text sequences to 128 tokens. Most tokens are [PAD].
# The V→T cross-attention attends over ALL 128 positions including ~110 padding
# tokens, producing near-zero softmax values that underflow to NaN.
# Fix: pass the BERT attention_mask as key_padding_mask (inverted: 0=ignore)
# and clamp any residual NaN after each attention operation.

import types, torch

_orig_fusion_forward = model.fusion.forward.__func__   # unbound original

def _patched_fusion_forward(self, V, T, key_padding_mask=None):
    V_proj = self.proj_v(V)
    T_proj = self.proj_t(T)

    # T → V  (history attends over image patches — no mask needed on visual side)
    T_fused, attn_tv = self.t_over_v_attn(
        query=T_proj, key=V_proj, value=V_proj,
        need_weights=True, average_attn_weights=False,
    )
    T_fused = torch.nan_to_num(T_fused, nan=0.0)
    attn_tv = torch.nan_to_num(attn_tv, nan=0.0)
    T_fused = self.t_norm(T_proj + T_fused)

    attn_weights_tv = attn_tv.mean(dim=1)   # (B, N_t, N_v)

    # V → T  (image patches attend over history — mask out padding tokens)
    if self.bidirectional:
        # key_padding_mask: True = ignore that position
        # BERT attention_mask: 1 = attend, 0 = pad  →  invert for PyTorch
        kpm = (~key_padding_mask.bool()) if key_padding_mask is not None else None
        V_fused, _ = self.v_over_t_attn(
            query=V_proj, key=T_proj, value=T_proj,
            key_padding_mask=kpm,
        )
        V_fused = torch.nan_to_num(V_fused, nan=0.0)
        V_fused = self.v_norm(V_proj + V_fused)
    else:
        V_fused = V_proj

    return V_fused, T_fused, attn_weights_tv

model.fusion.forward = types.MethodType(_patched_fusion_forward, model.fusion)
print("Fusion forward patched — padding mask applied, NaN clamped.")


# %% Cell 4c — Patch run_inference to pass attention_mask into fusion
# Override the model's forward to thread the mask through to fusion.

_orig_model_forward = model.forward.__func__

def _patched_model_forward(self, images, input_ids, attention_mask, token_type_ids=None):
    V = self.visual_encoder(images)
    V = torch.nan_to_num(V, nan=0.0)

    T, t_cls = self.text_encoder(
        input_ids=input_ids,
        attention_mask=attention_mask,
        token_type_ids=token_type_ids,
    )
    T     = torch.nan_to_num(T,     nan=0.0)
    t_cls = torch.nan_to_num(t_cls, nan=0.0)

    # Pass attention_mask so padding tokens are masked in V→T attention
    V_fused, T_fused, attn_tv = self.fusion(V, T, key_padding_mask=attention_mask)
    V_fused = torch.nan_to_num(V_fused, nan=0.0)
    T_fused = torch.nan_to_num(T_fused, nan=0.0)
    attn_tv = torch.nan_to_num(attn_tv, nan=0.0)

    s = self.consistency(V_fused, T_fused, t_cls)
    s = torch.nan_to_num(s, nan=0.0)

    v_pool      = self.img_proj(V.mean(dim=1))
    v_pool      = self.dropout(v_pool)
    logits_img  = self.head_img(v_pool)

    h_fused     = (V_fused * s.unsqueeze(1)).mean(dim=1)
    h_fused     = self.dropout(h_fused)
    logits_fused = self.head_fused(h_fused)

    from models.consistency import gated_logits
    logits = gated_logits(logits_img, logits_fused, s)

    return {
        "logits":     logits,
        "probs":      torch.sigmoid(logits),
        "logits_img": logits_img,
        "score":      s,
        "attn_tv":    attn_tv,
    }

model.forward = types.MethodType(_patched_model_forward, model)
print("Model forward patched — NaN guards added at every stage.")


# %% Cell 5 — Load NIH metadata and build image path map

# Build filename → full path mapping across all images_XXX/images/ subfolders
img_map: dict[str, str] = {}
for path in glob.glob(os.path.join(DATA_ROOT, "images_*", "images", "*.png")):
    img_map[os.path.basename(path)] = path

# Fallback: flat images/ folder
if not img_map:
    flat = os.path.join(DATA_ROOT, "images")
    if os.path.isdir(flat):
        for fname in os.listdir(flat):
            if fname.endswith(".png"):
                img_map[fname] = os.path.join(flat, fname)

meta = pd.read_csv(os.path.join(DATA_ROOT, "Data_Entry_2017.csv"))
meta.columns = meta.columns.str.strip()
# Keep only images that exist on disk
meta = meta[meta["Image Index"].isin(img_map)].reset_index(drop=True)
print(f"Metadata rows after filtering to available images: {len(meta)}")


# %% Cell 6 — Pick one representative sample per disease class

def find_sample(class_name: str, meta: pd.DataFrame) -> pd.Series | None:
    """Return a valid metadata row whose Finding Labels contains class_name."""
    mask = meta["Finding Labels"].str.contains(class_name, na=False, case=False)
    hits = meta[mask]
    if hits.empty:
        return None
    # Prefer images with only this one label (cleaner demo)
    solo = hits[~hits["Finding Labels"].str.contains(r"\|", na=False)]
    candidates = list(solo.itertuples()) if not solo.empty else list(hits.itertuples())
    # Walk candidates until we find one whose image opens without error
    for c in candidates[:20]:
        row = meta.loc[c.Index]
        try:
            Image.open(img_map[row["Image Index"]]).convert("RGB")
            return row
        except Exception:
            continue
    return None

samples: dict[str, pd.Series] = {}
for cls in NIH_CLASSES:
    row = find_sample(cls, meta)
    if row is not None:
        samples[cls] = row
    else:
        print(f"  WARNING: no sample found for class '{cls}'")

print(f"Samples ready for {len(samples)}/14 classes.")


# %% Cell 7 — Inference helper

@torch.no_grad()
def run_inference(row: pd.Series):
    """
    Load image, build history, run model.
    Returns: (pil_image, history_str, probs array, score scalar, attn_tv tensor)
    """
    img_name = row["Image Index"]
    pil_img  = Image.open(img_map[img_name]).convert("RGB")
    img_t    = TRANSFORM(pil_img).unsqueeze(0).to(DEVICE)

    history  = build_clinical_history(row)
    encoding = model.tokenize([history], DEVICE)

    out = model(
        images=img_t,
        input_ids=encoding["input_ids"],
        attention_mask=encoding["attention_mask"],
        token_type_ids=encoding.get("token_type_ids"),
    )

    probs   = out["probs"][0].cpu().numpy()                  # (14,)
    probs   = np.nan_to_num(probs, nan=0.0, posinf=1.0, neginf=0.0).clip(0.0, 1.0)
    raw_score = out["score"][0, 0].item()
    score   = float(np.nan_to_num(raw_score, nan=0.0))
    attn_tv = out["attn_tv"][0].cpu()                        # (N_t, N_v)

    if np.isnan(raw_score):
        print(f"  WARNING: score was NaN for {row['Image Index']} — replaced with 0")

    return pil_img, history, probs, score, attn_tv, img_t[0]


def build_attn_map(attn_tv: torch.Tensor, img_t: torch.Tensor) -> np.ndarray:
    """
    Average T→V attention over text tokens, reshape to 7×7 (DenseNet spatial),
    upsample to 224×224, and normalise to [0, 1].
    """
    attn = attn_tv.mean(dim=0)          # (N_v,)  — average over text tokens
    h    = int(attn.shape[0] ** 0.5)   # 7 for DenseNet (49 tokens)
    attn_2d = attn.view(h, h)
    attn_up = F.interpolate(
        attn_2d.unsqueeze(0).unsqueeze(0).float(),
        size=(224, 224), mode="bilinear", align_corners=False,
    ).squeeze().numpy()
    attn_up = (attn_up - attn_up.min()) / (attn_up.max() - attn_up.min() + 1e-8)
    return attn_up


# %% Cell 8 — Full per-class visualisation grid
# For each disease: shows the X-ray, attention heatmap, and probability bar chart.

THRESHOLD = 0.15   # NIH models output low raw probs due to class imbalance; 0.15 is appropriate
COLS      = 3
N         = len(samples)

fig, axes = plt.subplots(N, COLS, figsize=(16, N * 4))
fig.suptitle("Per-Class Demonstration — Multimodal Chest X-ray Diagnosis",
             fontsize=16, fontweight="bold", y=1.002)

for row_i, (cls_name, row) in enumerate(samples.items()):
    pil_img, history, probs, score, attn_tv, img_t = run_inference(row)
    attn_map = build_attn_map(attn_tv, img_t)
    img_np   = denorm(img_t)

    ax_img, ax_attn, ax_bar = axes[row_i]

    # ── Column 1: original X-ray ─────────────────────────────────────────────
    ax_img.imshow(img_np, cmap="gray")
    ax_img.set_title(f"{cls_name}\n{row['Image Index']}", fontsize=9, fontweight="bold")
    ax_img.axis("off")
    short_hist = history if len(history) < 80 else history[:77] + "..."
    ax_img.set_xlabel(short_hist, fontsize=7, labelpad=4)

    # ── Column 2: attention heatmap overlay ──────────────────────────────────
    ax_attn.imshow(img_np, cmap="gray")
    ax_attn.imshow(attn_map, alpha=0.5, cmap="jet")
    ax_attn.set_title(f"Attention Map  (s={score:.6f})", fontsize=9)
    ax_attn.axis("off")

    # ── Column 3: probability bar chart ──────────────────────────────────────
    gt_idx = NIH_CLASSES.index(cls_name)

    # Scale x-axis to actual data range so bars are always visible
    prob_max = float(probs.max()) if np.isfinite(probs.max()) else THRESHOLD
    x_max = max(prob_max * 1.35, THRESHOLD * 2.0, 0.08)

    colors = []
    for i, c in enumerate(NIH_CLASSES):
        if c == cls_name:
            colors.append("#e74c3c" if probs[i] >= THRESHOLD else "#c0392b55")
        else:
            colors.append("#3498db" if probs[i] >= THRESHOLD else "#2c3e5066")

    ax_bar.barh(NIH_CLASSES, probs, color=colors, height=0.6)
    ax_bar.set_xlim(0, x_max)
    ax_bar.axvline(THRESHOLD, color="#f39c12", linestyle="--", linewidth=1.2)
    ax_bar.text(THRESHOLD + x_max * 0.01, 13.4, f"t={THRESHOLD}",
                color="#f39c12", fontsize=7, va="center")

    ax_bar.set_xlabel("Probability", fontsize=8)
    ax_bar.tick_params(axis="y", labelsize=7)
    ax_bar.tick_params(axis="x", labelsize=7)

    # Highlight ground-truth label and annotate with exact value
    ax_bar.get_yticklabels()[gt_idx].set_color("#e74c3c")
    ax_bar.get_yticklabels()[gt_idx].set_fontweight("bold")
    ax_bar.text(probs[gt_idx] + x_max * 0.015, gt_idx,
                f"{probs[gt_idx]:.3f}", va="center", fontsize=7, color="#e74c3c", fontweight="bold")

    predicted = probs[gt_idx] >= THRESHOLD
    status = "PREDICTED ✓" if predicted else "MISSED ✗"
    ax_bar.set_title(f"{status}  (p={probs[gt_idx]:.3f})", fontsize=9,
                     color="#2ecc71" if predicted else "#e74c3c")

    patches = [
        mpatches.Patch(color="#e74c3c", label="Ground-truth class"),
        mpatches.Patch(color="#3498db", label="Other positive"),
    ]
    ax_bar.legend(handles=patches, fontsize=6, loc="lower right")

plt.tight_layout()
plt.savefig("per_class_demo.png", dpi=120, bbox_inches="tight")
plt.show()
print("Saved: per_class_demo.png")


# %% Cell 9 — Summary table: prediction result for the target class in each sample

print(f"\n{'Class':<22} {'Image':<25} {'GT Prob':>8}  {'Predicted':>10}  {'Score s (raw)':>15}")
print("─" * 88)
for cls_name, row in samples.items():
    _, _, probs, score, _, _ = run_inference(row)
    gt_prob   = probs[NIH_CLASSES.index(cls_name)]
    predicted = gt_prob >= THRESHOLD   # THRESHOLD defined in Cell 8
    status    = "YES ✓" if predicted else "no ✗"
    note      = " ← NaN in model" if score == 0.0 else ""
    print(f"{cls_name:<22} {row['Image Index']:<25} {gt_prob:>8.4f}  {status:>10}  {score:>15.8f}{note}")


# %% Cell 10 — Per-class attention: 4×4 subplot grid (one heatmap per disease)
# Clean overview for presentation slides.

fig2, axes2 = plt.subplots(4, 4, figsize=(16, 16))
fig2.suptitle("Cross-Modal Attention Maps — One Sample per Disease Class",
              fontsize=14, fontweight="bold")

for i, (cls_name, row) in enumerate(samples.items()):
    pil_img, _, probs, score, attn_tv, img_t = run_inference(row)
    attn_map = build_attn_map(attn_tv, img_t)
    img_np   = denorm(img_t)

    ax = axes2[i // 4][i % 4]
    ax.imshow(img_np, cmap="gray")
    ax.imshow(attn_map, alpha=0.55, cmap="jet")
    gt_prob = probs[NIH_CLASSES.index(cls_name)]
    ax.set_title(f"{cls_name}\np={gt_prob:.3f}  s={score:.3f}", fontsize=8)
    ax.axis("off")

# Hide the 15th and 16th subplot slots (only 14 classes)
for j in range(14, 16):
    axes2[j // 4][j % 4].axis("off")

plt.tight_layout()
plt.savefig("attention_grid.png", dpi=120, bbox_inches="tight")
plt.show()
print("Saved: attention_grid.png")
