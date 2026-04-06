"""
Single-sample inference with attention map visualisation.

Usage
─────
  python infer.py \\
      --image    /path/to/chest_xray.png \\
      --history  "Patient is a 58-year-old male. Follow-up visit 2." \\
      --checkpoint checkpoints/best.pth \\
      --output_dir results/

Outputs (saved to --output_dir):
  • attention_map.png  — cross-attention heatmap overlaid on the input image
  • top_tokens.txt     — top clinical history tokens by attention weight
  • predictions.txt    — per-class disease probabilities + compatibility score
"""

import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from torchvision import transforms

from data.preprocess import NIH_CLASSES
from models.model import MultimodalDiagnosticModel

# ── Default image normalisation (must match training) ─────────────────────────
_TRANSFORM = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])

# Inverse normalisation for display
_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
_STD  = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)


def denormalise(tensor: torch.Tensor) -> np.ndarray:
    img = tensor.cpu() * _STD + _MEAN
    return img.clamp(0, 1).permute(1, 2, 0).numpy()


KAGGLE_MODEL_DIR = "/kaggle/input/models/bleachiful/mv-model/other/default/1"


def resolve_checkpoint(path: str) -> str:
    """If path is a directory, find the first .pth file inside it."""
    if os.path.isdir(path):
        import glob
        pth_files = sorted(glob.glob(os.path.join(path, "*.pth")))
        if not pth_files:
            raise FileNotFoundError(f"No .pth files found in directory: {path}")
        if len(pth_files) > 1:
            print(f"Multiple checkpoints found, using: {os.path.basename(pth_files[0])}")
        return pth_files[0]
    return path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--image",           required=True,  help="Path to chest X-ray image")
    p.add_argument("--history",         default="NO_HISTORY")
    p.add_argument("--checkpoint",      default=KAGGLE_MODEL_DIR)
    p.add_argument("--output_dir",      default="results")
    p.add_argument("--shared_dim",      type=int,   default=512)
    p.add_argument("--num_heads",       type=int,   default=8)
    p.add_argument("--max_text_length", type=int,   default=128)
    p.add_argument("--visual_backbone", default="densenet")
    p.add_argument("--threshold",       type=float, default=0.5)
    p.add_argument("--top_k_tokens",    type=int,   default=10)
    p.add_argument("--device",          default=None)
    return p.parse_args()


@torch.no_grad()
def run(args):
    device = torch.device(
        args.device if args.device
        else ("cuda" if torch.cuda.is_available() else "cpu")
    )

    # ── Load model ────────────────────────────────────────────────────────
    model = MultimodalDiagnosticModel(
        visual_backbone=args.visual_backbone,
        shared_dim=args.shared_dim,
        num_heads=args.num_heads,
        max_text_length=args.max_text_length,
    ).to(device)

    ckpt_path = resolve_checkpoint(args.checkpoint)
    print(f"Loading checkpoint: {ckpt_path}")
    ckpt = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(ckpt["model"])
    model.eval()

    # ── Preprocess image ──────────────────────────────────────────────────
    pil_img = Image.open(args.image).convert("RGB")
    img_tensor = _TRANSFORM(pil_img).unsqueeze(0).to(device)  # (1, 3, 224, 224)

    # ── Tokenise history ──────────────────────────────────────────────────
    encoding = model.tokenize([args.history], device)

    # ── Forward pass ──────────────────────────────────────────────────────
    out = model(
        images=img_tensor,
        input_ids=encoding["input_ids"],
        attention_mask=encoding["attention_mask"],
        token_type_ids=encoding.get("token_type_ids"),
    )

    probs   = out["probs"][0].cpu().numpy()        # (14,)
    score   = out["score"][0, 0].item()            # scalar
    attn_tv = out["attn_tv"][0].cpu()              # (N_t, N_v)

    os.makedirs(args.output_dir, exist_ok=True)

    # ── Predictions ───────────────────────────────────────────────────────
    pred_path = os.path.join(args.output_dir, "predictions.txt")
    with open(pred_path, "w") as f:
        f.write(f"Image–history compatibility score: {score:.4f}\n\n")
        f.write(f"{'Disease':<25} {'Probability':>11}  {'Predicted':>9}\n")
        f.write("-" * 50 + "\n")
        for cls_name, p in zip(NIH_CLASSES, probs):
            flag = "YES" if p >= args.threshold else "no"
            f.write(f"{cls_name:<25} {p:>11.4f}  {flag:>9}\n")
    print(f"Saved predictions → {pred_path}")

    # ── Visual attention map ───────────────────────────────────────────────
    # attn_tv: (N_t, N_v)  — aggregate over text tokens by mean
    # For ViT: N_v = 196 = 14×14 patches
    attn_map = attn_tv.mean(dim=0)   # (N_v,)
    n_v = attn_map.shape[0]
    h_patches = int(n_v ** 0.5)

    if h_patches * h_patches == n_v:
        attn_2d = attn_map.view(h_patches, h_patches)
        attn_up = F.interpolate(
            attn_2d.unsqueeze(0).unsqueeze(0),
            size=(224, 224),
            mode="bilinear",
            align_corners=False,
        ).squeeze().numpy()
    else:
        # DenseNet spatial tokens — reshape to 7×7
        h = int(n_v ** 0.5)
        attn_2d = attn_map.view(h, h)
        attn_up = F.interpolate(
            attn_2d.unsqueeze(0).unsqueeze(0),
            size=(224, 224),
            mode="bilinear",
            align_corners=False,
        ).squeeze().numpy()

    # Normalise heatmap
    attn_up = (attn_up - attn_up.min()) / (attn_up.max() - attn_up.min() + 1e-8)

    # Overlay on image
    img_np = denormalise(img_tensor[0])
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    axes[0].imshow(img_np)
    axes[0].set_title("Input X-ray")
    axes[0].axis("off")
    axes[1].imshow(img_np)
    axes[1].imshow(attn_up, alpha=0.5, cmap="jet")
    axes[1].set_title(f"Cross-Attention Map  (s={score:.3f})")
    axes[1].axis("off")
    plt.tight_layout()
    attn_path = os.path.join(args.output_dir, "attention_map.png")
    plt.savefig(attn_path, dpi=150)
    plt.close()
    print(f"Saved attention map → {attn_path}")

    # ── Top clinical history tokens ────────────────────────────────────────
    # Token-level importance: mean attention weight per history token
    token_importance = attn_tv.mean(dim=1)  # (N_t,)
    tokenizer = model.text_encoder.tokenizer
    token_ids = encoding["input_ids"][0].cpu()
    tokens = tokenizer.convert_ids_to_tokens(token_ids.tolist())

    # Filter padding tokens
    valid = [(t, token_importance[i].item())
             for i, t in enumerate(tokens)
             if t not in ("[PAD]", "<pad>", "[CLS]", "[SEP]")]
    valid.sort(key=lambda x: x[1], reverse=True)
    top_tokens = valid[:args.top_k_tokens]

    tok_path = os.path.join(args.output_dir, "top_tokens.txt")
    with open(tok_path, "w") as f:
        f.write(f"Top {args.top_k_tokens} clinical history tokens by attention weight:\n\n")
        f.write(f"{'Token':<20} {'Attention':>9}\n")
        f.write("-" * 32 + "\n")
        for token, weight in top_tokens:
            f.write(f"{token:<20} {weight:>9.6f}\n")
    print(f"Saved top tokens → {tok_path}")

    # ── Console summary ────────────────────────────────────────────────────
    print(f"\nCompatibility score s = {score:.4f}")
    positives = [(NIH_CLASSES[i], float(probs[i]))
                 for i in range(14) if probs[i] >= args.threshold]
    if positives:
        print("Predicted diseases:")
        for name, p in sorted(positives, key=lambda x: -x[1]):
            print(f"  {name:<25} {p:.4f}")
    else:
        print("No diseases predicted above threshold.")


if __name__ == "__main__":
    args = parse_args()
    run(args)
