"""
Demo script — run a single chest X-ray through the model and print results.

Usage
─────
  python demo.py --image path/to/xray.png --checkpoint checkpoints/best.pth
  python demo.py --image path/to/xray.png --checkpoint checkpoints/best.pth \
                 --age 67 --gender M --view PA --followup 2

Arguments
─────────
  --image        Path to a chest X-ray image (PNG or JPG)
  --checkpoint   Path to a saved model checkpoint (.pth)
  --age          Patient age  (default: 50)
  --gender       M or F       (default: M)
  --view         PA or AP     (default: PA)
  --followup     Follow-up visit number, 0 = initial (default: 0)
  --threshold    Probability threshold for a positive prediction (default: 0.5)
  --backbone     vit or densenet (default: densenet)
"""

import argparse

import torch
from PIL import Image
from torchvision import transforms

from data.preprocess import NIH_CLASSES
from models.model import MultimodalDiagnosticModel


# ── Image preprocessing (must match training) ─────────────────────────────────
TRANSFORM = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])


def build_history(age: int, gender: str, view: str, followup: int) -> str:
    """Build a synthetic clinical history string from patient metadata."""
    gender_str = "male" if gender.upper() == "M" else "female"
    view_str   = view.upper() if view.upper() in ("PA", "AP") else "unknown view"

    history = (
        f"Patient is a {age}-year-old {gender_str}. "
        f"Chest X-ray taken in {view_str} view. "
    )
    if followup == 0:
        history += "This is the initial presentation."
    else:
        history += f"Follow-up visit number {followup}."

    return history


KAGGLE_MODEL_DIR = "/kaggle/input/models/bleachiful/mv-model/other/default/1"


def resolve_checkpoint(path: str) -> str:
    """
    If path is a directory, find the first .pth file inside it.
    This handles the Kaggle model directory automatically.
    """
    if os.path.isdir(path):
        import glob
        pth_files = sorted(glob.glob(os.path.join(path, "*.pth")))
        if not pth_files:
            raise FileNotFoundError(f"No .pth files found in directory: {path}")
        if len(pth_files) > 1:
            print(f"Multiple checkpoints found, using: {os.path.basename(pth_files[0])}")
            for f in pth_files:
                print(f"  {f}")
        return pth_files[0]
    return path


def parse_args():
    p = argparse.ArgumentParser(description="Demo: run one X-ray through the model")
    p.add_argument("--image",      required=True,  help="Path to chest X-ray image")
    p.add_argument("--checkpoint", default=KAGGLE_MODEL_DIR,
                   help="Path to checkpoint file or directory (default: Kaggle model path)")
    p.add_argument("--age",        type=int,   default=50)
    p.add_argument("--gender",     default="M", choices=["M", "F"])
    p.add_argument("--view",       default="PA", choices=["PA", "AP"])
    p.add_argument("--followup",   type=int,   default=0)
    p.add_argument("--threshold",  type=float, default=0.5)
    p.add_argument("--backbone",   default="densenet", choices=["vit", "densenet"])
    p.add_argument("--device",     default=None)
    return p.parse_args()


@torch.no_grad()
def main():
    args = parse_args()

    device = torch.device(
        args.device if args.device
        else ("cuda" if torch.cuda.is_available() else "cpu")
    )
    print(f"Device: {device}\n")

    # ── Step 1: Build the clinical history string ─────────────────────────────
    history = build_history(args.age, args.gender, args.view, args.followup)
    print(f"Patient history: \"{history}\"\n")

    # ── Step 2: Load the model ────────────────────────────────────────────────
    ckpt_path = resolve_checkpoint(args.checkpoint)
    print(f"Loading model from: {ckpt_path}")
    model = MultimodalDiagnosticModel(
        visual_backbone=args.backbone,
        shared_dim=512,
        num_heads=8,
    ).to(device)

    checkpoint = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    print("Model loaded.\n")

    # ── Step 3: Preprocess the X-ray image ───────────────────────────────────
    image = Image.open(args.image).convert("RGB")
    image_tensor = TRANSFORM(image).unsqueeze(0).to(device)  # (1, 3, 224, 224)

    # ── Step 4: Tokenise the history text ────────────────────────────────────
    encoding = model.tokenize([history], device)

    # ── Step 5: Run the forward pass ─────────────────────────────────────────
    output = model(
        images=image_tensor,
        input_ids=encoding["input_ids"],
        attention_mask=encoding["attention_mask"],
        token_type_ids=encoding.get("token_type_ids"),
    )

    probs = output["probs"][0].cpu().tolist()    # list of 14 probabilities
    score = output["score"][0, 0].item()         # image-history compatibility score

    # ── Step 6: Print results ─────────────────────────────────────────────────
    print(f"Image–history compatibility score: {score:.4f}")
    print(f"  (1.0 = image and history strongly agree, 0.0 = conflict)\n")

    print(f"{'Disease':<25} {'Probability':>12}  {'Result':>8}")
    print("─" * 52)
    predicted = []
    for name, prob in zip(NIH_CLASSES, probs):
        result = "POSITIVE" if prob >= args.threshold else "negative"
        print(f"{name:<25} {prob:>12.4f}  {result:>8}")
        if prob >= args.threshold:
            predicted.append((name, prob))
    print("─" * 52)

    print()
    if predicted:
        print("Predicted conditions (above threshold):")
        for name, prob in sorted(predicted, key=lambda x: -x[1]):
            bar = "█" * int(prob * 20)
            print(f"  {name:<25} {prob:.4f}  {bar}")
    else:
        print("No conditions predicted above threshold.")


if __name__ == "__main__":
    main()
