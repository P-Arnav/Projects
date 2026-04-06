"""
Evaluation script: AUC-ROC, mAP, and F1 on the NIH ChestXray14 test set.

Usage
─────
  python evaluate.py --data_root /path/to/NIH_ChestXray14 --checkpoint checkpoints/best.pth
"""

import argparse

import numpy as np
import torch
from torch.utils.data import DataLoader
from tqdm import tqdm

from data.dataset import ChestXray14Dataset
from data.preprocess import NIH_CLASSES
from models.model import MultimodalDiagnosticModel


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--data_root",       required=True)
    p.add_argument("--checkpoint",      required=True)
    p.add_argument("--batch_size",      type=int,   default=32)
    p.add_argument("--image_size",      type=int,   default=224)
    p.add_argument("--max_text_length", type=int,   default=128)
    p.add_argument("--shared_dim",      type=int,   default=512)
    p.add_argument("--num_heads",       type=int,   default=8)
    p.add_argument("--visual_backbone", default="vit")
    p.add_argument("--threshold",       type=float, default=0.5)
    p.add_argument("--device",          default=None)
    p.add_argument("--num_workers",     type=int,   default=4)
    return p.parse_args()


@torch.no_grad()
def run_inference(model, loader, device):
    model.eval()
    all_probs, all_labels = [], []

    for batch in tqdm(loader, desc="Inference"):
        images = batch["image"].to(device)
        labels = batch["labels"]
        texts  = batch["history_text"]

        encoding = model.tokenize(texts, device)
        out = model(
            images=images,
            input_ids=encoding["input_ids"],
            attention_mask=encoding["attention_mask"],
            token_type_ids=encoding.get("token_type_ids"),
        )
        all_probs.append(out["probs"].cpu().numpy())
        all_labels.append(labels.numpy())

    return np.concatenate(all_probs, axis=0), np.concatenate(all_labels, axis=0)


def compute_metrics(probs: np.ndarray, labels: np.ndarray, threshold: float = 0.5):
    from sklearn.metrics import (
        average_precision_score,
        f1_score,
        roc_auc_score,
    )

    aucs, aps, f1s = [], [], []
    per_class = {}

    for i, cls_name in enumerate(NIH_CLASSES):
        y_true = labels[:, i]
        y_score = probs[:, i]
        y_pred  = (y_score >= threshold).astype(int)

        if y_true.sum() == 0:
            continue

        auc = roc_auc_score(y_true, y_score)
        ap  = average_precision_score(y_true, y_score)
        f1  = f1_score(y_true, y_pred, zero_division=0)

        aucs.append(auc)
        aps.append(ap)
        f1s.append(f1)
        per_class[cls_name] = {"AUC": auc, "AP": ap, "F1": f1}

    return {
        "mean_AUC": float(np.mean(aucs)),
        "mAP":      float(np.mean(aps)),
        "mean_F1":  float(np.mean(f1s)),
        "per_class": per_class,
    }


def main():
    args = parse_args()
    device = torch.device(
        args.device if args.device
        else ("cuda" if torch.cuda.is_available() else "cpu")
    )
    print(f"Device: {device}")

    test_ds = ChestXray14Dataset(
        args.data_root, split="test", image_size=args.image_size
    )
    test_loader = DataLoader(
        test_ds, batch_size=args.batch_size, shuffle=False,
        num_workers=args.num_workers, pin_memory=True,
    )

    model = MultimodalDiagnosticModel(
        visual_backbone=args.visual_backbone,
        shared_dim=args.shared_dim,
        num_heads=args.num_heads,
        max_text_length=args.max_text_length,
    ).to(device)

    ckpt = torch.load(args.checkpoint, map_location=device)
    model.load_state_dict(ckpt["model"])
    print(f"Loaded checkpoint from epoch {ckpt.get('epoch', '?')}")

    probs, labels = run_inference(model, test_loader, device)
    metrics = compute_metrics(probs, labels, threshold=args.threshold)

    print(f"\n{'='*55}")
    print(f"  Mean AUC : {metrics['mean_AUC']:.4f}")
    print(f"  mAP      : {metrics['mAP']:.4f}")
    print(f"  Mean F1  : {metrics['mean_F1']:.4f}")
    print(f"{'='*55}")
    print(f"\n{'Class':<22} {'AUC':>6}  {'AP':>6}  {'F1':>6}")
    print("-" * 44)
    for cls_name, vals in metrics["per_class"].items():
        print(f"{cls_name:<22} {vals['AUC']:>6.4f}  {vals['AP']:>6.4f}  {vals['F1']:>6.4f}")


if __name__ == "__main__":
    main()
