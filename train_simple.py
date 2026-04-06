"""
Simplified training script for the Multimodal Chest X-ray Diagnosis model.

This is a clean, readable version of train.py — good for understanding
the training loop. For full options (differential LR, resume, ROI, etc.)
use train.py instead.

Usage
─────
  python train_simple.py --data_root /path/to/NIH_ChestXray14

Key options
───────────
  --data_root     Path to NIH dataset root (required)
  --epochs        Number of training epochs       (default: 20)
  --batch_size    Batch size                      (default: 16)
  --lr            Learning rate                   (default: 1e-4)
  --backbone      vit or densenet                 (default: densenet)
  --checkpoint    Where to save the best model    (default: best_model.pth)
  --max_samples   Limit dataset size for testing  (default: no limit)
"""

import argparse
import os

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm

from data.dataset import ChestXray14Dataset
from models.model import MultimodalDiagnosticModel
from models.consistency import consistency_loss


LAMBDA_CONS = 0.1   # consistency loss weight (λ from the paper)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--data_root",   required=True)
    p.add_argument("--epochs",      type=int,   default=20)
    p.add_argument("--batch_size",  type=int,   default=16)
    p.add_argument("--lr",          type=float, default=1e-4)
    p.add_argument("--backbone",    default="densenet", choices=["vit", "densenet"])
    p.add_argument("--checkpoint",  default="best_model.pth")
    p.add_argument("--max_samples", type=int,   default=None)
    p.add_argument("--num_workers", type=int,   default=2)
    return p.parse_args()


def train_one_epoch(model, loader, optimizer, criterion, device):
    model.train()
    total_loss = 0.0

    for batch in tqdm(loader, desc="  training", leave=False):
        images = batch["image"].to(device)
        labels = batch["labels"].to(device)
        texts  = batch["history_text"]

        # Tokenise the history strings
        encoding = model.tokenize(texts, device)

        # Forward pass
        out = model(
            images=images,
            input_ids=encoding["input_ids"],
            attention_mask=encoding["attention_mask"],
            token_type_ids=encoding.get("token_type_ids"),
        )

        # Loss = classification loss + consistency regularisation
        l_cls  = criterion(out["logits"], labels)
        l_cons = consistency_loss(out["logits"], out["logits_img"], out["score"])
        loss   = l_cls + LAMBDA_CONS * l_cons

        optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()

        total_loss += loss.item()

    return total_loss / len(loader)


@torch.no_grad()
def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    all_probs, all_labels = [], []

    for batch in tqdm(loader, desc="  evaluating", leave=False):
        images = batch["image"].to(device)
        labels = batch["labels"].to(device)
        texts  = batch["history_text"]

        encoding = model.tokenize(texts, device)
        out = model(
            images=images,
            input_ids=encoding["input_ids"],
            attention_mask=encoding["attention_mask"],
            token_type_ids=encoding.get("token_type_ids"),
        )

        l_cls  = criterion(out["logits"], labels)
        l_cons = consistency_loss(out["logits"], out["logits_img"], out["score"])
        total_loss += (l_cls + LAMBDA_CONS * l_cons).item()

        all_probs.append(out["probs"].cpu())
        all_labels.append(labels.cpu())

    # Compute mean AUC across all 14 disease classes
    from sklearn.metrics import roc_auc_score
    probs_np  = torch.cat(all_probs).numpy()
    labels_np = torch.cat(all_labels).numpy()

    aucs = []
    for c in range(labels_np.shape[1]):
        if labels_np[:, c].sum() > 0:   # skip classes with no positive samples
            aucs.append(roc_auc_score(labels_np[:, c], probs_np[:, c]))
    mean_auc = sum(aucs) / len(aucs) if aucs else 0.0

    return total_loss / len(loader), mean_auc


def main():
    args  = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # ── Datasets and DataLoaders ──────────────────────────────────────────────
    train_ds = ChestXray14Dataset(args.data_root, split="train",
                                  max_samples=args.max_samples)
    val_ds   = ChestXray14Dataset(args.data_root, split="val",
                                  max_samples=args.max_samples)

    train_loader = DataLoader(train_ds, batch_size=args.batch_size,
                              shuffle=True,  num_workers=args.num_workers,
                              pin_memory=True)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch_size,
                              shuffle=False, num_workers=args.num_workers,
                              pin_memory=True)

    print(f"Train: {len(train_ds)} samples | Val: {len(val_ds)} samples\n")

    # ── Model ─────────────────────────────────────────────────────────────────
    model = MultimodalDiagnosticModel(
        visual_backbone=args.backbone,
        shared_dim=512,
        num_heads=8,
    ).to(device)

    # ── Loss: class-weighted BCE to handle label imbalance ────────────────────
    # Simple version: equal weight per class. For weighted, use train.py.
    criterion = nn.BCEWithLogitsLoss()

    # ── Optimizer ─────────────────────────────────────────────────────────────
    optimizer = torch.optim.AdamW(
        filter(lambda p: p.requires_grad, model.parameters()),
        lr=args.lr, weight_decay=1e-4,
    )

    # ── Training loop ─────────────────────────────────────────────────────────
    best_auc = 0.0
    for epoch in range(1, args.epochs + 1):
        print(f"Epoch {epoch}/{args.epochs}")

        train_loss = train_one_epoch(model, train_loader, optimizer, criterion, device)
        val_loss, mean_auc = evaluate(model, val_loader, criterion, device)

        print(f"  train_loss={train_loss:.4f}  val_loss={val_loss:.4f}  mean_AUC={mean_auc:.4f}")

        # Save the best model based on validation AUC
        if mean_auc > best_auc:
            best_auc = mean_auc
            torch.save({"model": model.state_dict(), "epoch": epoch,
                        "auc": best_auc}, args.checkpoint)
            print(f"  --> New best AUC {best_auc:.4f} — saved to {args.checkpoint}")

    print(f"\nDone. Best validation AUC: {best_auc:.4f}")


if __name__ == "__main__":
    main()
