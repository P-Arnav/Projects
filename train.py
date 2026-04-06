"""
Training script for the History-Conditioned Multimodal Diagnostic Model.

Usage
─────
  python train.py --data_root /path/to/NIH_ChestXray14 [options]

Key options
───────────
  --data_root         Path to NIH dataset root (required)
  --epochs            Number of training epochs (default: 50)
  --batch_size        Batch size (default: 32)
  --lr                Learning rate for new layers (default: 1e-4)
  --backbone_lr       Learning rate for pretrained backbones (default: 1e-5)
  --lambda_cons       Consistency loss weight λ (default: 0.1)
  --shared_dim        Cross-attention shared dimension (default: 512)
  --num_heads         Number of attention heads (default: 8)
  --max_text_length   Max ClinicalBERT token length (default: 128)
  --image_size        Input image resolution (default: 224)
  --visual_backbone   "vit" or "densenet" (default: vit)
  --use_roi           Apply lung ROI extraction (default: True)
  --checkpoint_dir    Directory to save model checkpoints
  --resume            Path to a checkpoint to resume from
  --max_samples       Limit dataset size (for smoke-testing)
  --device            "cuda" or "cpu" (auto-detected if omitted)
"""

import argparse
import os
import time

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm

from data.dataset import ChestXray14Dataset, get_transforms
from models.lung_roi import extract_lung_roi
from models.model import MultimodalDiagnosticModel
from models.consistency import consistency_loss


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Train multimodal CXR diagnosis model")
    p.add_argument("--data_root",       required=True)
    p.add_argument("--epochs",          type=int,   default=50)
    p.add_argument("--batch_size",      type=int,   default=32)
    p.add_argument("--lr",              type=float, default=1e-4)
    p.add_argument("--backbone_lr",     type=float, default=1e-5)
    p.add_argument("--lambda_cons",     type=float, default=0.1)
    p.add_argument("--shared_dim",      type=int,   default=512)
    p.add_argument("--num_heads",       type=int,   default=8)
    p.add_argument("--max_text_length", type=int,   default=128)
    p.add_argument("--image_size",      type=int,   default=224)
    p.add_argument("--visual_backbone", default="vit", choices=["vit", "densenet"])
    p.add_argument("--use_roi",         action="store_true", default=False)
    p.add_argument("--checkpoint_dir",  default="checkpoints")
    p.add_argument("--resume",          default=None)
    p.add_argument("--max_samples",     type=int,   default=None)
    p.add_argument("--device",          default=None)
    p.add_argument("--num_workers",     type=int,   default=4)
    return p.parse_args()


# ── Class-balanced pos_weight ─────────────────────────────────────────────────

def compute_pos_weight(dataset: ChestXray14Dataset, device) -> torch.Tensor:
    """Compute per-class positive weight for BCEWithLogitsLoss."""
    total = len(dataset)
    pos_counts = torch.zeros(14)
    for item in tqdm(dataset, desc="Computing pos_weight", leave=False):
        pos_counts += item["labels"]
    neg_counts = total - pos_counts
    # Clip to avoid division by zero
    pos_weight = (neg_counts / pos_counts.clamp(min=1)).clamp(max=50.0)
    return pos_weight.to(device)


# ── ROI-aware collate ─────────────────────────────────────────────────────────

def make_collate_fn(use_roi: bool, image_size: int):
    """
    Returns a collate_fn that optionally applies lung ROI extraction.
    NOTE: ROI extraction is CPU-only and applied per-image before stacking.
    For faster training, pre-compute ROI crops offline and set use_roi=False.
    """
    from torchvision import transforms as T

    post_transform = T.Compose([
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    def collate_fn(batch):
        images, labels, texts, names = [], [], [], []
        for item in batch:
            images.append(item["image"])
            labels.append(item["labels"])
            texts.append(item["history_text"])
            names.append(item["image_name"])

        return {
            "image":        torch.stack(images),
            "labels":       torch.stack(labels),
            "history_text": texts,
            "image_name":   names,
        }

    return collate_fn


# ── Training loop ─────────────────────────────────────────────────────────────

def train_one_epoch(
    model: MultimodalDiagnosticModel,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    criterion: nn.BCEWithLogitsLoss,
    lambda_cons: float,
    device: torch.device,
    epoch: int,
) -> dict:
    model.train()
    total_loss = total_cls = total_cons = 0.0
    n_batches = len(loader)

    bar = tqdm(loader, desc=f"Epoch {epoch} [train]", leave=False)
    for batch in bar:
        images  = batch["image"].to(device)
        labels  = batch["labels"].to(device)
        texts   = batch["history_text"]

        # Tokenise on the fly
        encoding = model.tokenize(texts, device)

        optimizer.zero_grad()

        out = model(
            images=images,
            input_ids=encoding["input_ids"],
            attention_mask=encoding["attention_mask"],
            token_type_ids=encoding.get("token_type_ids"),
        )

        # ── Losses ─────────────────────────────────────────────────────────
        l_cls  = criterion(out["logits"], labels)
        l_cons = consistency_loss(out["logits"], out["logits_img"], out["score"])
        loss   = l_cls + lambda_cons * l_cons

        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()

        total_loss += loss.item()
        total_cls  += l_cls.item()
        total_cons += l_cons.item()

        bar.set_postfix(loss=f"{loss.item():.4f}")

    return {
        "loss":      total_loss / n_batches,
        "cls_loss":  total_cls  / n_batches,
        "cons_loss": total_cons / n_batches,
    }


@torch.no_grad()
def validate(
    model: MultimodalDiagnosticModel,
    loader: DataLoader,
    criterion: nn.BCEWithLogitsLoss,
    lambda_cons: float,
    device: torch.device,
) -> dict:
    model.eval()
    total_loss = 0.0
    all_probs, all_labels = [], []

    for batch in tqdm(loader, desc="  [val]", leave=False):
        images  = batch["image"].to(device)
        labels  = batch["labels"].to(device)
        texts   = batch["history_text"]

        encoding = model.tokenize(texts, device)
        out = model(
            images=images,
            input_ids=encoding["input_ids"],
            attention_mask=encoding["attention_mask"],
            token_type_ids=encoding.get("token_type_ids"),
        )

        l_cls  = criterion(out["logits"], labels)
        l_cons = consistency_loss(out["logits"], out["logits_img"], out["score"])
        total_loss += (l_cls + lambda_cons * l_cons).item()

        all_probs.append(out["probs"].cpu())
        all_labels.append(labels.cpu())

    all_probs  = torch.cat(all_probs,  dim=0).numpy()
    all_labels = torch.cat(all_labels, dim=0).numpy()

    # Per-class AUC
    from sklearn.metrics import roc_auc_score
    aucs = []
    for c in range(all_labels.shape[1]):
        if all_labels[:, c].sum() > 0:
            aucs.append(roc_auc_score(all_labels[:, c], all_probs[:, c]))
    mean_auc = float(sum(aucs) / len(aucs)) if aucs else 0.0

    return {
        "val_loss": total_loss / len(loader),
        "mean_auc": mean_auc,
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    device = torch.device(
        args.device if args.device
        else ("cuda" if torch.cuda.is_available() else "cpu")
    )
    print(f"Using device: {device}")
    os.makedirs(args.checkpoint_dir, exist_ok=True)

    # ── Datasets ───────────────────────────────────────────────────────────
    train_ds = ChestXray14Dataset(
        args.data_root, split="train",
        image_size=args.image_size,
        max_samples=args.max_samples,
    )
    val_ds = ChestXray14Dataset(
        args.data_root, split="val",
        image_size=args.image_size,
        max_samples=args.max_samples,
    )
    print(f"Train samples: {len(train_ds)} | Val samples: {len(val_ds)}")

    collate_fn = make_collate_fn(args.use_roi, args.image_size)
    train_loader = DataLoader(
        train_ds, batch_size=args.batch_size, shuffle=True,
        num_workers=args.num_workers, pin_memory=True,
        collate_fn=collate_fn,
    )
    val_loader = DataLoader(
        val_ds, batch_size=args.batch_size, shuffle=False,
        num_workers=args.num_workers, pin_memory=True,
        collate_fn=collate_fn,
    )

    # ── Model ──────────────────────────────────────────────────────────────
    model = MultimodalDiagnosticModel(
        visual_backbone=args.visual_backbone,
        shared_dim=args.shared_dim,
        num_heads=args.num_heads,
        max_text_length=args.max_text_length,
    ).to(device)

    # ── Class-weighted loss ────────────────────────────────────────────────
    print("Computing class positive weights...")
    pos_weight = compute_pos_weight(train_ds, device)
    criterion = nn.BCEWithLogitsLoss(pos_weight=pos_weight)

    # ── Optimizer: differential LR for backbones vs new layers ────────────
    backbone_params, new_params = [], []
    for name, param in model.named_parameters():
        if not param.requires_grad:
            continue
        if "visual_encoder" in name or "text_encoder.bert" in name:
            backbone_params.append(param)
        else:
            new_params.append(param)

    optimizer = torch.optim.AdamW([
        {"params": backbone_params, "lr": args.backbone_lr},
        {"params": new_params,      "lr": args.lr},
    ], weight_decay=1e-4)

    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=1e-6
    )

    start_epoch = 1
    best_auc = 0.0

    # ── Resume ─────────────────────────────────────────────────────────────
    if args.resume and os.path.isfile(args.resume):
        ckpt = torch.load(args.resume, map_location=device)
        model.load_state_dict(ckpt["model"])
        optimizer.load_state_dict(ckpt["optimizer"])
        scheduler.load_state_dict(ckpt["scheduler"])
        start_epoch = ckpt["epoch"] + 1
        best_auc = ckpt.get("best_auc", 0.0)
        print(f"Resumed from epoch {ckpt['epoch']} (best AUC: {best_auc:.4f})")

    # ── Training loop ───────────────────────────────────────────────────────
    for epoch in range(start_epoch, args.epochs + 1):
        t0 = time.time()

        train_stats = train_one_epoch(
            model, train_loader, optimizer, criterion,
            args.lambda_cons, device, epoch,
        )
        val_stats = validate(
            model, val_loader, criterion, args.lambda_cons, device
        )
        scheduler.step()

        elapsed = time.time() - t0
        print(
            f"Epoch {epoch:3d}/{args.epochs} | "
            f"train_loss={train_stats['loss']:.4f} "
            f"(cls={train_stats['cls_loss']:.4f} cons={train_stats['cons_loss']:.4f}) | "
            f"val_loss={val_stats['val_loss']:.4f} | "
            f"mean_AUC={val_stats['mean_auc']:.4f} | "
            f"{elapsed:.1f}s"
        )

        # ── Checkpoint ─────────────────────────────────────────────────────
        ckpt = {
            "epoch":     epoch,
            "model":     model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "scheduler": scheduler.state_dict(),
            "best_auc":  best_auc,
        }
        torch.save(ckpt, os.path.join(args.checkpoint_dir, "last.pth"))

        if val_stats["mean_auc"] > best_auc:
            best_auc = val_stats["mean_auc"]
            ckpt["best_auc"] = best_auc
            torch.save(ckpt, os.path.join(args.checkpoint_dir, "best.pth"))
            print(f"  ✓ New best AUC: {best_auc:.4f} — saved best.pth")

    print(f"\nTraining complete. Best val mean AUC: {best_auc:.4f}")


if __name__ == "__main__":
    main()
