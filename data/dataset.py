"""
NIH ChestXray14 Dataset with synthetic clinical history generation.

Expected directory layout
─────────────────────────
<data_root>/
    images/                  # all 112 120 PNG files
    Data_Entry_2017.csv      # official metadata CSV
    train_val_list.txt       # official train+val image names (one per line)
    test_list.txt            # official test image names (one per line)

Synthetic clinical history
──────────────────────────
The NIH dataset has no real free-text history, so we synthesise one from
available metadata:
  • age, gender, view position, follow-up number
  • for follow-up visits (Follow Up # > 0) we optionally append the
    disease labels from the *previous* visit as pseudo-history.
If any required field is missing the string "NO_HISTORY" is used instead.
"""

import os
import random
from typing import Optional

import pandas as pd
import torch
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms

from data.preprocess import NIH_CLASSES, normalize_labels, validate_image

# ── Default image transforms ──────────────────────────────────────────────────

def get_transforms(split: str = "train", image_size: int = 224) -> transforms.Compose:
    if split == "train":
        return transforms.Compose([
            transforms.Resize((image_size, image_size)),
            transforms.RandomHorizontalFlip(),
            transforms.ColorJitter(brightness=0.2, contrast=0.2),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])
    else:
        return transforms.Compose([
            transforms.Resize((image_size, image_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])


# ── History generation helpers ────────────────────────────────────────────────

def _gender_str(patient_gender: str) -> str:
    g = str(patient_gender).strip().upper()
    if g == "M":
        return "male"
    elif g == "F":
        return "female"
    return "patient"


def _view_str(view_position: str) -> str:
    v = str(view_position).strip().upper()
    if v in ("PA", "AP"):
        return v
    return "unknown view"


def build_clinical_history(row: pd.Series,
                           prior_labels: Optional[list[str]] = None) -> str:
    """
    Construct a synthetic free-text clinical history string from NIH metadata.

    Args:
        row:          A row from Data_Entry_2017.csv.
        prior_labels: Disease label strings from the patient's previous visit
                      (used only when Follow Up # > 0).

    Returns:
        A natural-language history string, or "NO_HISTORY" if data is absent.
    """
    try:
        age = int(row.get("Patient Age", 0))
        gender = _gender_str(row.get("Patient Gender", ""))
        view = _view_str(row.get("View Position", ""))
        followup = int(row.get("Follow-up #", 0))

        if age <= 0:
            return "NO_HISTORY"

        parts = [
            f"Patient is a {age}-year-old {gender}.",
            f"Chest X-ray taken in {view} view.",
        ]

        if followup == 0:
            parts.append("This is the initial presentation.")
        else:
            parts.append(f"Follow-up visit number {followup}.")

        if prior_labels:
            clean = [l for l in prior_labels if l != "No Finding"]
            if clean:
                parts.append(f"Previous findings: {', '.join(clean)}.")
            else:
                parts.append("No significant findings on previous visit.")

        return " ".join(parts)

    except Exception:
        return "NO_HISTORY"


# ── Dataset ───────────────────────────────────────────────────────────────────

class ChestXray14Dataset(Dataset):
    """
    Multi-label NIH ChestXray14 dataset with synthetic clinical history.

    Args:
        data_root:        Root directory (see layout above).
        split:            "train", "val", or "test".
        transform:        torchvision transform applied to images.
        image_size:       Resize target (used only when transform is None).
        use_prior_labels: Whether to include prior-visit labels in history.
        validate:         Skip corrupted images during __init__ (slow on HDD).
        max_samples:      Limit dataset size for quick smoke-tests.
    """

    def __init__(
        self,
        data_root: str,
        split: str = "train",
        transform=None,
        image_size: int = 224,
        use_prior_labels: bool = True,
        validate: bool = False,
        max_samples: Optional[int] = None,
    ):
        self.data_root = data_root
        self.image_dir = self._find_image_dirs(data_root)
        self.split = split
        self.transform = transform or get_transforms(split, image_size)
        self.use_prior_labels = use_prior_labels

        # ── Load metadata ──────────────────────────────────────────────────
        meta_path = os.path.join(data_root, "Data_Entry_2017.csv")
        self.meta = pd.read_csv(meta_path)
        self.meta.columns = self.meta.columns.str.strip()

        # ── Load split file ────────────────────────────────────────────────
        split_file = {
            "train": "train_val_list.txt",
            "val":   "train_val_list.txt",   # will sub-split below
            "test":  "test_list.txt",
        }[split]
        with open(os.path.join(data_root, split_file)) as f:
            names = [l.strip() for l in f if l.strip()]

        # For train/val we do an 80/20 split of the official train_val list
        if split in ("train", "val"):
            random.seed(42)
            random.shuffle(names)
            cut = int(0.8 * len(names))
            names = names[:cut] if split == "train" else names[cut:]

        # Filter to only images present in metadata
        meta_names = set(self.meta["Image Index"].tolist())
        names = [n for n in names if n in meta_names]

        if validate:
            names = [n for n in names
                     if validate_image(os.path.join(self.image_dir, n))]

        if max_samples is not None:
            names = names[:max_samples]

        self.names = names

        # ── Index metadata by image name for O(1) lookup ───────────────────
        self.meta_index = self.meta.set_index("Image Index")

        # ── Build prior-label lookup (patient_id → sorted visit rows) ─────
        self._prior_map: dict[str, list] = {}
        if use_prior_labels:
            grouped = self.meta.groupby("Patient ID")
            for pid, grp in grouped:
                sorted_rows = grp.sort_values("Follow-up #").to_dict("records")
                self._prior_map[str(pid)] = sorted_rows

    # ── Static helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _find_image_dirs(data_root: str) -> dict[str, str]:
        """
        Build a mapping  {image_filename: full_path}  by scanning all
        images_XXX/images/ subdirectories (NIH split-archive layout).
        Falls back to a single data_root/images/ folder if no subdirs found.
        """
        import glob
        mapping: dict[str, str] = {}

        # Pattern: data_root/images_*/images/*.png
        pattern = os.path.join(data_root, "images_*", "images", "*.png")
        for path in glob.glob(pattern):
            mapping[os.path.basename(path)] = path

        if not mapping:
            # Flat layout fallback
            flat_dir = os.path.join(data_root, "images")
            if os.path.isdir(flat_dir):
                for fname in os.listdir(flat_dir):
                    mapping[fname] = os.path.join(flat_dir, fname)

        return mapping

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_prior_labels(self, row: pd.Series) -> Optional[list[str]]:
        followup = int(row.get("Follow-up #", 0))
        if not self.use_prior_labels or followup == 0:
            return None

        pid = str(row.get("Patient ID", ""))
        visits = self._prior_map.get(pid, [])
        current_fu = followup
        prior = [v for v in visits if int(v.get("Follow-up #", 0)) < current_fu]
        if not prior:
            return None

        last = prior[-1]
        raw = last.get("Finding Labels", "No Finding")
        if not isinstance(raw, str):
            return None
        return [l.strip() for l in raw.split("|")]

    # ── Dataset interface ─────────────────────────────────────────────────────

    def __len__(self) -> int:
        return len(self.names)

    def __getitem__(self, idx: int) -> dict:
        img_name = self.names[idx]
        row = self.meta_index.loc[img_name]

        # Handle duplicate index (multiple rows with same image name)
        if isinstance(row, pd.DataFrame):
            row = row.iloc[0]

        # ── Image ──────────────────────────────────────────────────────────
        img_path = self.image_dir.get(img_name, "")
        try:
            image = Image.open(img_path).convert("RGB")
        except Exception:
            image = Image.new("RGB", (224, 224), color=0)

        image = self.transform(image)

        # ── Labels ─────────────────────────────────────────────────────────
        finding_labels = row.get("Finding Labels", "No Finding")
        labels = torch.tensor(normalize_labels(finding_labels), dtype=torch.float32)

        # ── Clinical history ───────────────────────────────────────────────
        prior = self._get_prior_labels(row)
        history_text = build_clinical_history(row, prior_labels=prior)

        return {
            "image":        image,           # (3, H, W) float tensor
            "labels":       labels,          # (14,) float tensor
            "history_text": history_text,    # str
            "image_name":   img_name,        # str, for traceability
        }
