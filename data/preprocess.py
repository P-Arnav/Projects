"""
Data validation and label normalization utilities for NIH ChestXray14.
"""

import os
from PIL import Image

NIH_CLASSES = [
    "Atelectasis", "Cardiomegaly", "Effusion", "Infiltration",
    "Mass", "Nodule", "Pneumonia", "Pneumothorax",
    "Consolidation", "Edema", "Emphysema", "Fibrosis",
    "Pleural_Thickening", "Hernia",
]

# Map raw NIH label strings to canonical class names
_LABEL_ALIASES = {
    "pleural thickening": "Pleural_Thickening",
    "pleural_thickening": "Pleural_Thickening",
}


def normalize_labels(finding_labels_str: str) -> list[int]:
    """
    Convert the NIH 'Finding Labels' string (pipe-separated) to a 14-dim binary vector.

    Args:
        finding_labels_str: e.g. "Atelectasis|Effusion" or "No Finding"

    Returns:
        List of 14 ints (0 or 1), one per NIH_CLASSES entry.
    """
    vec = [0] * len(NIH_CLASSES)
    if not isinstance(finding_labels_str, str) or finding_labels_str.strip() == "No Finding":
        return vec

    raw_labels = [l.strip() for l in finding_labels_str.split("|")]
    for raw in raw_labels:
        canonical = _LABEL_ALIASES.get(raw.lower(), raw)
        if canonical in NIH_CLASSES:
            vec[NIH_CLASSES.index(canonical)] = 1
    return vec


def validate_image(image_path: str) -> bool:
    """
    Return True if the image file exists and can be opened without error.
    """
    if not os.path.isfile(image_path):
        return False
    try:
        with Image.open(image_path) as img:
            img.verify()
        return True
    except Exception:
        return False
