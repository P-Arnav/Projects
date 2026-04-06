"""
Lung ROI extraction using torchxrayvision's pretrained segmentation model.

Falls back to returning the full image if segmentation fails (e.g. model
weights not yet downloaded or GPU OOM).
"""

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from torchvision import transforms


def _load_seg_model():
    """Lazy-load the torchxrayvision PSPNet lung segmentation model."""
    try:
        import torchxrayvision as xrv
        model = xrv.baseline_models.chestx_det.PSPNet()
        model.eval()
        return model
    except Exception as e:
        print(f"[lung_roi] Could not load torchxrayvision seg model: {e}")
        return None


# Module-level singleton — loaded once per process
_SEG_MODEL = None


def _get_seg_model():
    global _SEG_MODEL
    if _SEG_MODEL is None:
        _SEG_MODEL = _load_seg_model()
    return _SEG_MODEL


# ── Pre/post-processing ───────────────────────────────────────────────────────

_XRV_TRANSFORM = transforms.Compose([
    transforms.Grayscale(num_output_channels=1),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    # torchxrayvision expects values in [-1024, 1024]
    transforms.Lambda(lambda x: x * 2048.0 - 1024.0),
])


def _pil_to_xrv_tensor(pil_image: Image.Image) -> torch.Tensor:
    """Convert PIL RGB image to the (1, 1, 224, 224) tensor expected by XRV."""
    t = _XRV_TRANSFORM(pil_image)   # (1, 224, 224)
    return t.unsqueeze(0)            # (1, 1, 224, 224)


def _mask_to_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
    """
    Compute a tight bounding box around all positive pixels in a binary mask.
    Returns (x_min, y_min, x_max, y_max) in pixel coordinates.
    Falls back to the full image extent if no positive pixels are found.
    """
    h, w = mask.shape
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    if not rows.any():
        return 0, 0, w, h
    y_min, y_max = np.where(rows)[0][[0, -1]]
    x_min, x_max = np.where(cols)[0][[0, -1]]
    # Add a small margin (5 %)
    margin_y = max(1, int(0.05 * (y_max - y_min)))
    margin_x = max(1, int(0.05 * (x_max - x_min)))
    y_min = max(0, y_min - margin_y)
    y_max = min(h, y_max + margin_y)
    x_min = max(0, x_min - margin_x)
    x_max = min(w, x_max + margin_x)
    return int(x_min), int(y_min), int(x_max), int(y_max)


# ── Public API ────────────────────────────────────────────────────────────────

def extract_lung_roi(
    pil_image: Image.Image,
    output_size: int = 224,
    device: str = "cpu",
) -> Image.Image:
    """
    Extract the lung region from a chest X-ray PIL image.

    Strategy:
      1. Run XRV PSPNet to predict a lung segmentation mask.
      2. Threshold at 0.5 → binary mask.
      3. Compute bounding box, crop, resize to output_size × output_size.

    Falls back to a plain resize if segmentation is unavailable or fails.

    Args:
        pil_image:   Input chest X-ray as a PIL RGB image.
        output_size: Target spatial size of the returned crop.
        device:      "cpu" or "cuda".

    Returns:
        PIL RGB image of shape (output_size, output_size).
    """
    seg_model = _get_seg_model()
    if seg_model is None:
        return pil_image.resize((output_size, output_size), Image.BILINEAR)

    try:
        seg_model = seg_model.to(device)
        xrv_tensor = _pil_to_xrv_tensor(pil_image).to(device)

        with torch.no_grad():
            pred = seg_model(xrv_tensor)  # (1, num_classes, H, W)

        # XRV PSPNet returns predictions for several structures; lung labels
        # are indices 4 & 5 ("Left Lung" and "Right Lung").
        # We combine both into one binary mask.
        pred = torch.sigmoid(pred)
        try:
            lung_mask = (pred[0, 4] + pred[0, 5]).clamp(0, 1)
        except IndexError:
            # Fallback: take the max across all channels
            lung_mask = pred[0].max(dim=0).values

        # Resize mask back to original image dimensions
        orig_w, orig_h = pil_image.size
        lung_mask_resized = F.interpolate(
            lung_mask.unsqueeze(0).unsqueeze(0),
            size=(orig_h, orig_w),
            mode="bilinear",
            align_corners=False,
        ).squeeze().cpu().numpy()

        binary_mask = (lung_mask_resized > 0.5).astype(np.uint8)
        x_min, y_min, x_max, y_max = _mask_to_bbox(binary_mask)

        cropped = pil_image.crop((x_min, y_min, x_max, y_max))
        return cropped.resize((output_size, output_size), Image.BILINEAR)

    except Exception as e:
        print(f"[lung_roi] Segmentation failed ({e}), using full image.")
        return pil_image.resize((output_size, output_size), Image.BILINEAR)
