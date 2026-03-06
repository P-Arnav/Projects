"""
grounding_dino_test.py — Test Grounding DINO grocery detection + item counts.

Install deps:
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
    pip install transformers pillow opencv-python

Usage:
    # From an image file
    python grounding_dino_test.py --image path/to/fridge.jpg

    # From webcam (press Q to capture and run detection)
    python grounding_dino_test.py --webcam

    # Custom item list (dot-separated)
    python grounding_dino_test.py --image fridge.jpg --items "milk . eggs . apple . yogurt . cheese"

    # Adjust confidence threshold (default 0.3)
    python grounding_dino_test.py --image fridge.jpg --threshold 0.25
"""

import argparse
import sys
from collections import Counter

import cv2
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

# ── Default grocery prompt ────────────────────────────────────────────────────
DEFAULT_ITEMS = (
    "milk . eggs . apple . orange . banana . yogurt . cheese . butter . "
    "cream . juice . water . soda . chicken . beef . fish . carrot . "
    "tomato . lettuce . broccoli . onion . potato . garlic . pepper . "
    "strawberry . grape . lemon . bread . leftovers . container . bottle . "
    "jar . can . sauce . condiment . spice . powder . chocolate . snack"
)

MODEL_ID = "IDEA-Research/grounding-dino-base"


def load_model():
    print(f"Loading model '{MODEL_ID}' (first run downloads ~700 MB)…")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    processor = AutoProcessor.from_pretrained(MODEL_ID)
    model     = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL_ID).to(device)
    print(f"Model loaded on {device.upper()}.")
    return processor, model, device


def detect(pil_image, processor, model, device, items_text: str, threshold: float):
    inputs = processor(images=pil_image, text=items_text, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_grounded_object_detection(
        outputs,
        inputs.input_ids,
        threshold=threshold,
        text_threshold=threshold,
        target_sizes=[pil_image.size[::-1]],
    )[0]

    boxes  = results["boxes"].cpu().tolist()    # [[x1,y1,x2,y2], ...]
    scores = results["scores"].cpu().tolist()
    labels = results["labels"]                  # list of str

    return boxes, scores, labels


def draw_results(cv_image, boxes, scores, labels):
    """Draw bounding boxes and labels onto a cv2 image (BGR). Returns annotated copy."""
    img = cv_image.copy()
    colors = {}

    for box, score, label in zip(boxes, scores, labels):
        x1, y1, x2, y2 = (int(v) for v in box)

        # Assign a consistent colour per label
        if label not in colors:
            h = hash(label) % 179
            import colorsys
            r, g, b = (int(c * 255) for c in colorsys.hsv_to_rgb(h / 179, 0.85, 0.95))
            colors[label] = (b, g, r)   # BGR

        color = colors[label]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        text = f"{label}  {score:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(img, text, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    return img


def print_summary(labels, scores):
    print("\n" + "=" * 48)
    print("DETECTION SUMMARY")
    print("=" * 48)
    counts = Counter(labels)
    for item, count in sorted(counts.items(), key=lambda x: -x[1]):
        item_scores = [s for l, s in zip(labels, scores) if l == item]
        avg_conf = sum(item_scores) / len(item_scores)
        print(f"  {item:<22} x{count}   avg conf {avg_conf:.2f}")
    print("-" * 48)
    print(f"  Total detections: {len(labels)}")
    print(f"  Unique item types: {len(counts)}")
    print("=" * 48 + "\n")

    return dict(counts)


def run_on_image(path, processor, model, device, items_text, threshold):
    pil_image = Image.open(path).convert("RGB")
    cv_image  = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2BGR)   # keep BGR

    print(f"Image: {path}  ({pil_image.width}x{pil_image.height})")
    print(f"Running detection…")

    boxes, scores, labels = detect(pil_image, processor, model, device, items_text, threshold)

    if not labels:
        print("No items detected. Try lowering --threshold or adding more items to --items.")
        return

    counts = print_summary(labels, scores)
    annotated = draw_results(cv_image, boxes, scores, labels)

    window = "FridgeAI — Grounding DINO  (press any key to close)"
    cv2.imshow(window, annotated)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Optionally save
    out_path = path.rsplit(".", 1)[0] + "_detected.jpg"
    cv2.imwrite(out_path, annotated)
    print(f"Annotated image saved to: {out_path}")

    return counts


def run_on_webcam(cam_index, processor, model, device, items_text, threshold):
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE,      -5)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,    35)
    cap.set(cv2.CAP_PROP_CONTRAST,      70)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera index {cam_index}.")
        sys.exit(1)

    print("Webcam live feed — press SPACE to capture + detect, R to reset counter, Q to quit.")

    session_counter = Counter()
    capture_count   = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Overlay session totals on the live feed
        overlay = frame.copy()
        y = 28
        cv2.putText(overlay, f"Session captures: {capture_count}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 180), 2, cv2.LINE_AA)
        for item, count in session_counter.most_common():
            y += 24
            cv2.putText(overlay, f"  {item}: {count}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("FridgeAI — Webcam  (SPACE=detect  R=reset  Q=quit)", overlay)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        if key == ord('r'):
            session_counter.clear()
            capture_count = 0
            print("Counter reset.")

        if key == ord(' '):
            cv2.destroyAllWindows()
            capture_count += 1
            print(f"Captured #{capture_count}. Running detection…")

            pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            boxes, scores, labels = detect(pil_image, processor, model, device, items_text, threshold)

            if not labels:
                print("No items detected. Try lowering --threshold.")
            else:
                print_summary(labels, scores)
                for label in labels:
                    session_counter[label] += 1

                print("SESSION TOTALS:")
                for item, count in session_counter.most_common():
                    print(f"  {item}: {count}")

                annotated = draw_results(frame, boxes, scores, labels)
                cv2.imshow("FridgeAI — Detection result  (any key to return to camera)", annotated)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    cap.release()
    cv2.destroyAllWindows()

    if session_counter:
        print("\nFINAL SESSION TOTALS:")
        for item, count in session_counter.most_common():
            print(f"  {item}: {count}")


def main():
    parser = argparse.ArgumentParser(description="Grounding DINO grocery detection test")
    parser.add_argument("--image",  type=str, default=None, help="Path to an image file (omit to use webcam)")
    parser.add_argument("--cam",       type=int,   default=0,           help="Camera index (default 0)")
    parser.add_argument("--items",     type=str,   default=DEFAULT_ITEMS, help="Dot-separated item list")
    parser.add_argument("--threshold", type=float, default=0.5,          help="Detection confidence threshold (default 0.5, min 0.5)")
    parser.add_argument("--model",     type=str,   default=MODEL_ID,     help="HuggingFace model ID")
    args = parser.parse_args()

    processor, model, device = load_model()

    # Enforce minimum threshold
    if args.threshold < 0.5:
        print(f"WARNING: threshold {args.threshold} is below minimum 0.5, clamping to 0.5.")
        args.threshold = 0.5

    # Normalise prompt: ensure each token is separated by . and ends with .
    items_text = args.items.strip().rstrip(".").strip() + " ."

    if args.image:
        run_on_image(args.image, processor, model, device, items_text, args.threshold)
    else:
        run_on_webcam(args.cam, processor, model, device, items_text, args.threshold)  # default


if __name__ == "__main__":
    main()
