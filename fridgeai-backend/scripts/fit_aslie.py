"""
fit_aslie.py — Fit ASLIE beta coefficients from the Mendeley dataset.

Usage:
    py scripts/fit_aslie.py [--csv PATH] [--update-config]

Features used:  Temp, Humidity, category_enc (normalised to [0,1])
Features ignored: Light (Lux), CO2 (ppm) — impractical for mixed-fridge use

WHY NORMALISE?
  The Mendeley dataset covers Temp 21-27 degC and Humidity 71-95%.
  A fridge operates at 0-10 degC and 30-80% humidity — a very different regime.
  Fitting on raw units produces extreme coefficients (B0=-247, B4=2.36) that
  make P_spoil always ~0 at fridge conditions.
  Normalising both features and inference inputs to [0,1] using fixed reference
  ranges keeps the model numerically stable across all operating conditions.

REFERENCE RANGES (stored in config.py, applied in aslie.py at inference time):
  Temp:     0–30 degC     (covers fridge + room temp + dataset range)
  Humidity: 0–100 %
  Category: 1–8  (ordinal encoding)

NOTE: B1 (time coefficient) cannot be fitted — the dataset has no elapsed-time
column. It remains at its heuristic value (0.18 per day).
"""

import argparse
import csv
import sys
import math
from pathlib import Path

# Fruit -> ASLIE category encoding
# dairy=1 protein=2 meat=3 vegetable=4 fruit=5 fish=6 cooked=7 beverage=8
FRUIT_TO_CATEGORY = {
    "banana":    5,
    "orange":    5,
    "pineapple": 5,
    "tomato":    4,
}

# Fixed reference ranges for [0,1] normalisation
TEMP_MIN,  TEMP_MAX  = 0.0,  30.0
HUMID_MIN, HUMID_MAX = 0.0, 100.0
CAT_MIN,   CAT_MAX   = 1.0,   8.0


def _norm(x, xmin, xmax):
    return (x - xmin) / (xmax - xmin)


try:
    import numpy as np
    from sklearn.linear_model import LogisticRegression
    from sklearn.model_selection import train_test_split
    from sklearn.metrics import classification_report, roc_auc_score
except ImportError:
    print("ERROR: scikit-learn and numpy required.  py -m pip install scikit-learn numpy")
    sys.exit(1)


def load_dataset(csv_path: str):
    rows = []
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            fruit   = row["Fruit"].strip()
            cat_enc = FRUIT_TO_CATEGORY.get(fruit.lower(), 5)
            spoiled = 1 if row["Class"].strip().lower() == "bad" else 0
            rows.append({
                "temp":     float(row["Temp"]),
                "humidity": float(row["Humid (%)"]),
                "cat_enc":  cat_enc,
                "spoiled":  spoiled,
            })
    return rows


def fit(csv_path: str):
    rows = load_dataset(csv_path)
    print(f"Loaded {len(rows)} rows")
    print(f"Temp:     {min(r['temp'] for r in rows):.0f}-{max(r['temp'] for r in rows):.0f} degC")
    print(f"Humidity: {min(r['humidity'] for r in rows):.0f}-{max(r['humidity'] for r in rows):.0f} %")
    print(f"Classes:  Good={sum(1 for r in rows if r['spoiled']==0)}  Bad={sum(1 for r in rows if r['spoiled']==1)}")

    # Normalise to [0,1] using fixed reference ranges
    X = np.array([
        [_norm(r["temp"], TEMP_MIN, TEMP_MAX),
         _norm(r["humidity"], HUMID_MIN, HUMID_MAX),
         _norm(r["cat_enc"], CAT_MIN, CAT_MAX)]
        for r in rows
    ])
    y = np.array([r["spoiled"] for r in rows])

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=y
    )

    model = LogisticRegression(max_iter=2000, solver="lbfgs")
    model.fit(X_train, y_train)

    b0       = float(model.intercept_[0])
    b2_temp  = float(model.coef_[0][0])   # temperature (normalised)
    b4_humid = float(model.coef_[0][1])   # humidity    (normalised)
    b3_cat   = float(model.coef_[0][2])   # category    (normalised)

    y_pred  = model.predict(X_test)
    y_proba = model.predict_proba(X_test)[:, 1]
    auc     = roc_auc_score(y_test, y_proba)

    print("\n" + "="*58)
    print("FITTED ASLIE COEFFICIENTS  (features normalised to [0,1])")
    print("="*58)
    print(f"  B0  (intercept):                    {b0:+.4f}")
    print(f"  B1  (time, days):                    0.1800  [heuristic]")
    print(f"  B2  (temp, norm to {TEMP_MIN:.0f}-{TEMP_MAX:.0f} degC):     {b2_temp:+.4f}")
    print(f"  B3  (category, norm to {CAT_MIN:.0f}-{CAT_MAX:.0f}):       {b3_cat:+.4f}")
    print(f"  B4  (humidity, norm to {HUMID_MIN:.0f}-{HUMID_MAX:.0f}%):    {b4_humid:+.4f}")
    print(f"  theta:                               0.7500")
    print("="*58)
    print(f"\nTest-set metrics  (n={len(y_test)}):")
    print(classification_report(y_test, y_pred, target_names=["Good", "Bad"]))
    print(f"ROC-AUC: {auc:.4f}")

    # Sanity checks at inference time (using normalised inputs)
    checks = [
        ("Fresh dairy,  4degC, 50% RH, t=0",  0,  4.0, 50.0, 1),
        ("Fresh fruit, 25degC, 90% RH, t=0",  0, 25.0, 90.0, 5),
        ("Old fish,    25degC, 90% RH, t=10", 10, 25.0, 90.0, 6),
    ]
    print("\nSanity checks (B1*t uses heuristic 0.18):")
    for label, t, temp, humid, cat in checks:
        temp_n  = _norm(temp,  TEMP_MIN,  TEMP_MAX)
        humid_n = _norm(humid, HUMID_MIN, HUMID_MAX)
        cat_n   = _norm(cat,   CAT_MIN,   CAT_MAX)
        logit   = b0 + 0.18*t + b2_temp*temp_n + b3_cat*cat_n + b4_humid*humid_n
        p       = 1 / (1 + math.exp(-max(-500, min(500, logit))))
        print(f"  {label:40s}  P_spoil={p:.3f}")

    # RSL sanity: how many days to reach theta for fresh dairy at 4degC, 50% RH?
    temp_n  = _norm(4.0,  TEMP_MIN,  TEMP_MAX)
    humid_n = _norm(50.0, HUMID_MIN, HUMID_MAX)
    cat_n   = _norm(1.0,  CAT_MIN,   CAT_MAX)
    base = b0 + b2_temp*temp_n + b3_cat*cat_n + b4_humid*humid_n
    # Solve: base + 0.18*t = logit(0.75) = 1.0986
    days_to_threshold = (1.0986 - base) / 0.18 if 0.18 != 0 else float('inf')
    print(f"\n  Days to reach P_spoil=0.75 (dairy, 4degC, 50%RH): {days_to_threshold:.1f} days")

    return b0, b2_temp, b3_cat, b4_humid


def update_config(b0, b2, b3, b4, config_path: str):
    with open(config_path, "r") as f:
        lines = f.readlines()

    new_lines = []
    replacements = {
        "BETA_0": f"BETA_0: float = {b0:.4f}",
        "BETA_2": f"BETA_2: float = {b2:.4f}",
        "BETA_3": f"BETA_3: float = {b3:.4f}",
        "BETA_4": f"BETA_4: float = {b4:.4f}",
    }
    for line in lines:
        replaced = False
        for key, new_val in replacements.items():
            if line.strip().startswith(key + ":"):
                comment = ("  " + line[line.index("#"):].rstrip()) if "#" in line else ""
                new_lines.append(new_val + comment + "\n")
                replaced = True
                break
        if not replaced:
            new_lines.append(line)

    with open(config_path, "w") as f:
        f.writelines(new_lines)

    print(f"\nUpdated {config_path}")
    print("NOTE: aslie.py normalises inputs using TEMP_REF/HUMID_REF/CAT_REF")
    print("      before applying these coefficients.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="../Mendeley_dataset.csv")
    parser.add_argument("--update-config", action="store_true")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        csv_path = Path(__file__).parent.parent.parent / "Mendeley_dataset.csv"
    if not csv_path.exists():
        print(f"ERROR: dataset not found at {args.csv}")
        sys.exit(1)

    b0, b2, b3, b4 = fit(str(csv_path))

    if args.update_config:
        config_path = Path(__file__).parent.parent / "core" / "config.py"
        update_config(b0, b2, b3, b4, str(config_path))
    else:
        print("\nRerun with --update-config to write to core/config.py")
