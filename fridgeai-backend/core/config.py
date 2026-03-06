import os
from dotenv import load_dotenv

load_dotenv()

# Database
DB_PATH: str = os.getenv("DB_PATH", "db/fridgeai.sqlite")

# Settle timer — set to 5 in tests for fast iteration
SETTLE_DELAY_SECONDS: int = int(os.getenv("SETTLE_DELAY_SECONDS", "1800"))

# ASLIE coefficients
# B0, B2, B3, B4 fitted from Mendeley Multi-Parameter Fruit Spoilage dataset
#   (10,995 readings; features: Temp + Humidity; CO2/Light excluded)
#   Features are normalised to [0,1] using NORM_* ranges below before inference.
# B1 not in dataset (no elapsed-time column); recalibrated so that dairy at
#   4 degC / 50% RH reaches P_spoil=0.75 in approx 7 days.
BETA_0: float = -37.9506  # intercept
BETA_1: float = 3.40      # time (days) — recalibrated for fridge conditions
BETA_2: float = 17.0408   # temperature (normalised)
BETA_3: float = -0.0282   # category encoding (normalised)
BETA_4: float = 25.9930   # humidity (normalised)
THETA:  float = 0.75      # spoilage decision threshold

# Feature normalisation reference ranges applied in aslie.py before inference
# Chosen to cover both fridge (0-10 degC) and the Mendeley warm-storage regime (21-27 degC)
TEMP_NORM:  tuple = (0.0,  30.0)   # [min, max] degC
HUMID_NORM: tuple = (0.0, 100.0)   # [min, max] %
CAT_NORM:   tuple = (1.0,   8.0)   # [min, max] ordinal encoding

# Category ordinal encodings used by ASLIE
CATEGORY_ENC: dict[str, int] = {
    "dairy": 1, "protein": 2, "meat": 3, "vegetable": 4,
    "fruit": 5, "fish": 6, "cooked": 7, "beverage": 8,
}

# Alert thresholds
ALERT_CRITICAL:  float = 0.80
ALERT_WARNING:   float = 0.50
ALERT_USE_TODAY: float = 1.0   # RSL in days
