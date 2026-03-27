"""
scorer.py — orchestrates ASLIE + FAPF, writes results to DB, fires alerts.
Called by settle_timer after the 30-minute settle delay.
"""

import logging
from datetime import datetime, timezone
from uuid import uuid4

from core.config import ALERT_CRITICAL, ALERT_WARNING, ALERT_USE_TODAY, CATEGORY_ENC
from core.database import get_db
from services import aslie, fapf, paif, corrections
from websocket.manager import manager

logger = logging.getLogger(__name__)


async def run_for_item(item_id: str) -> None:
    async with get_db() as conn:
        row = await conn.fetchrow("SELECT * FROM items WHERE item_id = $1", item_id)
        if row is None:
            logger.warning("scorer: item %s not found (deleted before settle?)", item_id)
            return

        entry_time = datetime.fromisoformat(row["entry_time"])
        now_utc = datetime.now(tz=timezone.utc)
        elapsed_days = (now_utc - entry_time.replace(tzinfo=timezone.utc)).total_seconds() / 86400.0

        category_enc = CATEGORY_ENC.get(row["category"], 4)  # default: vegetable

        # Apply adaptive correction learned from user feedback for this category
        correction_days = await corrections.get_correction(conn, row["category"])
        effective_shelf_life = max(1, round(row["shelf_life"] + correction_days))

        ps, remaining = aslie.compute(
            t_elapsed=elapsed_days,
            temp=row["storage_temp"],
            shelf_life=effective_shelf_life,
            category_enc=category_enc,
            humidity=row["humidity"],
        )

        # Cost normalisation across current registry
        cost_rows = await conn.fetch("SELECT estimated_cost FROM items")
        costs = [r["estimated_cost"] for r in cost_rows]
        min_c = min(costs) if costs else 0.0
        max_c = max(costs) if costs else 1.0
        cost_norm = (row["estimated_cost"] - min_c) / (max_c - min_c + 1e-9)

        s = fapf.score(ps, cost_norm, row["category"])
        action = paif.recommend(ps, remaining, row["category"])

        now_iso = now_utc.isoformat()
        await conn.execute(
            "UPDATE items SET p_spoil=$1, rsl=$2, fapf_score=$3, paif_action=$4, updated_at=$5 WHERE item_id=$6",
            ps, remaining, s, action, now_iso, item_id,
        )

        await manager.broadcast({
            "event": "ITEM_SCORED",
            "timestamp": now_iso,
            "data": {
                "item_id": item_id,
                "P_spoil": round(ps, 4),
                "RSL": round(remaining, 4),
                "fapf_score": round(s, 4),
                "paif_action": action,
            },
        })

        # Alert evaluation
        alerts: list[tuple[str, str]] = []
        if ps > ALERT_CRITICAL:
            alerts.append(("CRITICAL_ALERT", f"P_spoil={ps:.2f} exceeds critical threshold {ALERT_CRITICAL}"))
        elif ps > ALERT_WARNING:
            alerts.append(("WARNING_ALERT", f"P_spoil={ps:.2f} exceeds warning threshold {ALERT_WARNING}"))
        if remaining < ALERT_USE_TODAY:
            alerts.append(("USE_TODAY_ALERT", f"RSL={remaining:.2f} d — use today"))

        for alert_type, message in alerts:
            alert_id = str(uuid4())
            await conn.execute(
                """INSERT INTO alerts
                   (alert_id, item_id, item_name, alert_type, p_spoil, rsl, message, created_at)
                   VALUES ($1,$2,$3,$4,$5,$6,$7,$8)""",
                alert_id, item_id, row["name"], alert_type, ps, remaining, message, now_iso,
            )

            await manager.broadcast({
                "event": "ALERT_FIRED",
                "timestamp": now_iso,
                "data": {
                    "item_id": item_id,
                    "item_name": row["name"],
                    "alert_type": alert_type,
                    "P_spoil": round(ps, 4),
                    "RSL": round(remaining, 4),
                    "message": message,
                },
            })
            logger.info("Alert fired: %s for item %s (%s)", alert_type, item_id, row["name"])
