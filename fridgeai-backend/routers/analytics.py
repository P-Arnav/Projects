from __future__ import annotations
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends
import asyncpg
from pydantic import BaseModel

from core.database import db_dependency

router = APIRouter(prefix="/analytics", tags=["analytics"])


class ConsumptionPoint(BaseModel):
    date: str
    items_consumed: int
    total_quantity: int


class WastePattern(BaseModel):
    name: str
    category: str
    times_wasted: int
    avg_p_spoil_at_removal: Optional[float]


class WasteSummary(BaseModel):
    total_consumed: int
    total_wasted: int
    waste_rate_pct: float
    top_wasted: list[WastePattern]
    daily_trend: list[ConsumptionPoint]


@router.get("/consumption", response_model=list[ConsumptionPoint])
async def consumption_trend(
    days: int = 30,
    conn: asyncpg.Connection = Depends(db_dependency),
):
    """Daily consumption counts for the last N days."""
    rows = await conn.fetch(
        """
        SELECT DATE(consumed_at::timestamp) as date,
               COUNT(*) as items_consumed,
               SUM(quantity_consumed) as total_quantity
        FROM consumption_history
        WHERE consumed_at::timestamp >= NOW() - ($1 || ' days')::interval
          AND reason != 'wasted'
        GROUP BY DATE(consumed_at::timestamp)
        ORDER BY date ASC
        """,
        str(days),
    )
    return [
        ConsumptionPoint(
            date=str(row["date"]),
            items_consumed=row["items_consumed"],
            total_quantity=row["total_quantity"] or 0,
        )
        for row in rows
    ]


@router.get("/waste-patterns", response_model=list[WastePattern])
async def waste_patterns(conn: asyncpg.Connection = Depends(db_dependency)):
    """Items most frequently wasted/expired."""
    rows = await conn.fetch(
        """
        SELECT item_name, category,
               COUNT(*) as times_wasted,
               AVG(p_spoil_at_removal) as avg_p_spoil
        FROM consumption_history
        WHERE reason = 'wasted'
        GROUP BY LOWER(item_name), category
        ORDER BY times_wasted DESC
        LIMIT 20
        """
    )
    return [
        WastePattern(
            name=row["item_name"],
            category=row["category"],
            times_wasted=row["times_wasted"],
            avg_p_spoil_at_removal=float(row["avg_p_spoil"]) if row["avg_p_spoil"] is not None else None,
        )
        for row in rows
    ]


@router.get("/summary", response_model=WasteSummary)
async def waste_summary(days: int = 30, conn: asyncpg.Connection = Depends(db_dependency)):
    row = await conn.fetchrow(
        """
        SELECT
            SUM(CASE WHEN reason != 'wasted' THEN 1 ELSE 0 END) as consumed,
            SUM(CASE WHEN reason  = 'wasted' THEN 1 ELSE 0 END) as wasted
        FROM consumption_history
        WHERE consumed_at::timestamp >= NOW() - ($1 || ' days')::interval
        """,
        str(days),
    )
    consumed = row["consumed"] or 0
    wasted = row["wasted"] or 0
    total = consumed + wasted
    waste_rate = (wasted / total * 100) if total > 0 else 0.0

    patterns = [
        WastePattern(
            name=r["item_name"], category=r["category"],
            times_wasted=r["times_wasted"], avg_p_spoil_at_removal=float(r["avg_p_spoil"]) if r["avg_p_spoil"] is not None else None,
        )
        for r in await conn.fetch(
            """
            SELECT item_name, category, COUNT(*) as times_wasted, AVG(p_spoil_at_removal) as avg_p_spoil
            FROM consumption_history WHERE reason = 'wasted'
            GROUP BY LOWER(item_name), category ORDER BY times_wasted DESC LIMIT 5
            """
        )
    ]

    trend = [
        ConsumptionPoint(date=str(r["date"]), items_consumed=r["items_consumed"], total_quantity=r["total_quantity"] or 0)
        for r in await conn.fetch(
            """
            SELECT DATE(consumed_at::timestamp) as date, COUNT(*) as items_consumed, SUM(quantity_consumed) as total_quantity
            FROM consumption_history
            WHERE consumed_at::timestamp >= NOW() - ($1 || ' days')::interval AND reason != 'wasted'
            GROUP BY DATE(consumed_at::timestamp) ORDER BY date ASC
            """,
            str(days),
        )
    ]

    return WasteSummary(
        total_consumed=consumed,
        total_wasted=wasted,
        waste_rate_pct=round(waste_rate, 1),
        top_wasted=patterns,
        daily_trend=trend,
    )


class ConsumptionPrediction(BaseModel):
    name: str
    category: str
    times_consumed: int
    total_quantity: int
    avg_interval_days: Optional[float]   # None if only 1 event
    weekly_rate: Optional[float]         # items per week
    last_consumed: str
    predicted_next_days: Optional[float] # days from now; negative = overdue
    confidence: str                      # LOW | MEDIUM | HIGH


@router.get("/predictions", response_model=list[ConsumptionPrediction])
async def consumption_predictions(conn: asyncpg.Connection = Depends(db_dependency)):
    """
    Per-item consumption behaviour model.
    Uses average inter-consumption interval to predict next consumption date.
    Only considers non-wasted events (consumed / cooked).
    """
    rows = await conn.fetch(
        """
        WITH intervals AS (
            SELECT
                item_name,
                category,
                quantity_consumed,
                consumed_at::timestamp AS ts,
                EXTRACT(EPOCH FROM (
                    consumed_at::timestamp
                    - LAG(consumed_at::timestamp) OVER (
                        PARTITION BY LOWER(item_name)
                        ORDER BY consumed_at::timestamp
                    )
                )) / 86400.0 AS interval_days
            FROM consumption_history
            WHERE reason != 'wasted'
        )
        SELECT
            LOWER(item_name)        AS item_key,
            MAX(item_name)          AS name,
            MAX(category)           AS category,
            COUNT(*)                AS times_consumed,
            SUM(quantity_consumed)  AS total_quantity,
            MIN(ts)                 AS first_consumed,
            MAX(ts)                 AS last_consumed,
            AVG(interval_days)      AS avg_interval_days
        FROM intervals
        GROUP BY LOWER(item_name)
        ORDER BY times_consumed DESC, last_consumed DESC
        """
    )

    now = datetime.now(tz=timezone.utc)
    predictions: list[ConsumptionPrediction] = []

    for row in rows:
        times = row["times_consumed"]
        last_ts = row["last_consumed"].replace(tzinfo=timezone.utc)
        first_ts = row["first_consumed"].replace(tzinfo=timezone.utc)
        total_qty = row["total_quantity"] or 1
        avg_interval = float(row["avg_interval_days"]) if row["avg_interval_days"] is not None else None

        # Weekly rate: total items / total weeks spanned (min 1 week denominator)
        span_weeks = max((now - first_ts).total_seconds() / (86400 * 7), 1.0)
        weekly_rate = round(total_qty / span_weeks, 2)

        # Predicted days from now until next consumption
        if avg_interval is not None:
            days_since_last = (now - last_ts).total_seconds() / 86400.0
            predicted_next_days = round(avg_interval - days_since_last, 1)
        else:
            predicted_next_days = None

        # Confidence based on number of data points
        if times >= 5:
            confidence = "HIGH"
        elif times >= 3:
            confidence = "MEDIUM"
        else:
            confidence = "LOW"

        predictions.append(ConsumptionPrediction(
            name=row["name"],
            category=row["category"],
            times_consumed=times,
            total_quantity=total_qty,
            avg_interval_days=round(avg_interval, 1) if avg_interval is not None else None,
            weekly_rate=weekly_rate,
            last_consumed=last_ts.isoformat(),
            predicted_next_days=predicted_next_days,
            confidence=confidence,
        ))

    return predictions
