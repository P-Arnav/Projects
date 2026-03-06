"""
settle_timer.py — per-item asyncio tasks for the 30-minute settle delay.

After an item is inserted into the fridge, we wait SETTLE_DELAY_SECONDS
before running the ASLIE/FAPF scorer. This gives the item time to reach
stable storage temperature and avoids false-positive spoilage reads.
"""

import asyncio
import logging
from datetime import datetime, timezone

from core.config import SETTLE_DELAY_SECONDS
from core.database import get_db
from services.scorer import run_for_item

logger = logging.getLogger(__name__)

# item_id → asyncio.Task
_pending: dict[str, asyncio.Task] = {}


async def _run_settle(item_id: str, delay: float) -> None:
    try:
        await asyncio.sleep(delay)
        logger.info("Settle timer fired for item %s", item_id)
        await run_for_item(item_id)
    except asyncio.CancelledError:
        logger.info("Settle timer cancelled for item %s", item_id)
    finally:
        _pending.pop(item_id, None)


def schedule(item_id: str, delay: float | None = None) -> None:
    """Start a settle timer for item_id. Cancels any existing timer first."""
    cancel(item_id)
    actual_delay = delay if delay is not None else float(SETTLE_DELAY_SECONDS)
    task = asyncio.create_task(_run_settle(item_id, actual_delay))
    _pending[item_id] = task
    logger.info("Settle timer scheduled for item %s (delay=%.0fs)", item_id, actual_delay)


def cancel(item_id: str) -> None:
    """Cancel a pending settle timer (e.g. on DELETE or score-relevant PATCH)."""
    task = _pending.pop(item_id, None)
    if task and not task.done():
        task.cancel()


def pending_count() -> int:
    return len(_pending)


async def recover_on_startup() -> None:
    """
    On server restart, reschedule timers for items inserted in the last
    SETTLE_DELAY_SECONDS seconds that have not been scored yet (P_spoil IS NULL).
    """
    now_utc = datetime.now(tz=timezone.utc)
    async with get_db() as db:
        cur = await db.execute("SELECT item_id, entry_time FROM items WHERE P_spoil IS NULL")
        rows = await cur.fetchall()

    for row in rows:
        entry_time = datetime.fromisoformat(row["entry_time"]).replace(tzinfo=timezone.utc)
        elapsed = (now_utc - entry_time).total_seconds()
        remaining_delay = max(0.0, float(SETTLE_DELAY_SECONDS) - elapsed)
        schedule(row["item_id"], delay=remaining_delay)
        logger.info(
            "Startup recovery: rescheduled item %s (%.0fs remaining)",
            row["item_id"],
            remaining_delay,
        )
