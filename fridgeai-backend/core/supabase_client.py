from __future__ import annotations
import os
import logging

from supabase import acreate_client, AsyncClient

logger = logging.getLogger(__name__)

SUPABASE_URL: str = os.getenv("SUPABASE_URL", "")
SUPABASE_SERVICE_KEY: str = os.getenv("SUPABASE_SERVICE_KEY", "")

_client: AsyncClient | None = None


async def init_supabase() -> AsyncClient:
    global _client
    if not SUPABASE_URL or not SUPABASE_SERVICE_KEY:
        raise RuntimeError(
            "SUPABASE_URL and SUPABASE_SERVICE_KEY must be set in .env — "
            "get them from your Supabase project: Settings > API"
        )
    _client = await acreate_client(SUPABASE_URL, SUPABASE_SERVICE_KEY)
    logger.info("Supabase client initialised (%s)", SUPABASE_URL)
    return _client


def get_supabase() -> AsyncClient:
    if _client is None:
        raise RuntimeError("Supabase client not initialised — call init_supabase() in lifespan")
    return _client
