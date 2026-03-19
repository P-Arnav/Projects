from contextlib import asynccontextmanager
import logging
import os

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from core.database import init_db
from services.settle_timer import recover_on_startup
from routers import items as items_router
from routers import alerts as alerts_router
from routers import status as status_router
from routers import lookup as lookup_router
from routers import vision as vision_router
from routers import grocery as grocery_router
from routers import restock as restock_router
from routers import recipes as recipes_router
from routers import receipt as receipt_router
from websocket.ws_router import router as ws_router

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
)


@asynccontextmanager
async def lifespan(app: FastAPI):
    await init_db()
    await recover_on_startup()
    yield
    # asyncio cancels all running tasks on shutdown automatically


app = FastAPI(
    title="FridgeAI Backend",
    description="Real-time sync engine for the FridgeAI food waste reduction system.",
    version="0.1.0",
    lifespan=lifespan,
)

_origins = os.getenv("ALLOWED_ORIGINS", "*").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(items_router.router)
app.include_router(alerts_router.router)
app.include_router(status_router.router)
app.include_router(lookup_router.router)
app.include_router(vision_router.router)
app.include_router(grocery_router.router)
app.include_router(restock_router.router)
app.include_router(recipes_router.router)
app.include_router(receipt_router.router)
app.include_router(ws_router)
