from contextlib import asynccontextmanager
import logging
import os

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from core.database import init_db, close_db
from core.supabase_client import init_supabase
from services.settle_timer import recover_on_startup
from services import auto_restock
from routers import items as items_router
from routers import alerts as alerts_router
from routers import status as status_router
from routers import lookup as lookup_router
from routers import vision as vision_router
from routers import grocery as grocery_router
from routers import restock as restock_router
from routers import recipes as recipes_router
from routers import receipt as receipt_router
from routers import analytics as analytics_router
from routers import auth as auth_router
from websocket.ws_router import router as ws_router

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
)


@asynccontextmanager
async def lifespan(app: FastAPI):
    await init_db()
    await init_supabase()
    await recover_on_startup()
    auto_restock.start()
    yield
    auto_restock.stop()
    await close_db()


app = FastAPI(
    title="FridgeAI Backend",
    description="Real-time sync engine for the FridgeAI food waste reduction system.",
    version="0.2.0",
    lifespan=lifespan,
)

@app.exception_handler(RequestValidationError)
async def validation_error_handler(request: Request, exc: RequestValidationError):
    messages = []
    for err in exc.errors():
        field = err["loc"][-1] if err["loc"] else "field"
        field_label = str(field).replace("_", " ").title()
        messages.append(f"{field_label}: {err['msg']}")
    return JSONResponse(status_code=422, content={"detail": "; ".join(messages)})


_origins = os.getenv("ALLOWED_ORIGINS", "*").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(auth_router.router)
app.include_router(items_router.router)
app.include_router(alerts_router.router)
app.include_router(status_router.router)
app.include_router(lookup_router.router)
app.include_router(vision_router.router)
app.include_router(grocery_router.router)
app.include_router(restock_router.router)
app.include_router(recipes_router.router)
app.include_router(receipt_router.router)
app.include_router(analytics_router.router)
app.include_router(ws_router)
