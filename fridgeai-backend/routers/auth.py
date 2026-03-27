from __future__ import annotations
import os
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from core.supabase_client import get_supabase
from models.user import UserRegister, UserLogin, UserRead, TokenResponse, UserPrefs, UserPrefsUpdate

router = APIRouter(prefix="/auth", tags=["auth"])
_bearer = HTTPBearer(auto_error=False)
REQUIRE_AUTH: bool = os.getenv("REQUIRE_AUTH", "false").lower() == "true"


async def _user_from_token(token: str) -> UserRead:
    sb = get_supabase()
    try:
        resp = await sb.auth.get_user(jwt=token)
    except Exception as exc:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail=f"Invalid or expired token: {exc}")
    if resp.user is None:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token")
    user_id = str(resp.user.id)
    prefs = await sb.table("user_prefs").select("household_id").eq("user_id", user_id).execute()
    household_id = prefs.data[0]["household_id"] if prefs.data else ""
    return UserRead.from_supabase(resp.user, household_id)


async def get_current_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(_bearer),
) -> Optional[UserRead]:
    """Soft auth dependency — enforces only when REQUIRE_AUTH=true."""
    if not REQUIRE_AUTH:
        return None
    if credentials is None:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Not authenticated")
    return await _user_from_token(credentials.credentials)


@router.get("/config")
async def auth_config():
    return {"require_auth": REQUIRE_AUTH}


@router.post("/register", response_model=TokenResponse, status_code=status.HTTP_201_CREATED)
async def register(body: UserRegister):
    sb = get_supabase()
    household_name = body.household_name or f"{body.username}'s household"

    try:
        resp = await sb.auth.sign_up({
            "email": body.email,
            "password": body.password,
            "options": {"data": {"username": body.username}},
        })
    except Exception as exc:
        msg = getattr(exc, 'message', None) or getattr(exc, 'msg', None) or str(exc)
        raise HTTPException(status_code=400, detail=msg)

    if resp.user is None:
        raise HTTPException(
            status_code=400,
            detail="Registration failed — if email confirmation is enabled in Supabase, disable it under Auth > Settings for local dev.",
        )

    user_id = str(resp.user.id)

    try:
        h_result = await sb.table("households").insert({"name": household_name}).execute()
        household_id = h_result.data[0]["household_id"] if h_result.data else None
        if household_id is None:
            raise ValueError(f"Household insert returned no data: {h_result}")
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Household creation failed: {exc}")

    try:
        await sb.table("user_prefs").insert({
            "user_id": user_id,
            "household_id": household_id,
            "auto_restock_enabled": False,
        }).execute()
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"User prefs creation failed: {exc}")

    token = resp.session.access_token if resp.session else ""
    user = UserRead.from_supabase(resp.user, household_id)
    return TokenResponse(access_token=token, user=user)


@router.post("/login", response_model=TokenResponse)
async def login(body: UserLogin):
    sb = get_supabase()
    try:
        resp = await sb.auth.sign_in_with_password({"email": body.email, "password": body.password})
    except Exception:
        raise HTTPException(status_code=401, detail="Invalid email or password")

    if resp.user is None or resp.session is None:
        raise HTTPException(status_code=401, detail="Invalid email or password")

    user_id = str(resp.user.id)
    prefs = await sb.table("user_prefs").select("household_id").eq("user_id", user_id).execute()
    household_id = prefs.data[0]["household_id"] if prefs.data else ""

    user = UserRead.from_supabase(resp.user, household_id)
    return TokenResponse(access_token=resp.session.access_token, user=user)


@router.post("/logout", status_code=status.HTTP_204_NO_CONTENT)
async def logout(credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())):
    sb = get_supabase()
    try:
        await sb.auth.sign_out()
    except Exception:
        pass


@router.get("/me", response_model=UserRead)
async def get_me(credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())):
    return await _user_from_token(credentials.credentials)


@router.get("/prefs", response_model=UserPrefs)
async def get_prefs(credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())):
    sb = get_supabase()
    try:
        resp = await sb.auth.get_user(jwt=credentials.credentials)
    except Exception:
        raise HTTPException(status_code=401, detail="Invalid token")
    user_id = str(resp.user.id)
    result = await sb.table("user_prefs").select("auto_restock_enabled").eq("user_id", user_id).execute()
    enabled = result.data[0]["auto_restock_enabled"] if result.data else False
    return UserPrefs(auto_restock_enabled=bool(enabled))


@router.patch("/prefs", response_model=UserPrefs)
async def update_prefs(
    body: UserPrefsUpdate,
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer()),
):
    sb = get_supabase()
    try:
        resp = await sb.auth.get_user(jwt=credentials.credentials)
    except Exception:
        raise HTTPException(status_code=401, detail="Invalid token")
    user_id = str(resp.user.id)
    updates = body.model_dump(exclude_none=True)
    if updates:
        await sb.table("user_prefs").update(updates).eq("user_id", user_id).execute()
    result = await sb.table("user_prefs").select("auto_restock_enabled").eq("user_id", user_id).execute()
    enabled = result.data[0]["auto_restock_enabled"] if result.data else False
    return UserPrefs(auto_restock_enabled=bool(enabled))
