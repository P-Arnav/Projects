from __future__ import annotations
from typing import Optional
from pydantic import BaseModel, Field


class UserRegister(BaseModel):
    username: str = Field(..., min_length=3, max_length=50)
    email: str
    password: str = Field(..., min_length=6)
    household_name: Optional[str] = None


class UserLogin(BaseModel):
    email: str
    password: str


class UserRead(BaseModel):
    user_id: str
    username: str
    email: str
    household_id: str
    role: str = "member"
    created_at: str

    @classmethod
    def from_supabase(cls, user, household_id: str) -> "UserRead":
        meta = user.user_metadata or {}
        return cls(
            user_id=str(user.id),
            username=meta.get("username", user.email.split("@")[0]),
            email=user.email,
            household_id=household_id,
            role=meta.get("role", "member"),
            created_at=str(user.created_at),
        )


class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: UserRead


class UserPrefs(BaseModel):
    auto_restock_enabled: bool = False


class UserPrefsUpdate(BaseModel):
    auto_restock_enabled: Optional[bool] = None
