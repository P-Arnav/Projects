from __future__ import annotations
from typing import Literal, Optional
from datetime import datetime
from pydantic import BaseModel, Field

Category = Literal[
    "dairy", "protein", "meat", "vegetable",
    "fruit", "fish", "cooked", "beverage"
]

ConfidenceTier = Literal["HIGH", "MEDIUM", "LOW", "CONFIRMED"]


class ItemCreate(BaseModel):
    name: str
    category: Category
    quantity: int = Field(default=1, ge=1)
    shelf_life: int = Field(..., ge=1, description="Expected shelf life in days")
    location: str = ""
    estimated_cost: float = Field(default=0.0, ge=0.0)
    storage_temp: float = Field(default=4.0, description="Storage temperature °C")
    humidity: float = Field(default=50.0, ge=0.0, le=100.0, description="Relative humidity %")


class ItemUpdate(BaseModel):
    quantity: Optional[int] = Field(default=None, ge=1)
    location: Optional[str] = None
    shelf_life: Optional[int] = Field(default=None, ge=1)
    estimated_cost: Optional[float] = Field(default=None, ge=0.0)
    storage_temp: Optional[float] = None
    humidity: Optional[float] = Field(default=None, ge=0.0, le=100.0)


class ItemRead(BaseModel):
    item_id: str
    name: str
    category: str
    quantity: int
    entry_time: str
    shelf_life: int
    location: str
    estimated_cost: float
    storage_temp: float
    humidity: float
    P_spoil: Optional[float]
    RSL: Optional[float]
    fapf_score: Optional[float]
    paif_action: Optional[str]
    confidence_tier: str
    updated_at: str

    @classmethod
    def from_row(cls, row) -> "ItemRead":
        d = dict(row)
        d["P_spoil"] = d.pop("p_spoil", d.get("P_spoil"))
        d["RSL"] = d.pop("rsl", d.get("RSL"))
        return cls(**d)
