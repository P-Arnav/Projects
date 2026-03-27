from __future__ import annotations
from enum import Enum
from typing import Optional
from pydantic import BaseModel


class AlertType(str, Enum):
    CRITICAL_ALERT  = "CRITICAL_ALERT"
    WARNING_ALERT   = "WARNING_ALERT"
    USE_TODAY_ALERT = "USE_TODAY_ALERT"


class AlertRead(BaseModel):
    alert_id:   str
    item_id:    str
    item_name:  str
    alert_type: AlertType
    P_spoil:    Optional[float]
    RSL:        Optional[float]
    message:    str
    created_at: str

    @classmethod
    def from_row(cls, row) -> "AlertRead":
        d = dict(row)
        d["P_spoil"] = d.pop("p_spoil", d.get("P_spoil"))
        d["RSL"] = d.pop("rsl", d.get("RSL"))
        return cls(**d)
