from __future__ import annotations
from typing import Optional
from pydantic import BaseModel, Field, model_validator


class FeedbackCreate(BaseModel):
    """
    Either provide shelf_life_actual (exact days the item lasted)
    or set still_good=True (system infers it lasted at least until now).
    """
    shelf_life_actual: Optional[float] = Field(
        default=None, gt=0, description="Actual days the item lasted"
    )
    still_good: bool = Field(
        default=False, description="Item is still good right now"
    )

    @model_validator(mode="after")
    def _at_least_one(self) -> "FeedbackCreate":
        if self.shelf_life_actual is None and not self.still_good:
            raise ValueError("Provide shelf_life_actual or set still_good=True")
        return self


class FeedbackRead(BaseModel):
    feedback_id: str
    item_id: str
    category: str
    shelf_life_declared: int
    shelf_life_actual: float
    correction: float
    created_at: str
