"""
Clinical history encoder using Bio_ClinicalBERT.

Encodes free-text patient history into:
  T    ∈ R^{N_t × d_t}   — per-token contextual embeddings
  t_cls ∈ R^{d_t}         — [CLS] global summary vector
"""

import torch
import torch.nn as nn
from transformers import AutoModel, AutoTokenizer

MODEL_NAME = "emilyalsentzer/Bio_ClinicalBERT"
NO_HISTORY_TOKEN = "NO_HISTORY"


class ClinicalHistoryEncoder(nn.Module):
    """
    Bio_ClinicalBERT-based encoder for free-text clinical histories.

    Args:
        model_name:    HuggingFace model identifier.
        max_length:    Maximum tokenisation length (tokens).
        freeze_layers: Number of BERT encoder layers to freeze from layer 0.
                       The embedding layer is always frozen when freeze_layers>0.
        output_dim:    Optional linear projection applied to all token outputs.
                       If None, the raw d_t=768 dimension is kept.
    """

    def __init__(
        self,
        model_name: str = MODEL_NAME,
        max_length: int = 128,
        freeze_layers: int = 8,
        output_dim: int | None = None,
    ):
        super().__init__()
        self.max_length = max_length
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.bert = AutoModel.from_pretrained(model_name)
        self.d_t = self.bert.config.hidden_size  # 768

        # ── Selective freezing ─────────────────────────────────────────────
        if freeze_layers > 0:
            # Always freeze embeddings
            for p in self.bert.embeddings.parameters():
                p.requires_grad = False
            # Freeze first `freeze_layers` encoder layers
            for i, layer in enumerate(self.bert.encoder.layer):
                if i < freeze_layers:
                    for p in layer.parameters():
                        p.requires_grad = False

        # ── Optional projection ────────────────────────────────────────────
        self.proj = nn.Linear(self.d_t, output_dim) if output_dim else None

    # ── Tokenisation helper (called on CPU, outside the model forward) ─────

    def tokenize(
        self,
        texts: list[str],
        device: torch.device | str = "cpu",
    ) -> dict[str, torch.Tensor]:
        """
        Tokenise a list of clinical history strings.

        "NO_HISTORY" inputs are replaced with a single padding sequence so the
        model still receives a valid tensor (it will learn to produce a neutral
        embedding for these samples).

        Returns a dict with keys: input_ids, attention_mask, token_type_ids.
        """
        # Replace NO_HISTORY sentinel with an empty string
        clean_texts = [
            "" if t.strip() == NO_HISTORY_TOKEN else t for t in texts
        ]
        encoding = self.tokenizer(
            clean_texts,
            padding="max_length",
            truncation=True,
            max_length=self.max_length,
            return_tensors="pt",
        )
        return {k: v.to(device) for k, v in encoding.items()}

    # ── Forward ────────────────────────────────────────────────────────────

    def forward(
        self,
        input_ids: torch.Tensor,
        attention_mask: torch.Tensor,
        token_type_ids: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            input_ids:      (B, N_t) long tensor
            attention_mask: (B, N_t) long tensor
            token_type_ids: (B, N_t) long tensor (optional)

        Returns:
            T:     (B, N_t, d) token-level embeddings
            t_cls: (B, d)       [CLS] token embedding (global summary)
        """
        kwargs = dict(
            input_ids=input_ids,
            attention_mask=attention_mask,
        )
        if token_type_ids is not None:
            kwargs["token_type_ids"] = token_type_ids

        outputs = self.bert(**kwargs)
        # last_hidden_state: (B, N_t, d_t)
        T = outputs.last_hidden_state
        t_cls = T[:, 0, :]   # [CLS] is always the first token

        if self.proj is not None:
            T = self.proj(T)
            t_cls = self.proj(t_cls)

        return T, t_cls
