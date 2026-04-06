"""
Full end-to-end MultimodalDiagnosticModel.

Architecture (per Algorithm 1 in the paper):
  1. Visual tokens  V  from ViT-Base/16 (or DenseNet-121)
  2. Text tokens    T, t_cls  from Bio_ClinicalBERT
  3. Bidirectional cross-modal attention → V_fused, T_fused
  4. Consistency module → compatibility score s
  5. Image-only head  → logits_img
  6. Fused head       → logits_fused
  7. Gated combination: logits = logits_img + s · logits_fused
  8. Sigmoid → disease probabilities p ∈ [0,1]^14
"""

import torch
import torch.nn as nn

from models.visual_encoder import build_visual_encoder
from models.text_encoder import ClinicalHistoryEncoder
from models.cross_attention import CrossModalFusion
from models.consistency import ConsistencyModule, gated_logits

NUM_CLASSES = 14


class MultimodalDiagnosticModel(nn.Module):
    """
    History-conditioned multimodal chest X-ray diagnosis model.

    Args:
        visual_backbone: "vit" or "densenet"
        shared_dim:      Common embedding dimension for cross-modal attention.
        num_heads:       Number of attention heads in cross-modal fusion.
        freeze_vis_layers: Encoder blocks to freeze in the visual backbone.
        freeze_text_layers: BERT layers to freeze in the text encoder.
        max_text_length: Max token length for ClinicalBERT.
        mlp_hidden_dim:  Hidden size of the consistency MLP.
        dropout:         Dropout rate throughout.
        bidirectional:   Whether to use symmetric V→T cross-attention.
    """

    def __init__(
        self,
        visual_backbone: str = "vit",
        shared_dim: int = 512,
        num_heads: int = 8,
        freeze_vis_layers: int = 8,
        freeze_text_layers: int = 8,
        max_text_length: int = 128,
        mlp_hidden_dim: int = 256,
        dropout: float = 0.1,
        bidirectional: bool = True,
    ):
        super().__init__()

        # ── 1. Visual encoder ──────────────────────────────────────────────
        self.visual_encoder = build_visual_encoder(
            backbone=visual_backbone,
            pretrained=True,
            freeze_layers=freeze_vis_layers,
            output_dim=None,   # keep raw backbone dim; project inside fusion
        )
        vis_dim = self.visual_encoder.d_v   # 768 for ViT, 1024 for DenseNet

        # ── 2. Text encoder ────────────────────────────────────────────────
        self.text_encoder = ClinicalHistoryEncoder(
            max_length=max_text_length,
            freeze_layers=freeze_text_layers,
            output_dim=None,   # keep d_t=768
        )
        txt_dim = self.text_encoder.d_t   # 768

        # ── 3. Cross-modal fusion ──────────────────────────────────────────
        self.fusion = CrossModalFusion(
            visual_dim=vis_dim,
            text_dim=txt_dim,
            shared_dim=shared_dim,
            num_heads=num_heads,
            dropout=dropout,
            bidirectional=bidirectional,
        )

        # ── 4. Consistency module ──────────────────────────────────────────
        self.consistency = ConsistencyModule(
            visual_dim=shared_dim,
            text_dim=shared_dim,
            cls_dim=txt_dim,       # ClinicalBERT [CLS] dim
            hidden_dim=mlp_hidden_dim,
            dropout=dropout,
        )

        # ── 5. Classification heads ────────────────────────────────────────
        # Image-only head operates on projected visual tokens (shared_dim)
        self.img_proj = nn.Linear(vis_dim, shared_dim)
        self.head_img = nn.Linear(shared_dim, NUM_CLASSES)

        # Fused head operates on pooled gated fused representation
        self.head_fused = nn.Linear(shared_dim, NUM_CLASSES)

        self.dropout = nn.Dropout(dropout)

    # ── Convenience: tokenise text (delegates to text encoder) ─────────────

    def tokenize(self, texts: list[str], device) -> dict:
        return self.text_encoder.tokenize(texts, device)

    # ── Forward ─────────────────────────────────────────────────────────────

    def forward(
        self,
        images: torch.Tensor,
        input_ids: torch.Tensor,
        attention_mask: torch.Tensor,
        token_type_ids: torch.Tensor | None = None,
    ) -> dict[str, torch.Tensor]:
        """
        Args:
            images:         (B, 3, 224, 224)  lung-ROI chest X-rays
            input_ids:      (B, N_t)           tokenised clinical history
            attention_mask: (B, N_t)
            token_type_ids: (B, N_t)  (optional)

        Returns:
            A dict with keys:
              "logits"       (B, 14)  final gated logits
              "probs"        (B, 14)  sigmoid probabilities
              "logits_img"   (B, 14)  image-only logits
              "score"        (B, 1)   compatibility score s
              "attn_tv"      (B, N_t, N_v)  T→V attention weights (explainability)
        """
        # ── Visual encoding ────────────────────────────────────────────────
        V = self.visual_encoder(images)          # (B, N_v, vis_dim)

        # ── Text encoding ──────────────────────────────────────────────────
        T, t_cls = self.text_encoder(
            input_ids=input_ids,
            attention_mask=attention_mask,
            token_type_ids=token_type_ids,
        )                                        # T: (B, N_t, 768), t_cls: (B, 768)

        # ── Cross-modal fusion ─────────────────────────────────────────────
        V_fused, T_fused, attn_tv = self.fusion(V, T)
        # V_fused: (B, N_v, shared_dim), T_fused: (B, N_t, shared_dim)

        # ── Compatibility score ────────────────────────────────────────────
        s = self.consistency(V_fused, T_fused, t_cls)   # (B, 1)

        # ── Image-only branch ──────────────────────────────────────────────
        v_pool = self.img_proj(V.mean(dim=1))    # (B, shared_dim)
        v_pool = self.dropout(v_pool)
        logits_img = self.head_img(v_pool)       # (B, 14)

        # ── Fused branch ───────────────────────────────────────────────────
        # Gate: element-wise scale fused tokens by compatibility score
        h_fused = (V_fused * s.unsqueeze(1)).mean(dim=1)   # (B, shared_dim)
        h_fused = self.dropout(h_fused)
        logits_fused = self.head_fused(h_fused)             # (B, 14)

        # ── Gated combination (eq. 15) ─────────────────────────────────────
        logits = gated_logits(logits_img, logits_fused, s)  # (B, 14)

        return {
            "logits":     logits,
            "probs":      torch.sigmoid(logits),
            "logits_img": logits_img,
            "score":      s,
            "attn_tv":    attn_tv,
        }
