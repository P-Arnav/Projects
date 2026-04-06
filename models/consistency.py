"""
Consistency Learning Module.

Estimates a scalar image–history compatibility score s ∈ [0,1] from pooled
fused representations and the ClinicalBERT [CLS] token, then gates the
multimodal logits using that score:

    logits_final = logits_img + s · logits_fused        (eq. 15 in paper)

The consistency loss penalises prediction disagreement weighted by (1-s):

    L_cons = (1 - s) · mean(|σ(z) - σ(z_img)|)         (eq. 19 in paper)
"""

import torch
import torch.nn as nn


class ConsistencyModule(nn.Module):
    """
    MLP-based compatibility estimator + gated prediction combiner.

    Args:
        visual_dim:   Dimension of pooled visual fused representation.
        text_dim:     Dimension of pooled text fused representation.
        cls_dim:      Dimension of the ClinicalBERT [CLS] token (t_cls).
        hidden_dim:   Hidden size of the MLP.
        dropout:      Dropout rate inside the MLP.
    """

    def __init__(
        self,
        visual_dim: int = 512,
        text_dim: int = 512,
        cls_dim: int = 768,
        hidden_dim: int = 256,
        dropout: float = 0.1,
    ):
        super().__init__()
        in_dim = visual_dim + text_dim + cls_dim

        self.mlp = nn.Sequential(
            nn.Linear(in_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim // 2, 1),
            nn.Sigmoid(),
        )

    def forward(
        self,
        V_fused: torch.Tensor,
        T_fused: torch.Tensor,
        t_cls: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            V_fused: (B, N_v, visual_dim)  fused visual tokens
            T_fused: (B, N_t, text_dim)    fused text tokens
            t_cls:   (B, cls_dim)           ClinicalBERT [CLS] embedding

        Returns:
            s: (B, 1) compatibility score in [0, 1]
        """
        v_g = V_fused.mean(dim=1)   # (B, visual_dim)
        t_g = T_fused.mean(dim=1)   # (B, text_dim)
        z_c = torch.cat([v_g, t_g, t_cls], dim=-1)  # (B, in_dim)
        s = self.mlp(z_c)           # (B, 1)
        return s


def gated_logits(
    logits_img: torch.Tensor,
    logits_fused: torch.Tensor,
    s: torch.Tensor,
) -> torch.Tensor:
    """
    Combine image-only and fused logits with the compatibility gate.

        logits_final = logits_img + s · logits_fused

    Args:
        logits_img:   (B, num_classes) image-only branch logits
        logits_fused: (B, num_classes) fused branch logits
        s:            (B, 1)           compatibility score

    Returns:
        (B, num_classes) final gated logits
    """
    return logits_img + s * logits_fused


def consistency_loss(
    z_final: torch.Tensor,
    z_img: torch.Tensor,
    s: torch.Tensor,
) -> torch.Tensor:
    """
    Eq. 19: penalise prediction disagreement between image-only and full model,
    weighted by (1 - s).

        L_cons = (1 - s) · mean(|σ(z_final) - σ(z_img)|)

    Args:
        z_final: (B, C) final (gated) logits
        z_img:   (B, C) image-only logits
        s:       (B, 1) compatibility score

    Returns:
        Scalar loss tensor.
    """
    diff = torch.abs(torch.sigmoid(z_final) - torch.sigmoid(z_img))  # (B, C)
    per_sample = diff.mean(dim=-1, keepdim=True)                      # (B, 1)
    loss = ((1.0 - s) * per_sample).mean()
    return loss
