"""
Token-level bidirectional cross-modal attention fusion module.

Implements:
  T_fused = CrossAttn(Q=T̃, K=Ṽ, V=Ṽ)   [history attends over image patches]
  V_fused = CrossAttn(Q=Ṽ, K=T̃, V=T̃)   [image patches attend over history]

Both modalities are first projected to a shared dimension d before fusion.
Residual connections and LayerNorm are applied after each cross-attention block.
"""

import torch
import torch.nn as nn


class CrossModalFusion(nn.Module):
    """
    Bidirectional token-level cross-modal attention between image and text.

    Args:
        visual_dim:   Dimensionality of incoming visual tokens (d_v).
        text_dim:     Dimensionality of incoming text tokens (d_t).
        shared_dim:   Common projection dimension d (must be divisible by num_heads).
        num_heads:    Number of attention heads.
        dropout:      Dropout applied inside MultiheadAttention.
        bidirectional: If True (default) also apply the symmetric V→T block.
    """

    def __init__(
        self,
        visual_dim: int = 768,
        text_dim: int = 768,
        shared_dim: int = 512,
        num_heads: int = 8,
        dropout: float = 0.1,
        bidirectional: bool = True,
    ):
        super().__init__()
        assert shared_dim % num_heads == 0, (
            f"shared_dim ({shared_dim}) must be divisible by num_heads ({num_heads})"
        )
        self.shared_dim = shared_dim
        self.bidirectional = bidirectional

        # ── Input projections ──────────────────────────────────────────────
        self.proj_v = nn.Linear(visual_dim, shared_dim)
        self.proj_t = nn.Linear(text_dim, shared_dim)

        # ── T → V cross-attention (history attends over image) ─────────────
        self.t_over_v_attn = nn.MultiheadAttention(
            embed_dim=shared_dim,
            num_heads=num_heads,
            dropout=dropout,
            batch_first=True,
        )
        self.t_norm = nn.LayerNorm(shared_dim)

        # ── V → T cross-attention (image attends over history) ─────────────
        if bidirectional:
            self.v_over_t_attn = nn.MultiheadAttention(
                embed_dim=shared_dim,
                num_heads=num_heads,
                dropout=dropout,
                batch_first=True,
            )
            self.v_norm = nn.LayerNorm(shared_dim)

    def forward(
        self,
        V: torch.Tensor,
        T: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Args:
            V: (B, N_v, visual_dim)  visual token sequence
            T: (B, N_t, text_dim)    text token sequence

        Returns:
            V_fused: (B, N_v, shared_dim)
            T_fused: (B, N_t, shared_dim)
            attn_weights_tv: (B, N_t, N_v)  attention weights from T→V direction
                             (used for visual explainability maps)
        """
        # Project both modalities to shared space
        V_proj = self.proj_v(V)   # (B, N_v, d)
        T_proj = self.proj_t(T)   # (B, N_t, d)

        # ── T → V: history tokens as queries, visual tokens as keys/values ─
        T_fused, attn_tv = self.t_over_v_attn(
            query=T_proj,
            key=V_proj,
            value=V_proj,
            need_weights=True,
            average_attn_weights=False,   # keep per-head weights
        )
        # Residual + norm
        T_fused = self.t_norm(T_proj + T_fused)   # (B, N_t, d)

        # Average over heads for downstream use: (B, N_t, N_v)
        attn_weights_tv = attn_tv.mean(dim=1)

        # ── V → T: image tokens as queries, history tokens as keys/values ──
        if self.bidirectional:
            V_fused, _ = self.v_over_t_attn(
                query=V_proj,
                key=T_proj,
                value=T_proj,
            )
            V_fused = self.v_norm(V_proj + V_fused)  # (B, N_v, d)
        else:
            V_fused = V_proj   # unchanged if unidirectional

        return V_fused, T_fused, attn_weights_tv
