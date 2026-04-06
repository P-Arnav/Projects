"""
Visual encoder: ViT-Base/16 (primary) or DenseNet-121 (lightweight alternative).

Both variants return a sequence of visual tokens  V ∈ R^{N_v × d_v}
suitable for downstream cross-modal attention.
"""

import torch
import torch.nn as nn


class ViTVisualEncoder(nn.Module):
    """
    ViT-Base/16 backbone (via timm) that returns per-patch token embeddings.

    Args:
        pretrained:      Load ImageNet-pretrained weights.
        freeze_layers:   Number of transformer blocks to freeze (from block 0).
                         Set to 0 to fine-tune everything, 8 to freeze first 8.
        output_dim:      If set, add a linear projection to this dimension.
                         If None, the raw d_v=768 embeddings are returned.
    """

    def __init__(
        self,
        pretrained: bool = True,
        freeze_layers: int = 8,
        output_dim: int | None = None,
    ):
        super().__init__()
        import timm
        self.backbone = timm.create_model(
            "vit_base_patch16_224",
            pretrained=pretrained,
            num_classes=0,      # remove classification head
        )
        self.d_v = self.backbone.embed_dim  # 768

        # Freeze early transformer blocks
        if freeze_layers > 0:
            # Freeze patch embedding + positional embedding
            for p in self.backbone.patch_embed.parameters():
                p.requires_grad = False
            if hasattr(self.backbone, "cls_token"):
                self.backbone.cls_token.requires_grad = False
            if hasattr(self.backbone, "pos_embed"):
                self.backbone.pos_embed.requires_grad = False
            # Freeze the first `freeze_layers` transformer blocks
            for i, block in enumerate(self.backbone.blocks):
                if i < freeze_layers:
                    for p in block.parameters():
                        p.requires_grad = False

        # Optional projection to a shared embedding dimension
        self.proj = nn.Linear(self.d_v, output_dim) if output_dim else None

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, 3, 224, 224) float tensor (ImageNet-normalised)

        Returns:
            V: (B, N_v, d) where N_v=196 patch tokens, d=768 (or output_dim)
        """
        # forward_features returns (B, N+1, d) including the [CLS] token
        tokens = self.backbone.forward_features(x)  # (B, 197, 768)
        patch_tokens = tokens[:, 1:, :]             # drop [CLS] → (B, 196, 768)

        if self.proj is not None:
            patch_tokens = self.proj(patch_tokens)

        return patch_tokens


class DenseNetVisualEncoder(nn.Module):
    """
    DenseNet-121 backbone (via torchvision) that returns spatial region tokens.

    The last dense block's feature map is spatially flattened to produce a
    token sequence  V ∈ R^{N_v × d_v}.

    Args:
        pretrained:  Load ImageNet-pretrained weights.
        output_dim:  Optional linear projection dimension.
    """

    def __init__(self, pretrained: bool = True, output_dim: int | None = None):
        super().__init__()
        import torchvision.models as tvm
        weights = tvm.DenseNet121_Weights.IMAGENET1K_V1 if pretrained else None
        dn = tvm.densenet121(weights=weights)
        # Keep everything except the classifier
        self.features = dn.features
        self.d_v = 1024   # DenseNet-121 last feature map channels

        self.proj = nn.Linear(self.d_v, output_dim) if output_dim else None

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, 3, 224, 224)

        Returns:
            V: (B, N_v, d) where N_v = 7*7 = 49 spatial locations
        """
        import torch.nn.functional as F
        feats = self.features(x)              # (B, 1024, 7, 7)
        feats = F.relu(feats, inplace=True)
        B, C, H, W = feats.shape
        tokens = feats.permute(0, 2, 3, 1).reshape(B, H * W, C)  # (B, 49, 1024)

        if self.proj is not None:
            tokens = self.proj(tokens)

        return tokens


def build_visual_encoder(
    backbone: str = "vit",
    pretrained: bool = True,
    freeze_layers: int = 8,
    output_dim: int | None = None,
) -> nn.Module:
    """
    Factory for visual encoders.

    Args:
        backbone:     "vit" or "densenet"
        pretrained:   Use pretrained weights.
        freeze_layers: Blocks to freeze (ViT only).
        output_dim:   Project tokens to this dim (None = raw backbone dim).

    Returns:
        An nn.Module with signature forward(x) → (B, N_v, d).
    """
    if backbone == "vit":
        return ViTVisualEncoder(
            pretrained=pretrained,
            freeze_layers=freeze_layers,
            output_dim=output_dim,
        )
    elif backbone == "densenet":
        return DenseNetVisualEncoder(pretrained=pretrained, output_dim=output_dim)
    else:
        raise ValueError(f"Unknown backbone '{backbone}'. Choose 'vit' or 'densenet'.")
