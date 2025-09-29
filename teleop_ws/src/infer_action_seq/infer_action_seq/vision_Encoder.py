import torch
import torch.nn as nn
import torchvision.models as models
from torchvision.models import ResNet18_Weights

class ResNetEncoder(nn.Module):
    def __init__(self, output_dim=256, pretrained=True):
        super().__init__()
        # 使用 torchvision 自带的 ResNet18
        backbone = models.resnet18(weights=ResNet18_Weights.DEFAULT)
        # 去掉最后的分类层(fc)，只保留卷积特征
        self.feature_extractor = nn.Sequential(*list(backbone.children())[:-1])
        # 投影到 diffusion policy 所需的维度
        self.proj = nn.Linear(backbone.fc.in_features, output_dim)

    def forward(self, x):
        """
        x: [B, 3, H, W] 输入图像 (RGB)
        return: [B, output_dim] 特征向量
        """
        feat = self.feature_extractor(x)   # [B, 512, 1, 1]
        feat = feat.flatten(1)             # [B, 512]
        feat = self.proj(feat)             # [B, output_dim]
        return feat

def main():
    B = 4
    dummy_img = torch.randn(B, 3, 224, 224)
    dummy_state = torch.randn(B, 10)

    encoder = ResNetEncoder(output_dim=256)
    img_feat = encoder(dummy_img)               # [B, 256]

    # 拼接图像特征与状态向量
    fused_feat = torch.cat([img_feat, dummy_state], dim=-1)  # [B, 266]

    print(f"视觉特征 shape: {img_feat.shape}")
    print(f"融合特征 shape: {fused_feat.shape}")
    pass

