"""Small dual-lidar recurrent behavior-cloning policy."""

from __future__ import annotations

import torch
from torch import nn


class LidarGruPolicy(nn.Module):
    def __init__(self, state_size: int = 8, hidden_size: int = 128) -> None:
        super().__init__()
        self.scan_encoder = nn.Sequential(
            nn.Conv1d(2, 16, kernel_size=9, stride=4, padding=4),
            nn.ReLU(),
            nn.Conv1d(16, 32, kernel_size=7, stride=4, padding=3),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(8),
            nn.Flatten(),
            nn.Linear(32 * 8, 96),
            nn.ReLU(),
        )
        self.state_encoder = nn.Sequential(nn.Linear(state_size, 32), nn.ReLU())
        self.recurrent = nn.GRU(128, hidden_size, batch_first=True)
        self.head = nn.Sequential(nn.Linear(hidden_size, 64), nn.ReLU(), nn.Linear(64, 2))

    def forward(self, scan, state, hidden=None):
        batch, steps, channels, beams = scan.shape
        scan_features = self.scan_encoder(scan.reshape(batch * steps, channels, beams))
        scan_features = scan_features.reshape(batch, steps, -1)
        state_features = self.state_encoder(state)
        encoded = torch.cat((scan_features, state_features), dim=-1)
        output, hidden = self.recurrent(encoded, hidden)
        return self.head(output), hidden
