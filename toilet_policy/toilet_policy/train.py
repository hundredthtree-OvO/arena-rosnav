"""Train the small dual-lidar GRU behavior-cloning baseline."""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import Sequence

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

from .model import LidarGruPolicy


class SequenceDataset(Dataset):
    def __init__(self, root: Path, manifest: dict, split: str, sequence_length: int) -> None:
        self.arrays = []
        self.windows = []
        for item in manifest["episodes"]:
            if item["split"] != split:
                continue
            data = np.load(root / item["shard"])
            array_index = len(self.arrays)
            self.arrays.append(
                {
                    "scan": data["scan"].astype(np.float32),
                    "state": data["state"].astype(np.float32),
                    "action": data["action"].astype(np.float32),
                }
            )
            for end in range(sequence_length, len(data["action"]) + 1):
                self.windows.append((array_index, end - sequence_length, end))

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, index: int):
        array_index, start, end = self.windows[index]
        item = self.arrays[array_index]
        return item["scan"][start:end], item["state"][start:end], item["action"][start:end]


def _evaluate(model, loader, device, loss_fn) -> float:
    model.eval()
    total = 0.0
    count = 0
    with torch.no_grad():
        for scan, state, action in loader:
            prediction, _ = model(scan.to(device), state.to(device))
            loss = loss_fn(prediction, action.to(device))
            total += float(loss) * scan.shape[0]
            count += scan.shape[0]
    return total / max(1, count)


def _action_metrics(model, loader, device) -> dict[str, float]:
    model.eval()
    absolute_error = torch.zeros(2, dtype=torch.float64)
    squared_error = torch.zeros(2, dtype=torch.float64)
    sample_count = 0
    with torch.no_grad():
        for scan, state, action in loader:
            prediction, _ = model(scan.to(device), state.to(device))
            error = (prediction[:, -1] - action.to(device)[:, -1]).cpu().double()
            absolute_error += error.abs().sum(dim=0)
            squared_error += error.square().sum(dim=0)
            sample_count += error.shape[0]
    denominator = max(1, sample_count)
    return {
        "sample_count": sample_count,
        "linear_mae": float(absolute_error[0] / denominator),
        "angular_mae": float(absolute_error[1] / denominator),
        "linear_rmse": float(torch.sqrt(squared_error[0] / denominator)),
        "angular_rmse": float(torch.sqrt(squared_error[1] / denominator)),
    }


def _checkpoint_payload(
    model,
    optimizer,
    manifest: dict,
    *,
    epoch: int,
    sequence_length: int,
    train_loss: float,
    validation_loss: float,
) -> dict:
    return {
        "epoch": int(epoch),
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "model": {"state_size": 8, "hidden_size": 128},
        "sequence_length": int(sequence_length),
        "sample_rate_hz": manifest["sample_rate_hz"],
        "beam_count": manifest["beam_count"],
        "train_loss": float(train_loss),
        "validation_loss": float(validation_loss),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="toilet_policy_train", description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--sequence-length", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--seed", type=int, default=42)
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _parser().parse_args(args)
    random.seed(namespace.seed)
    np.random.seed(namespace.seed)
    torch.manual_seed(namespace.seed)
    manifest = json.loads((namespace.dataset / "manifest.json").read_text())
    train_data = SequenceDataset(
        namespace.dataset, manifest, "train", namespace.sequence_length
    )
    validation_data = SequenceDataset(
        namespace.dataset, manifest, "validation", namespace.sequence_length
    )
    test_data = SequenceDataset(
        namespace.dataset, manifest, "test", namespace.sequence_length
    )
    if not train_data or not validation_data or not test_data:
        raise ValueError("training, validation, and test splits must contain sequence windows")
    train_loader = DataLoader(train_data, batch_size=namespace.batch_size, shuffle=True)
    validation_loader = DataLoader(validation_data, batch_size=namespace.batch_size)
    test_loader = DataLoader(test_data, batch_size=namespace.batch_size)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = LidarGruPolicy().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=namespace.learning_rate)
    loss_fn = nn.SmoothL1Loss()
    namespace.output.mkdir(parents=True, exist_ok=True)
    best_loss = float("inf")
    history = []
    for epoch in range(1, namespace.epochs + 1):
        model.train()
        total = 0.0
        count = 0
        for scan, state, action in train_loader:
            optimizer.zero_grad(set_to_none=True)
            prediction, _ = model(scan.to(device), state.to(device))
            loss = loss_fn(prediction, action.to(device))
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            total += float(loss.detach()) * scan.shape[0]
            count += scan.shape[0]
        train_loss = total / max(1, count)
        validation_loss = _evaluate(model, validation_loader, device, loss_fn)
        history.append(
            {"epoch": epoch, "train_loss": train_loss, "validation_loss": validation_loss}
        )
        print(
            f"epoch={epoch:03d} train_loss={train_loss:.6f} "
            f"validation_loss={validation_loss:.6f} device={device}"
        )
        checkpoint = _checkpoint_payload(
            model,
            optimizer,
            manifest,
            epoch=epoch,
            sequence_length=namespace.sequence_length,
            train_loss=train_loss,
            validation_loss=validation_loss,
        )
        torch.save(checkpoint, namespace.output / "last.pt")
        if validation_loss < best_loss:
            best_loss = validation_loss
            torch.save(checkpoint, namespace.output / "best.pt")
    if history:
        (namespace.output / "history.json").write_text(
            json.dumps(history, indent=2) + "\n", encoding="utf-8"
        )
    checkpoint = torch.load(
        namespace.output / "best.pt", map_location=device, weights_only=True
    )
    best_loss = float(checkpoint["validation_loss"])
    model.load_state_dict(checkpoint["model_state_dict"])
    test_metrics = _action_metrics(model, test_loader, device)
    (namespace.output / "test_metrics.json").write_text(
        json.dumps(test_metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "best_validation_loss": best_loss,
                "output": str(namespace.output),
                "test_metrics": test_metrics,
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
