"""AI PC-side PPO inference adapter for the car workflow."""

from pathlib import Path

import numpy as np


class PPOPolicy:
    """Load the trained PPO actor and infer one of the seven car actions.

    The adapter is inference-only. It does not own the network transport and
    therefore cannot send a command to the car.
    """

    def __init__(self, model_path, dqn_root, feature_count=21):
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError(f"PyTorch is required for PPO inference: {exc}") from exc

        import sys
        dqn_root = Path(dqn_root).resolve()
        if str(dqn_root) not in sys.path:
            sys.path.insert(0, str(dqn_root))
        from models.ppo_network import PPOPositionResidualNetwork, PPONetwork

        self.torch = torch
        self.model_path = str(Path(model_path).resolve())
        self.feature_count = feature_count
        checkpoint = torch.load(self.model_path, map_location="cpu", weights_only=False)
        if checkpoint.get('architecture') == 'position_residual_v1':
            self.model = PPOPositionResidualNetwork(state_shape=(8, 21), num_actions=7)
        else:
            self.model = PPONetwork(state_shape=(8, feature_count), num_actions=7)
        state = checkpoint.get("network_state_dict") or checkpoint.get("model_state_dict")
        if state is None:
            raise RuntimeError("PPO checkpoint has no network_state_dict/model_state_dict")
        self.model.load_state_dict(state)
        self.model.eval()
        self.train_steps = checkpoint.get("train_steps")

    def predict(self, history):
        values = np.asarray(history, dtype=np.float32)
        expected = (8, self.feature_count)
        if values.shape != expected:
            raise ValueError(f"PPO expects an {expected[0]}x{expected[1]} state, got {values.shape}")
        state = self.torch.from_numpy(values).unsqueeze(0)
        with self.torch.no_grad():
            logits, value = self.model(state)
            probabilities = self.torch.softmax(logits, dim=-1)[0]
            action_id = int(probabilities.argmax().item())
        return {
            "action_id": action_id,
            "probabilities": probabilities.tolist(),
            "value": float(value[0, 0].item()),
            "model_path": self.model_path,
            "train_steps": self.train_steps,
        }
