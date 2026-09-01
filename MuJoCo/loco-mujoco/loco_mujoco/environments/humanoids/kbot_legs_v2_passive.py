from typing import List
import numpy as np
from loco_mujoco.environments.humanoids.kbot_legs_v2 import KBotLegsV2
from mujoco import MjSpec


class PassiveKBotLegsV2(KBotLegsV2):
    """KBotLegsV2 variant for Stage 1 Passive Policy.

    Action space is doubled to [a_q (10) | a_alpha (10)].
    PDControl gates torques: tau_j = alpha_j * PD(...)
    Adds passive reward bonus: r = weight / (sum(alpha) + eps)
    """

    mjx_enabled = True

    def __init__(self, passive_reward_weight=0.01, alpha_curriculum_start=0.5, **kwargs):
        """
        Args:
            passive_reward_weight: Weight for passive reward. Default 0.01.
                Increase to 0.05 if alpha never drops below 0.9.
            alpha_curriculum_start: Minimum alpha floor at training start,
                linearly decayed to 0 over total_timesteps. Default 0.5.
        """
        self._passive_reward_weight = passive_reward_weight
        self._alpha_curriculum_start = alpha_curriculum_start
        super().__init__(**kwargs)

        # Manually double the action space to include alphas [0, 1]
        # This avoids confusing the base MuJoCo XML actuator mappings.
        from mushroom_rl.utils.spaces import Box
        orig_low = self.info.action_space.low
        orig_high = self.info.action_space.high
        low = np.concatenate([orig_low, np.zeros_like(orig_low)])
        high = np.concatenate([orig_high, np.ones_like(orig_high)])
        self._mdp_info.action_space = Box(low, high)



    def _compute_passive_reward(self, action: np.ndarray) -> float:
        """Passive reward: inversely proportional to the sum of alpha values."""
        n = len(action) // 2
        alpha = action[n:]
        return self._passive_reward_weight / (float(np.sum(np.abs(alpha))) + 1e-6)


class MjxPassiveKBotLegsV2(PassiveKBotLegsV2):
    """MJX-compatible variant for GPU-accelerated training."""
    mjx_enabled = True
