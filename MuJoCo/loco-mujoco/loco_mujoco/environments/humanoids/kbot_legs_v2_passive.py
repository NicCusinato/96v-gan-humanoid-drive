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

    def __init__(self, passive_reward_weight=0.01, alpha_curriculum_start=0.5, 
                 alpha_min_values=None, total_timesteps=None, **kwargs):
        """
        Args:
            passive_reward_weight: Weight for passive reward. Default 0.01.
                Increase to 0.05 if alpha never drops below 0.9.
            alpha_curriculum_start: Minimum alpha floor at training start,
                linearly decayed to 0 over total_timesteps. Default 0.5.
            alpha_min_values: Array of minimum alpha values per joint [0,1].
                If None, uses default joint-specific minimums:
                [0.6, 0.6, 0.6, 0.4, 0.8, 0.6, 0.6, 0.6, 0.4, 0.8]
                (knees can go lower for energy harvesting, ankles need higher values for stability due to low PD gain, hips medium)
            total_timesteps: Total number of training timesteps for curriculum decay.
                If None, no curriculum is applied (fixed alpha_min_values).
        """
        self._passive_reward_weight = passive_reward_weight
        self._alpha_curriculum_start = alpha_curriculum_start
        self._alpha_min_values = alpha_min_values
        self._total_timesteps = total_timesteps
        self._steps_taken = 0
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

    def _preprocess_action(self, action: np.ndarray) -> np.ndarray:
        """Apply alpha curriculum to ensure joint-specific minimum activation values."""
        # Increment step counter for curriculum
        self._steps_taken += 1
        
        # Apply joint-specific alpha curriculum if total_timesteps is specified
        if self._total_timesteps is not None and self._alpha_min_values is not None:
            # Compute curriculum progress (0.0 to 1.0)
            progress = min(1.0, self._steps_taken / self._total_timesteps)
            
            # Compute current minimum alpha values for each joint
            # alpha_min_values * (1 - progress) so they decay from initial values to 0
            current_alpha_min = np.array(self._alpha_min_values) * (1.0 - progress)
            
            # Split action into joint targets (a_q) and alpha values (a_alpha)
            n = len(action) // 2
            a_q = action[:n]
            a_alpha_raw = action[n:]  # These are the raw alpha values in [0, 1]
            
            # Apply curriculum: ensure alpha values don't go below current minimum
            # a_alpha_constrained = a_alpha_raw * (1 - current_alpha_min) + current_alpha_min
            # This maps [0,1] to [current_alpha_min, 1]
            a_alpha_constrained = a_alpha_raw * (1.0 - current_alpha_min) + current_alpha_min
            
            # Return constrained action
            return np.concatenate([a_q, a_alpha_constrained])
        else:
            # No curriculum, return action unchanged
            return action


class MjxPassiveKBotLegsV2(PassiveKBotLegsV2):
    """MJX-compatible variant for GPU-accelerated training."""
    mjx_enabled = True
