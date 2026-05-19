import numpy as np
from enum import Enum

class GaitMode(Enum):
    STAND = 0           # Rock-solid upright balance
    WEIGHT_SHIFT = 1    # Slow quasi-static multi-axis weight shifts (proves backdrivability)
    STEP_IN_PLACE = 2   # Small-amplitude compliant stepping in-place (proves impact absorption)

class GaitGenerator:
    """
    Trajectory Generator for the Kbot Whole-Body Controller (WBC).
    Provides smooth Operational Space targets (CoM offsets, Torso pitch, and Foot trajectories)
    optimized for highly backdrivable QDD actuators and high-bandwidth GaN motor drives.
    """
    def __init__(self, mode=GaitMode.STAND):
        self.mode = mode
        
        # --- Weight Shift Parameters ---
        self.shift_freq_x = 0.5   # 0.5 Hz sagittal shift
        self.shift_freq_y = 0.4   # 0.4 Hz lateral shift
        self.shift_amp_x = 0.04    # 4 cm sagittal amplitude
        self.shift_amp_y = 0.03    # 3 cm lateral amplitude
        
        # --- Stepping Parameters ---
        self.step_period = 1.0     # 1.0 second per step (agile QDD transition rate)
        self.double_support_frac = 0.2  # 20% double-support phase fraction
        self.step_height = 0.04    # 4 cm swing foot clearance (low clearance reduces impact)
        self.sway_amp_y = 0.025    # 2.5 cm lateral sway amplitude to unload swing foot
        
        # --- Nominal Reference Targets ---
        self.nominal_pitch = -0.05  # -2.8 degrees proud standing posture pitch
        self.nominal_height = 0.749  # Nominal spawn base height
        
    def set_mode(self, mode: GaitMode):
        """Changes the active gait mode."""
        self.mode = mode
        
    def get_targets(self, t: float):
        """
        Computes Operational Space targets at simulation time t.
        Returns:
            targets (dict): containing:
                - com_offset (np.ndarray): [x, y, z] target CoM displacement relative to feet midpoint.
                - torso_pitch (float): Target absolute torso pitch in radians.
                - left_foot_pos (np.ndarray): [x, y, z] target relative to nominal stance.
                - right_foot_pos (np.ndarray): [x, y, z] target relative to nominal stance.
                - contact_left (bool): Whether left foot is expected in stance contact.
                - contact_right (bool): Whether right foot is expected in stance contact.
        """
        if self.mode == GaitMode.STAND:
            return self._compute_stand_targets(t)
        elif self.mode == GaitMode.WEIGHT_SHIFT:
            return self._compute_weight_shift_targets(t)
        elif self.mode == GaitMode.STEP_IN_PLACE:
            return self._compute_stepping_targets(t)
        else:
            raise ValueError(f"[ERROR] Unsupported GaitMode: {self.mode}")
            
    def _compute_stand_targets(self, t: float):
        """Static stance balance targets."""
        return {
            "com_offset": np.array([0.0, 0.0, 0.0]),
            "torso_pitch": self.nominal_pitch,
            "left_foot_pos": np.array([0.0, 0.0, 0.0]),
            "right_foot_pos": np.array([0.0, 0.0, 0.0]),
            "contact_left": True,
            "contact_right": True
        }
        
    def _compute_weight_shift_targets(self, t: float):
        """Slow, multi-axis weight shifts to show off QDD smoothness and low-cogging characteristics."""
        dx = self.shift_amp_x * np.sin(2 * np.pi * self.shift_freq_x * t)
        dy = self.shift_amp_y * np.cos(2 * np.pi * self.shift_freq_y * t)
        
        return {
            "com_offset": np.array([dx, dy, 0.0]),
            "torso_pitch": self.nominal_pitch,
            "left_foot_pos": np.array([0.0, 0.0, 0.0]),
            "right_foot_pos": np.array([0.0, 0.0, 0.0]),
            "contact_left": True,
            "contact_right": True
        }
        
    def _compute_stepping_targets(self, t: float):
        """
        Generates in-place stepping trajectories with compliant swing foot arcs
        and lateral CoM sway synchronization.
        """
        # Determine current phase within step cycle
        cycle_time = t % self.step_period
        half_period = self.step_period / 2.0
        
        # Stance/Swing phase durations
        t_ds = half_period * self.double_support_frac
        t_ss = half_period * (1.0 - self.double_support_frac)
        
        # Initialize default stance foot targets
        l_foot = np.array([0.0, 0.0, 0.0])
        r_foot = np.array([0.0, 0.0, 0.0])
        contact_l = True
        contact_r = True
        
        # Lateral sway of CoM to naturally shift weight onto stance leg
        # sway_amp_y shifts negative when lifting right foot, positive when lifting left foot
        dy = self.sway_amp_y * np.sin(2 * np.pi * (t / self.step_period))
        dx = 0.0
        
        # Cycle through Left Stance/Right Swing and Right Stance/Left Swing
        if cycle_time < half_period:
            # First half-period: Right foot swings
            phase_t = cycle_time
            if phase_t > t_ds:
                # Single Support Phase: Right foot lifts
                t_swing = phase_t - t_ds
                # Cycloidal swing height trajectory: z = step_height * sin(pi * t_swing / t_ss)
                r_foot[2] = self.step_height * np.sin(np.pi * t_swing / t_ss)
                contact_r = False
        else:
            # Second half-period: Left foot swings
            phase_t = cycle_time - half_period
            if phase_t > t_ds:
                # Single Support Phase: Left foot lifts
                t_swing = phase_t - t_ds
                l_foot[2] = self.step_height * np.sin(np.pi * t_swing / t_ss)
                contact_l = False
                
        return {
            "com_offset": np.array([dx, dy, 0.0]),
            "torso_pitch": self.nominal_pitch,
            "left_foot_pos": l_foot,
            "right_foot_pos": r_foot,
            "contact_left": contact_l,
            "contact_right": contact_r
        }
