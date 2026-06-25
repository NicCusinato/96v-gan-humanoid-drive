"""
GaitReplay — Loads loco-mujoco .npz motion capture data and provides
interpolated joint-space reference trajectories for the full KBot model.

Usage:
    gait = GaitReplay("path/to/walk_clip.npz", model)
    qpos_ref, qvel_ref = gait.get_targets(t)  # returns (nu,) arrays matching actuator order
"""

import mujoco
import numpy as np
import os


class GaitReplay:
    """Loads a loco-mujoco .npz gait clip and provides interpolated targets
    that automatically map to whatever MuJoCo model you pass in."""

    def __init__(self, npz_path, model, loop=True):
        """
        Args:
            npz_path: Path to a loco-mujoco .npz gait data file.
            model: MjModel instance — used to discover actuator->joint mapping.
            loop: If True, the trajectory loops continuously.
        """
        if not os.path.exists(npz_path):
            raise FileNotFoundError(f"Gait data not found: {npz_path}")

        data = np.load(npz_path, allow_pickle=True)

        self.frequency = float(data["frequency"])
        self.dt_ref = 1.0 / self.frequency

        joint_names_data = list(data["joint_names"])
        qpos_full = data["qpos"].astype(np.float64)  # (T, 27)
        qvel_full = data["qvel"].astype(np.float64)  # (T, 26)

        self.n_frames = qpos_full.shape[0]
        self.duration = self.n_frames * self.dt_ref
        self.loop = loop
        self.nu = model.nu

        # --- Extract base trajectory ---
        self.base_pos = qpos_full[:, 0:3].copy()
        self.base_quat = qpos_full[:, 3:7].copy()
        self.base_vel = qvel_full[:, 0:6].copy()

        # Zero-origin X/Y so walking starts at origin
        self.base_pos[:, 0] -= self.base_pos[0, 0]
        self.base_pos[:, 1] -= self.base_pos[0, 1]

        # Shift Z so the lowest point of the trajectory touches the ground.
        # We add +0.03m (3cm) clearance because the foot collision geometry has radius/thickness,
        # otherwise the foot is driven through the floor, generating massive 5000N+ forces that 
        # crush the leg and force it to buckle sideways (causing the 138-degree hip roll error).
        min_z = np.min(data["site_xpos"][:, :, 2]) if "site_xpos" in data else 0.0
        if min_z != 0.0:
            self.base_pos[:, 2] -= (min_z - 0.03)  # Shift up an extra 3cm
        else:
            self.base_pos[:, 2] -= 0.02

        # --- Build actuator-ordered joint mapping ---
        # For each actuator in the model, find the matching column in the .npz data
        self._qpos_indices = []  # indices into qpos_full columns
        self._qvel_indices = []  # indices into qvel_full columns

        for i in range(model.nu):
            jid = model.actuator_trnid[i, 0]
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)

            # Find this joint in the .npz joint_names list
            data_idx = joint_names_data.index(jname)

            # qpos column: 7 (base pos+quat) + (data_idx - 1) since idx 0 is floating_base
            self._qpos_indices.append(7 + (data_idx - 1))
            # qvel column: 6 (base vel) + (data_idx - 1)
            self._qvel_indices.append(6 + (data_idx - 1))

        # Extract joint trajectories in actuator order: (T, nu)
        self.joint_qpos = qpos_full[:, self._qpos_indices].copy()
        self.joint_qvel = qvel_full[:, self._qvel_indices].copy()

        print(f"  [GaitReplay] Loaded {os.path.basename(npz_path)}: "
              f"{self.n_frames} frames @ {self.frequency} Hz = {self.duration:.2f}s, "
              f"mapped {model.nu} actuators")

    def _get_frame_and_alpha(self, t):
        """Returns (frame_index, alpha) for linear interpolation at time t."""
        if self.loop:
            t = t % self.duration
        t = np.clip(t, 0.0, self.duration - self.dt_ref)
        frame_f = t / self.dt_ref
        frame_i = int(frame_f)
        alpha = frame_f - frame_i
        frame_i = min(frame_i, self.n_frames - 2)
        return frame_i, alpha

    def get_targets(self, t):
        """
        Returns interpolated joint targets at simulation time t.

        Returns:
            qpos_target: (nu,) joint positions matching actuator order
            qvel_target: (nu,) joint velocities matching actuator order
        """
        i, alpha = self._get_frame_and_alpha(t)
        qpos_target = (1.0 - alpha) * self.joint_qpos[i] + alpha * self.joint_qpos[i + 1]
        qvel_target = (1.0 - alpha) * self.joint_qvel[i] + alpha * self.joint_qvel[i + 1]
        return qpos_target, qvel_target

    def get_base_target(self, t):
        """
        Returns interpolated base position and orientation at simulation time t.

        Returns:
            base_pos: (3,)
            base_quat: (4,) — w, x, y, z
            base_vel: (6,) — linear(3) + angular(3)
        """
        i, alpha = self._get_frame_and_alpha(t)
        pos = (1.0 - alpha) * self.base_pos[i] + alpha * self.base_pos[i + 1]
        vel = (1.0 - alpha) * self.base_vel[i] + alpha * self.base_vel[i + 1]
        q0 = self.base_quat[i]
        q1 = self.base_quat[i + 1]
        quat = self._slerp(q0, q1, alpha)
        return pos, quat, vel

    @staticmethod
    def _slerp(q0, q1, alpha):
        """Spherical linear interpolation between two quaternions."""
        dot = np.dot(q0, q1)
        if dot < 0:
            q1 = -q1
            dot = -dot
        dot = np.clip(dot, -1.0, 1.0)
        if dot > 0.9995:
            result = (1.0 - alpha) * q0 + alpha * q1
        else:
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0 * alpha
            sin_theta = np.sin(theta)
            s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0
            result = s0 * q0 + s1 * q1
        return result / np.linalg.norm(result)
