import mujoco
import numpy as np
import json
import os

class MinimalWBC:
    def __init__(self, xml_path, metadata_path=None):
        # Load model and initialize data
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # Load joint metadata for posture control gains
        if metadata_path is None:
            metadata_path = os.path.join(os.path.dirname(xml_path), "kbot", "metadata.json")
            if not os.path.exists(metadata_path):
                metadata_path = os.path.join(os.path.dirname(xml_path), "metadata.json")
                
        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                self.metadata = json.load(f)
        else:
            self.metadata = {"joint_name_to_metadata": {}}
            
        # Identify the ID of the body we want to balance (e.g., torso, base)
        self.torso_id = -1
        for name in ["torso", "base", "base_link"]:
            try:
                self.torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                if self.torso_id >= 0:
                    print(f"  [WBC] Linked balancing body: '{name}' (ID: {self.torso_id})")
                    break
            except Exception:
                pass
                
        if self.torso_id == -1:
            raise ValueError("[ERROR] Could not find torso or base body in the robot model!")

        self.target_pitch = 0.0  # Set target pitch to 0.0 rad (upright) by default for rock-solid stability

        # --- Dynamic Kinematic Standing Balanced Pose Solver ---
        left_foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "LFootBushing_GPF_1517_12")
        right_foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "RFootBushing_GPF_1517_12")
        
        # Leg joint names and index mappings
        leg_joint_names = [
            "dof_left_hip_pitch_04", "dof_left_hip_roll_03", "dof_left_hip_yaw_03", "dof_left_knee_04", "dof_left_ankle_02",
            "dof_right_hip_pitch_04", "dof_right_hip_roll_03", "dof_right_hip_yaw_03", "dof_right_knee_04", "dof_right_ankle_02"
        ]
        leg_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in leg_joint_names]
        leg_qpos_idx = [self.model.jnt_qposadr[jid] for jid in leg_joint_ids]

        # Use an isolated data object to prevent simulation state corruption during solver sweep
        kin_data = mujoco.MjData(self.model)
        knee_bend = 0.4  # Crouch bend angle

        def solve_flat_foot_ankles(hip_pitch, knee_bend_val):
            ankle_l = 0.0
            ankle_r = 0.0
            
            # Retrieve physical joint limits for the ankles
            jnt_range_l = self.model.jnt_range[leg_joint_ids[4]]
            jnt_range_r = self.model.jnt_range[leg_joint_ids[9]]
            
            # Left Leg
            for _ in range(10):
                kin_data.qpos[leg_qpos_idx[0]] = hip_pitch
                kin_data.qpos[leg_qpos_idx[3]] = knee_bend_val
                kin_data.qpos[leg_qpos_idx[4]] = ankle_l
                
                # Make sure base is oriented at target pitch
                kin_data.qpos[3:7] = [np.cos(self.target_pitch/2), 0, np.sin(self.target_pitch/2), 0]
                
                mujoco.mj_forward(self.model, kin_data)
                
                mat_l = kin_data.xmat[left_foot_body_id].reshape(3, 3)
                pitch_l = np.arcsin(mat_l[2, 0])
                ankle_l = np.clip(ankle_l - pitch_l, jnt_range_l[0], jnt_range_l[1])
                
            # Right Leg
            for _ in range(10):
                kin_data.qpos[leg_qpos_idx[5]] = -hip_pitch
                kin_data.qpos[leg_qpos_idx[8]] = -knee_bend_val
                kin_data.qpos[leg_qpos_idx[9]] = ankle_r
                
                # Make sure base is oriented at target pitch
                kin_data.qpos[3:7] = [np.cos(self.target_pitch/2), 0, np.sin(self.target_pitch/2), 0]
                
                mujoco.mj_forward(self.model, kin_data)
                
                mat_r = kin_data.xmat[right_foot_body_id].reshape(3, 3)
                pitch_r = np.arcsin(mat_r[2, 0])
                # Mirrored ankle axis requires inverting correction feedback direction to converge
                ankle_r = np.clip(ankle_r + pitch_r, jnt_range_r[0], jnt_range_r[1])
                
            return ankle_l, ankle_r

        # Midpoint of the physical foot capsule in local coordinates
        local_foot_center = np.array([-0.025, -0.038, 0.0])

        print("  [WBC] Solving for mathematically balanced initial standing pose...")
        best_theta = 0.0
        min_com_err = 100.0
        
        # Sweep hip pitch to align Center of Mass (CoM) perfectly over the foot centers
        for theta in np.linspace(-0.6, 0.6, 300):
            ankle_l, ankle_r = solve_flat_foot_ankles(theta, knee_bend)
            
            kin_data.qpos[leg_qpos_idx[0]] = theta
            kin_data.qpos[leg_qpos_idx[3]] = knee_bend
            kin_data.qpos[leg_qpos_idx[4]] = ankle_l
            
            kin_data.qpos[leg_qpos_idx[5]] = -theta
            kin_data.qpos[leg_qpos_idx[8]] = -knee_bend
            kin_data.qpos[leg_qpos_idx[9]] = ankle_r
            
            # Make sure base is oriented at target pitch during CoM calculation
            kin_data.qpos[3:7] = [np.cos(self.target_pitch/2), 0, np.sin(self.target_pitch/2), 0]
            
            mujoco.mj_forward(self.model, kin_data)
            
            com_x = kin_data.subtree_com[0][0]
            
            left_foot_glob = kin_data.xpos[left_foot_body_id] + kin_data.xmat[left_foot_body_id].reshape(3, 3).dot(local_foot_center)
            right_foot_glob = kin_data.xpos[right_foot_body_id] + kin_data.xmat[right_foot_body_id].reshape(3, 3).dot(local_foot_center)
            support_x = 0.5 * (left_foot_glob[0] + right_foot_glob[0])
            
            com_err = abs(com_x - support_x)
            if com_err < min_com_err:
                min_com_err = com_err
                best_theta = theta

        # Resolve final optimal flat-foot ankles for the best hip pitch
        best_ankle_l, best_ankle_r = solve_flat_foot_ankles(best_theta, knee_bend)
        print(f"  [WBC] Optimal Hip Pitch: {best_theta:.4f} rad, Optimal Ankles: Left = {best_ankle_l:.4f} rad, Right = {best_ankle_r:.4f} rad")
        
        # Verify the solved CoM and support center match
        kin_data.qpos[leg_qpos_idx[0]] = best_theta
        kin_data.qpos[leg_qpos_idx[3]] = knee_bend
        kin_data.qpos[leg_qpos_idx[4]] = best_ankle_l
        kin_data.qpos[leg_qpos_idx[5]] = -best_theta
        kin_data.qpos[leg_qpos_idx[8]] = -knee_bend
        kin_data.qpos[leg_qpos_idx[9]] = best_ankle_r
        # Make sure base is oriented at target pitch
        kin_data.qpos[3:7] = [np.cos(self.target_pitch/2), 0, np.sin(self.target_pitch/2), 0]
        mujoco.mj_forward(self.model, kin_data)
        final_com_x = kin_data.subtree_com[0][0]
        final_left_glob = kin_data.xpos[left_foot_body_id] + kin_data.xmat[left_foot_body_id].reshape(3, 3).dot(local_foot_center)
        final_right_glob = kin_data.xpos[right_foot_body_id] + kin_data.xmat[right_foot_body_id].reshape(3, 3).dot(local_foot_center)
        final_support_x = 0.5 * (final_left_glob[0] + final_right_glob[0])
        print(f"  [WBC] Solver Final COM x: {final_com_x:.6f}, Support x: {final_support_x:.6f}, Error: {abs(final_com_x - final_support_x):.6f}")

        # Compile dynamically calibrated crouch standing pose (20 DoFs)
        self.stand_target = [
            best_theta, 0.0, 0.0, knee_bend, best_ankle_l,  # Left leg
            -best_theta, 0.0, 0.0, -knee_bend, best_ankle_r,  # Right leg
            0.0, 0.0, 0.0, 0.0, 0.0,  # Right arm
            0.0, 0.0, 0.0, 0.0, 0.0   # Left arm
        ]
        
        # Expose foot body IDs for external diagnostics/monitoring
        self.left_foot_body_id = left_foot_body_id
        self.right_foot_body_id = right_foot_body_id

    def get_pitch_from_quat(self, quat):
        # Convert quaternion [w, x, y, z] to rotation matrix
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, quat)
        mat = mat.reshape(3, 3)
        
        # Z-column of orientation matrix represents the global 'Up' vector
        up_vector = mat[:, 2]
        pitch = np.arcsin(np.clip(up_vector[0], -1.0, 1.0))
        return pitch

    def compute_torques(self):
        # 1. Forward Kinematics & Dynamics (updates Jacobians, Gravity, Mass matrix)
        mujoco.mj_forward(self.model, self.data)
        
        # Get torso rotation matrix to resolve local-to-global angular coordinates
        quat = self.data.qpos[3:7]
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, quat)
        mat = mat.reshape(3, 3)
        
        # 2. Define the Task: Keep the torso pitch at 0 radians
        current_pitch = self.get_pitch_from_quat(quat)
        
        # Transform local-frame angular velocity (qvel[3:6]) to global world-frame
        omega_local = self.data.qvel[3:6]
        omega_global = mat.dot(omega_local)
        current_pitch_vel = omega_global[1]  # Y-component is global Y-axis pitch velocity!
        
        # Boosted gains for rock-solid upright balance response
        Kp, Kd = 250.0, 30.0
        acc_desired = Kp * (self.target_pitch - current_pitch) + Kd * (0.0 - current_pitch_vel)
        
        # 3. Get the Jacobians for the Torso and Feet
        jacp = np.zeros((3, self.model.nv)) # Translational Jacobian
        jacr = np.zeros((3, self.model.nv)) # Rotational Jacobian
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.torso_id)
        
        jacp_l = np.zeros((3, self.model.nv))
        jacr_l = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, jacp_l, jacr_l, self.left_foot_body_id)
        
        jacp_r = np.zeros((3, self.model.nv))
        jacr_r = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, jacp_r, jacr_r, self.right_foot_body_id)
        
        # Relative global pitch Jacobian (Torso pitch minus stance feet pitch)
        J_pitch = jacr[1, :] - 0.5 * (jacr_l[1, :] + jacr_r[1, :]) 
        
        # --- 4. Dynamic Operational Space Inertia Matrix Lambda Calculation ---
        # Obtain the joint-space inertia mass matrix inverse M_inv
        M_inv = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_solveM(self.model, self.data, M_inv, np.eye(self.model.nv))
        
        # Physical Y-axis rotational inertia of the torso (for stance Y-axis pitch)
        Lambda = self.model.body_inertia[self.torso_id][1]
        
        # Projected task force scaling
        F_task = Lambda * acc_desired
        
        # Compute joint torques projected using Jacobian Transpose
        self.tau_task = J_pitch * F_task
        
        # --- 4.5. Operational Space CoM Stabilization Task ---
        com_x_curr = self.data.subtree_com[0][0]
        
        # Midpoint of feet in world coordinates
        left_foot_glob_center = self.data.xpos[self.left_foot_body_id] + self.data.xmat[self.left_foot_body_id].reshape(3, 3).dot(np.array([-0.025, -0.038, 0.0]))
        right_foot_glob_center = self.data.xpos[self.right_foot_body_id] + self.data.xmat[self.right_foot_body_id].reshape(3, 3).dot(np.array([-0.025, -0.038, 0.0]))
        com_x_des = 0.5 * (left_foot_glob_center[0] + right_foot_glob_center[0])
        
        com_x_error = com_x_des - com_x_curr
        
        # Relative com translational Jacobian (using torso X translation relative to feet as proxy)
        J_com = jacp[0, :] - 0.5 * (jacp_l[0, :] + jacp_r[0, :])
        
        # Exact relative CoM velocity projected from state space
        com_x_vel = J_com.dot(self.data.qvel)
        
        Kp_com, Kd_com = 150.0, 15.0
        F_com = Kp_com * com_x_error - Kd_com * com_x_vel
        
        # Physical mass of the entire robot (moving translationally shifts the whole robot mass)
        Lambda_com = np.sum(self.model.body_mass)
        F_task_com = Lambda_com * F_com
        
        tau_task_com = J_com * F_task_com
        
        # --- 5. Dynamic Whole-Body Torso Gravity Compensation ---
        # Sum the total physical mass of the robot torso and base
        m_total = np.sum(self.model.body_mass)
        f_gravity_upward = m_total * 9.81
        
        # Stance-consistent relative vertical Jacobian (Z-axis is row 2)
        J_z = jacp[2, :] - 0.5 * (jacp_l[2, :] + jacp_r[2, :])
        tau_torso_gravity = J_z * f_gravity_upward
        
        # 6. Compute Joint-Space Posture PD (maintains arms and knees stance compliance)
        self.tau_pd = np.zeros(20)
        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[i, 0]
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            
            meta = self.metadata.get('joint_name_to_metadata', {}).get(joint_name, {'kp': 100, 'kd': 5})
            kp = float(meta['kp'])
            kd = float(meta['kd'])
            
            # Boost leg posture joints slightly for stability
            if 'ankle' in joint_name or 'hip' in joint_name or 'knee' in joint_name:
                kp *= 1.5
                kd *= 1.5
                
            qpos_idx = self.model.jnt_qposadr[joint_id]
            qvel_idx = self.model.jnt_dofadr[joint_id]
            
            self.tau_pd[i] = kp * (self.stand_target[i] - self.data.qpos[qpos_idx]) - kd * self.data.qvel[qvel_idx]
            
        # 7. Add native Gravity and Coriolis compensation
        tau_gravity_coriolis = self.data.qfrc_bias
        
        # Sum Task Torques, Torso Gravity Compensation, native link gravity forces, and CoM stabilizer
        total_torque = self.tau_task + tau_gravity_coriolis + tau_torso_gravity + tau_task_com
        
        # Strip out floating base dimensions (first 6 DoF) to get actuated motor torques
        motor_torques = total_torque[6:] + self.tau_pd
        
        return motor_torques

    def step(self):
        # Apply torques and step physics
        self.data.ctrl[:] = self.compute_torques()
        mujoco.mj_step(self.model, self.data)
