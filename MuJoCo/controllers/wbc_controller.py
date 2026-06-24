import mujoco
import numpy as np
import json
import os

class MinimalWBC:
    def __init__(self, xml_path, metadata_path=None, knee_bend=0.45, target_pitch=-0.05):
        # Load model and initialize data
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.knee_bend = knee_bend
        self.target_pitch = target_pitch
        
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
            ankle_l, ankle_r = solve_flat_foot_ankles(theta, self.knee_bend)
            
            kin_data.qpos[leg_qpos_idx[0]] = theta
            kin_data.qpos[leg_qpos_idx[3]] = self.knee_bend
            kin_data.qpos[leg_qpos_idx[4]] = ankle_l
            
            kin_data.qpos[leg_qpos_idx[5]] = -theta
            kin_data.qpos[leg_qpos_idx[8]] = -self.knee_bend
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
        best_ankle_l, best_ankle_r = solve_flat_foot_ankles(best_theta, self.knee_bend)
        print(f"  [WBC] Optimal Hip Pitch: {best_theta:.4f} rad, Optimal Ankles: Left = {best_ankle_l:.4f} rad, Right = {best_ankle_r:.4f} rad")
        
        # Verify the solved CoM and support center match
        kin_data.qpos[leg_qpos_idx[0]] = best_theta
        kin_data.qpos[leg_qpos_idx[3]] = self.knee_bend
        kin_data.qpos[leg_qpos_idx[4]] = best_ankle_l
        kin_data.qpos[leg_qpos_idx[5]] = -best_theta
        kin_data.qpos[leg_qpos_idx[8]] = -self.knee_bend
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
            best_theta, 0.0, 0.0, self.knee_bend, best_ankle_l,  # Left leg
            -best_theta, 0.0, 0.0, -self.knee_bend, best_ankle_r,  # Right leg
            0.0, 0.0, 0.0, 0.0, 0.0,  # Right arm
            0.0, 0.0, 0.0, 0.0, 0.0   # Left arm
        ]
        
        # Expose foot body IDs for external diagnostics/monitoring
        self.left_foot_body_id = left_foot_body_id
        self.right_foot_body_id = right_foot_body_id

        # Automatically pre-set joint position states in qpos
        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[i, 0]
            qpos_idx = self.model.jnt_qposadr[joint_id]
            self.data.qpos[qpos_idx] = self.stand_target[i]
            
        self.data.qpos[3:7] = [np.cos(self.target_pitch/2), 0, np.sin(self.target_pitch/2), 0]
        
        # Solve for and set initial base tangent spawn height
        spawn_height = self.solve_spawn_height()
        self.data.qpos[2] = spawn_height
        mujoco.mj_forward(self.model, self.data)
        
        # Define target torso relative height for active vertical stabilization
        self.target_height = self.data.xpos[self.torso_id][2] - 0.5 * (self.data.xpos[self.left_foot_body_id][2] + self.data.xpos[self.right_foot_body_id][2])
        print(f"  [WBC] Standing height target set to: {self.target_height:.4f} m (base spawn height: {spawn_height:.4f} m)")

    def get_pitch_from_quat(self, quat):
        # Convert quaternion [w, x, y, z] to rotation matrix
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, quat)
        mat = mat.reshape(3, 3)
        
        # Z-column of orientation matrix represents the global 'Up' vector
        up_vector = mat[:, 2]
        pitch = np.arcsin(np.clip(up_vector[0], -1.0, 1.0))
        return pitch

    def solve_spawn_height(self):
        """
        Determines the correct base height (qpos[2]) so that the feet are tangent to the ground.
        """
        # Save current state base height
        original_z = self.data.qpos[2]
        
        # Set base height to 1.0 for calculation
        self.data.qpos[2] = 1.0
        mujoco.mj_forward(self.model, self.data)
        
        local_foot_points = [
            np.array([-0.06, 0.0, 0.0]),  # Heel
            np.array([0.12, 0.0, 0.0]),   # Toe
            np.array([0.0, -0.03, 0.0]),  # Left lateral edge
            np.array([0.0, 0.03, 0.0])    # Right lateral edge
        ]
        
        lowest_z = 100.0
        for body_id in [self.left_foot_body_id, self.right_foot_body_id]:
            pos = self.data.xpos[body_id]
            mat = self.data.xmat[body_id].reshape(3, 3)
            for p in local_foot_points:
                p_glob = pos + mat.dot(p)
                z_bottom = p_glob[2] - 0.02  # Capsule radius is 0.02
                if z_bottom < lowest_z:
                    lowest_z = z_bottom
                    
        # Solved base height is such that lowest_z reaches 0
        solved_height = 1.0 - lowest_z
        self.data.qpos[2] = original_z  # restore
        return solved_height

    def compute_torques(self, gait_targets=None):
        # Default to static stand if no targets provided
        if gait_targets is None:
            gait_targets = {
                'com_offset': np.array([0.0, 0.0, 0.0]),
                'torso_pitch': self.target_pitch
            }
        com_offset = gait_targets.get('com_offset', np.zeros(3))
        target_pitch = gait_targets.get('torso_pitch', self.target_pitch)

        # 1. Forward Kinematics & Dynamics (updates Jacobians, Gravity, Mass matrix)
        mujoco.mj_forward(self.model, self.data)
        
        # Get torso rotation matrix to resolve local-to-global angular coordinates
        quat = self.data.qpos[3:7]
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, quat)
        mat = mat.reshape(3, 3)
        
        # 2. Define the Task: Keep the torso pitch at target
        current_pitch = self.get_pitch_from_quat(quat)
        
        # Transform local-frame angular velocity (qvel[3:6]) to global world-frame
        omega_local = self.data.qvel[3:6]
        omega_global = mat.dot(omega_local)
        current_pitch_vel = omega_global[1]  # Y-component is global Y-axis pitch velocity!
        
        # Compliant task gains to absorb perturbations in pitch rather than fight them
        Kp, Kd = 100.0, 20.0  # Critically damped
        
        # Stiffen pitch massively during explosive thrust to prevent front-flips!
        if gait_targets.get('jump_phase', -1) == 1:
            Kp, Kd = 2000.0, 50.0
            
        acc_desired = Kp * (target_pitch - current_pitch) + Kd * (0.0 - current_pitch_vel)
        
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
        
        # Physical mass of the entire robot
        m_total = np.sum(self.model.body_mass)
        
        # Physical Whole-Body Inertia for pitch (rotation of total mass about the stance feet)
        Lambda_pitch = m_total * (self.target_height ** 2)
        
        # Projected task force scaling
        F_task = Lambda_pitch * acc_desired
        
        # Compute joint torques projected using Jacobian Transpose
        self.tau_task = J_pitch * F_task
        
        # --- 4.5. Operational Space CoM Stabilization Task ---
        com_x_curr = self.data.subtree_com[0][0]
        com_y_curr = self.data.subtree_com[0][1]
        
        # Midpoint of feet in world coordinates
        left_foot_glob_center = self.data.xpos[self.left_foot_body_id] + self.data.xmat[self.left_foot_body_id].reshape(3, 3).dot(np.array([-0.025, -0.038, 0.0]))
        right_foot_glob_center = self.data.xpos[self.right_foot_body_id] + self.data.xmat[self.right_foot_body_id].reshape(3, 3).dot(np.array([-0.025, -0.038, 0.0]))
        # Apply commanded X/Y CoM offsets from gait trajectory
        com_x_des = 0.5 * (left_foot_glob_center[0] + right_foot_glob_center[0]) + com_offset[0]
        com_y_des = 0.5 * (left_foot_glob_center[1] + right_foot_glob_center[1]) + com_offset[1]
        
        com_x_error = com_x_des - com_x_curr
        com_y_error = com_y_des - com_y_curr
        
        # Relative com translational Jacobian
        J_com_x = jacp[0, :] - 0.5 * (jacp_l[0, :] + jacp_r[0, :])
        J_com_y = jacp[1, :] - 0.5 * (jacp_l[1, :] + jacp_r[1, :])
        
        # Exact relative CoM velocity projected from state space
        com_x_vel = J_com_x.dot(self.data.qvel)
        com_y_vel = J_com_y.dot(self.data.qvel)
        
        # Compliant task gains to absorb perturbations sagittally
        Kp_com, Kd_com = 60.0, 15.5  # Critically damped
        F_com_x = Kp_com * com_x_error - Kd_com * com_x_vel
        F_com_y = Kp_com * com_y_error - Kd_com * com_y_vel
        
        # Physical Whole-Body Mass for translational CoM task
        Lambda_com = m_total
        tau_task_com = J_com_x * (Lambda_com * F_com_x) + J_com_y * (Lambda_com * F_com_y)
        
        # --- 5. Dynamic Whole-Body Torso Gravity Compensation + Active Height Control ---
        # Relative height task: maintain torso height relative to feet midpoint
        # Apply commanded Z-offset from gait trajectory (e.g., squat depth)
        z_curr = self.data.xpos[self.torso_id][2] - 0.5 * (self.data.xpos[self.left_foot_body_id][2] + self.data.xpos[self.right_foot_body_id][2])
        z_target = self.target_height + com_offset[2]
        
        # Stance-consistent relative vertical Jacobian (Z-axis is row 2)
        J_z = jacp[2, :] - 0.5 * (jacp_l[2, :] + jacp_r[2, :])
        z_vel = J_z.dot(self.data.qvel)
        
        # Compliant active vertical force to absorb vertical impact
        Kp_z, Kd_z = 150.0, 24.5  # Critically damped defaults
        
        jump_phase = gait_targets.get('jump_phase', -1)
        
        if jump_phase == 1:
            # Thrust phase: Massive stiffness to launch
            Kp_z = 3000.0  
        elif jump_phase == 2:
            # Flight phase: Zero stiffness, let gravity pull it down, relax legs
            Kp_z = 0.0
            Kd_z = 5.0
        elif jump_phase == 3:
            # Landing phase: Soft, damped spring to absorb impact and backdrive (regen)
            Kp_z = 100.0
            Kd_z = 35.0  # High damping to suck energy out of the landing
            
        f_active_z = Kp_z * (z_target - z_curr) - Kd_z * z_vel
        
        # Physical Whole-Body Mass for vertical height task
        Lambda_z = m_total
        F_task_z = Lambda_z * f_active_z
        
        m_total = np.sum(self.model.body_mass)
        f_gravity_upward = m_total * 9.81 + F_task_z
        tau_torso_gravity = J_z * f_gravity_upward
        
        # 6. Compute Joint-Space Posture PD (maintains arms and knees stance compliance)
        self.tau_pd = np.zeros(self.model.nu)
        
        current_target = list(self.stand_target)
        jump_phase = gait_targets.get('jump_phase', -1)
        phase_time = gait_targets.get('phase_time', 0.0)
        
        if jump_phase == 0:
            # Squat (0 to 0.5s): Bend knees, dorsiflex ankles to keep feet flat!
            prog = phase_time / 0.5
            squat_val = 1.0 * (1.0 - np.cos(np.pi * prog)) / 2.0
            # Increase knee bend (Left is positive, Right is negative)
            current_target[3] += squat_val
            current_target[8] -= squat_val
            # Dorsiflex ankles to keep heels planted (Assume negative is dorsiflexion)
            current_target[4] -= squat_val * 0.6
            current_target[9] -= squat_val * 0.6
            
        elif jump_phase == 1:
            # Thrust (0 to 0.2s)
            # 0.0 to 0.1s: Explode knees straight
            # 0.1 to 0.2s: Explode ankles to tip-toe!
            knee_prog = np.clip(phase_time / 0.1, 0.0, 1.0)
            ankle_prog = np.clip((phase_time - 0.1) / 0.1, 0.0, 1.0)
            
            # Knee thrusts from deep squat (squat_val=1.0) back to 0.0
            current_target[3] += 1.0 * (1.0 - knee_prog)
            current_target[8] -= 1.0 * (1.0 - knee_prog)
            
            # Ankle thrusts from dorsiflexion (-0.6) to extreme tip-toe (+0.25)
            current_target[4] += -0.6 * (1.0 - ankle_prog) + 0.25 * ankle_prog
            current_target[9] += -0.6 * (1.0 - ankle_prog) + 0.25 * ankle_prog
            
        elif jump_phase == 3:
            # Landing: deeply bend knees and dorsiflex ankles to absorb impact
            current_target[3] += 0.8
            current_target[8] -= 0.8
            current_target[4] -= 0.5
            current_target[9] -= 0.5
            
        for i in range(self.model.nu):
            joint_id = self.model.actuator_trnid[i, 0]
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            
            meta = self.metadata.get('joint_name_to_metadata', {}).get(joint_name, {'kp': 100, 'kd': 5})
            kp = float(meta['kp'])
            kd = float(meta['kd'])
            
            # Scale leg joint gains for backdrivability while maintaining damping ratio
            if 'hip_pitch' in joint_name or 'knee' in joint_name or 'ankle' in joint_name:
                kp_scale = 0.20
                if jump_phase == 3:
                    kp_scale = 0.05  # Moderate compliance for landing
                
                kd_scale = np.sqrt(kp_scale)
                kp *= kp_scale
                kd *= kd_scale
            elif 'hip_roll' in joint_name or 'hip_yaw' in joint_name:
                kp_scale = 0.50
                kd_scale = np.sqrt(kp_scale)
                kp *= kp_scale
                kd *= kd_scale
                
            qpos_idx = self.model.jnt_qposadr[joint_id]
            qvel_idx = self.model.jnt_dofadr[joint_id]
            
            self.tau_pd[i] = kp * (current_target[i] - self.data.qpos[qpos_idx]) - kd * self.data.qvel[qvel_idx]
            
        # 7. Add native Gravity and Coriolis compensation
        tau_gravity_coriolis = self.data.qfrc_bias
        
        # Sum Task Torques, Torso Gravity Compensation, native link gravity forces, and CoM stabilizer
        total_torque = self.tau_task + tau_gravity_coriolis + tau_torso_gravity + tau_task_com
        
        # Strip out floating base dimensions (first 6 DoF) to get actuated motor torques
        motor_torques = total_torque[6:] + self.tau_pd
        
        return motor_torques

    def step(self, gait_targets=None):
        # Apply torques and step physics
        self.data.ctrl[:] = self.compute_torques(gait_targets)
        mujoco.mj_step(self.model, self.data)
