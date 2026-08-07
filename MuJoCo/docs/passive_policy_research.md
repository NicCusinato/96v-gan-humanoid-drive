# Passive Policy & Energy Efficient Locomotion Research

## Overview
To develop a robust and energy-efficient passive policy for our KBot humanoid, we have reviewed the recent literature in reinforcement learning and model-based control for legged locomotion. The core challenge in learning energy-efficient gaits is balancing the trade-off between task performance (e.g., matching a reference motion, tracking velocity) and energy consumption. 

In our previous attempts, simply adding a large energy penalty to the reward function (e.g., `energy_coeff: 0.05`) led to policy collapse—the penalty dominated the reward, causing the robot to curl up or freeze rather than walk efficiently. The literature provides several mathematical and architectural frameworks to solve this.

## Paper Breakdown

### 1. Duke Humanoid (2409.19795v2)
- **Concept:** Explicitly modulates joint torques to switch between active and passive modes.
- **Policy Modification:** The actor network outputs both joint position targets $a_q$ and a per-joint activation parameter $a_\alpha \in [0,1]$.
- **Control Law:** $\tau = \alpha \cdot (k_p(q^* - q) + k_d(\dot{q}^* - \dot{q}))$
- **Reward / Training:** They apply a passive action reward $r_{a_\alpha} = 1 / \|a_\alpha\|$, scaled by a small factor. To prevent the policy from getting stuck at small $\alpha$, they use a curriculum decay for a minimum activation threshold $\alpha_0$ from 0.5 to 0.
- **Benefits:** Direct mechanical! interpretation; allows the robot to "relax" specific joints (like the knee during the swing phase), exploiting natural pendulum dynamics. 
- **Shortcomings:** Requires careful tuning of the curriculum decay and the passive reward scale to avoid collapsing into a fully passive (falling) state prematurely.

### 2. ECO: Energy-Constrained Optimization !(ECO_Energy-Constrained_Optimization_With_Reinforcement_Learning_for_Humanoid_Walking)
- **Concept:** Treats energy consumption as a strict inequality constraint using Constrained RL (PPO-Lagrangian) rather than as a soft penalty in the reward function.
- **Methodology:** The energy cost is defined as the absolute mechanical power $C_1 = \sum |\tau_j \cdot \dot{q}_j|$. The policy is optimized to maximize task reward subject to $J_{C1}(\pi) \leq b_1$. 
- **Benefits:** Solves the exact problem we faced! By separating energy into a constraint, the policy does not collapse. The Lagrangian multiplier automatically adjusts the penalty weight dynamically during training based on whether the energy constraint $b_1$ is violated.
- **Shortcomings:** PPO-Lagrangian requires maintaining and tuning dual variables (Lagrange multipliers) and defining reasonable constraint thresholds ($b_1$) which might take trial and error.

### 3. Model-based RL for Reduced-order Models (Chen2024)
- **Concept:** Learns a reduced-order model (ROM) within a Model Predictive Controller (MPC). 
- **Methodology:** The RL agent learns the optimal CoM dynamics for a ROM, initialized as a Linear Inverted Pendulum (LIP).
- **Benefits:** Highly interpretable and theoretically stable; the MPC ensures stability while RL optimizes the reduced dynamics.
- **Shortcomings:** Computationally intensive to run MPC in the loop with RL; heavily relies on an accurate dynamic model, which might suffer in Sim-to-Real if not meticulously randomized.

### 4. Energy-Efficient Motion Planner (TR2025-151)
- **Concept:** Uses a geometry-based footstep planner where elliptical placement sets beneath the hip dictate when the robot takes a step.
- **Methodology:** The robot only steps when the foot is forced outside the ellipse, otherwise it remains passive in stance.
- **Benefits:** Naturally enforces a passive, low-frequency stepping gait which inherently reduces Cost of Transport (CoT).
- **Shortcomings:** Primarily a model-based heuristic; integrating this smoothly into an end-to-end RL framework requires significant observation engineering.

## Proposed Mathematical Framework for our Passive Policy

Based on the research, we propose a **Hybrid Constrained-Activation Policy** that combines the explicit action space of the Duke Humanoid with the stable training regime of the ECO paper.

### 1. Action Space Expansion
We will modify the LocoMuJoCo PPO actor to output:
$a = [a_q, a_\alpha]$
Where $a_q \in \mathbb{R}^{DoF}$ are the joint targets, and $a_\alpha \in [0, 1]^{DoF}$ are the activation multipliers.
The applied torque will be:
$\tau = \alpha \cdot \text{PD}(q^*, q, \dot{q}^*, \dot{q})$

### 2. PPO-Lagrangian Objective
Instead of statically weighting the energy reward, we will implement an adaptive penalty (Lagrangian multiplier $\lambda$). 
Objective: $\max_\theta \mathbb{E}[R] - \lambda (\mathbb{E}[C_{energy}] - \epsilon_{target})$
- $R$ is the standard AMASS imitation reward.
- $C_{energy} = \sum |\tau \cdot \dot{q}|$
- $\epsilon_{target}$ is our target energy threshold.
- $\lambda$ is updated via dual gradient descent: $\lambda_{t+1} = \max(0, \lambda_t + \eta (\mathbb{E}[C] - \epsilon))$

### Next Steps (Implementation Roadmap)

Our implementation is divided into four stages, building toward a fully hardware-aware passive policy:

#### Stage 1: Action Space Expansion (Duke Humanoid)
Modify the actor to output both joint targets $a_q$ and activation multipliers $a_\alpha \in [0, 1]^{DoF}$. The applied torque is gated by the activation parameter, and a soft passive reward $1/\|a_\alpha\|$ encourages the robot to relax its joints during swing phases.

#### Stage 2: PPO-Lagrangian Energy Constraint (ECO)
Remove soft energy penalties. Implement a strict inequality constraint on total mechanical power $\mathbb{E}[C_{energy}] \leq \epsilon_{target}$, enforced via a Lagrange multiplier $\lambda$ updated via dual gradient descent after every PPO update. This prevents the "bang-bang" mode collapse seen in standard RL.

#### Stage 3: Combined Hybrid Policy
Train the policy with both explicit activation multipliers ($a_\alpha$) and the strict PPO-Lagrangian energy constraint to achieve the smoothest motion with the absolute lowest Cost of Transport.

#### Stage 4: Regen Framework Integration (Hardware-Aware RL)
Exploit the physical backdrivability of KBot's Quasi-Direct Drive (QDD) actuators and the 96V GaN inverters. The actual conversion of negative work into battery charge happens **below the RL level** natively on the local FOC MCU (which shifts into Quadrant IV whenever torque opposes velocity, $\tau \cdot \omega < 0$).
However, **the RL policy sits at the top level** and must be trained to actively maximize this phenomenon! To prevent the policy from "twitching" to farm negative work, we will explicitly tie the Regeneration Bonus to specific phases of the gait (e.g., heel strikes and swing-leg deceleration):
* **Via Action Space ($a_\alpha$):** We condition the Regen Bonus on the policy intentionally dropping its activation parameter ($a_\alpha \approx 0$) during a predicted foot-strike, teaching the robot to explicitly "yield" to impacts to harvest energy.
* **Via PPO-Lagrangian:** We introduce a secondary *lower-bound* constraint (e.g., $\mathbb{E}[C_{regen}] \geq \epsilon_{regen}$) to mathematically force the policy to find opportunities in the gait cycle to harvest a minimum amount of negative work per step.
