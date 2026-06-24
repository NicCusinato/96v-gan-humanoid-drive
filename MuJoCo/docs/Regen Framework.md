.

# **System Architecture Brief: Direct Regenerative Braking in Humanoid Legs**

**Objective:** To capture and route negative mechanical work from highly dynamic gait phases (e.g., heel strikes, landing) directly back to the primary 96V battery bus, utilizing structural hardware backdrivability and phase-based software impedance scaling.

### **1\. Hardware Topology**

The physical layer relies on converting the existing motor and inverter architecture into a distributed network of synchronous boost converters during negative-work phases.

* **Actuators:** Quasi-Direct Drive (QDD) motors characterized by a high motor constant ($K\_m$) and ultra-low phase inductance ($L\_s$). The low gear ratio provides the essential mechanical backdrivability required for the environment to easily spin the rotor during impact.  
* **Power Electronics:** Distributed 96V GaN inverters. The absence of a reverse-recovery charge ($Q\_{rr}$) in GaN HEMTs permits switching frequencies of 50–100 kHz. The MCU executes **Active Synchronous Rectification**, turning on the reverse channels to provide a near-zero voltage drop path for regenerative currents.  
* **Communication:** EtherCAT bus operating at $\\ge 1$ kHz, providing rigid, deterministic synchronization between the central torso computer and the distal joint MCUs.  
* **Critical Failsafe:** A localized or central hardware Braking Chopper. If the battery State of Charge (SoC) is saturated and the Battery Management System isolates the pack, the bus voltage will instantly spike. A hardware comparator must trigger at $\\approx 105$V to dump the DC link into a high-wattage planar resistor, preventing GaN dielectric breakdown.

### **2\. Control Stack Hierarchy**

The control framework strictly isolates mechanical trajectory planning from the electrical generation constraints to preserve the convexity and speed of the high-level solvers.

* **Top Layer: Model Predictive Control (MPC)**  
  * **Rate:** $\\sim 50$ Hz.  
  * **Function:** Operates purely on Simplified Rigid Body Dynamics. It calculates the optimal Ground Reaction Forces (GRFs) required to maintain balance and execute the commanded velocity. It assumes perfect, idealized actuators and is completely blind to battery states.  
* **Middle Layer: Whole-Body Control (WBC) & Impedance Mapping**  
  * **Rate:** $\\sim 1$ kHz (Synchronized via EtherCAT).  
  * **Function:** Translates MPC force vectors into Cartesian/Joint torques using the Jacobian ($\\tau\_{cmd} \= J^T F\_{grf}$).  
  * **The Regeneration Mechanism:** Executes **Phase-Based Admittance Control**. During a predicted foot-strike, the WBC dynamically drops virtual stiffness ($K\_p$) and spikes virtual damping ($K\_d$). This equation ($\\tau \= \-K\_d \\dot{\\theta}$) forces the leg to yield to the impact, organically generating a negative torque command that directly opposes the backdriven velocity.  
  * **SoC Clamp:** Monitors battery SoC. If SoC is $\> 95\\%$, the WBC mathematically clamps the maximum allowable negative $\\tau\_{cmd}$ to prevent pushing power into a saturated battery.  
* **Bottom Layer: Field-Oriented Control (FOC)**  
  * **Rate:** $50$ to $100$ kHz.  
  * **Function:** Executes on the local joint MCU. When it receives a $\\tau\_{cmd}$ that opposes the measured velocity ($\\tau \\cdot \\omega \< 0$), the FOC natively shifts into Quadrant IV operation.  
  * **Efficiency Optimization:** Strictly enforces a Maximum Torque Per Ampere (MTPA) strategy, clamping $I\_d \= 0$ during braking. This ensures every ampere of phase current contributes directly to the damping torque, minimizing $I^2 R\_s$ Joule heating in the stator coils and maximizing the energy pushed back to the 96V bus.

### **3\. The Energy Cascade**

When the humanoid leg strikes the ground, the system executes the following operational flow:

1. Environment mechanically backdrives the QDD rotor.  
2. WBC recognizes the gait phase and commands a high damping torque (negative $I\_q^\*$).  
3. GaN inverter rapidly shorts the stator to build an inductive magnetic field, then opens to boost the collapsing $L \\frac{di}{dt}$ voltage spike above the 96V DC link.  
4. Active synchronous rectification smooths the transfer.  
5. Current flows up the EtherCAT-paralleled power harness to chemically recharge the torso battery.