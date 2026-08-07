# ⚡ Phase 2: Active Dyno Integration Guide — CubeMars AKE90 & Alxion 300STK Bench

This document outlines the step-by-step procedure to integrate the **CubeMars AKE90 actuator** (knee/hip joint) into the existing **Alxion 300STK2M dynamometer testbench** for operating point mapping and 4-quadrant regenerative braking characterization.

---

## 🎯 Overview & Four-Stage Process

To adapt the frameless Quasi-Direct Drive (QDD) AKE90 joint to the Alxion 300STK2M bench line, follow this four-stage setup workflow:

1. **Stage 1:** Mechanical Integration & Coaxial Alignment
2. **Stage 2:** Torque & Speed Instrumentation
3. **Stage 3:** Dual-Loop Electrical Wiring & Power Protection
4. **Stage 4:** Control System Commissioning & Test Execution

---

## 🛠️ Stage 1: Mechanical Integration & Alignment

Because the AKE90 is a frameless, high-torque Quasi-Direct Drive (QDD) joint rather than a foot-mounted cylindrical motor, it must be rigidly adapted to the existing bench center-line.

1. **Remove the Nord Assembly:**
   * Unbolt the Nord gearmotor assembly from the left side of the dyno baseplate.
   * Disconnect it from the black flexible coupling.
   * Leave the Alxion alternator clamped firmly in its custom silver housing.

2. **Machine a Custom Flange-to-Shaft Adapter:**
   * The Alxion uses a keyed cylindrical input shaft via the flexible coupling.
   * The AKE90 features a flat rotating output flange with a bolt circle pattern.
   * Machine a cylindrical steel or aluminum adapter hub:
     * **Side A:** Matches the AKE90 bolt circle pattern.
     * **Side B:** Extends as a solid keyed shaft stub that fits into the existing flexible jaw coupling.

3. **Fabricate a Rigid Reaction Mount:**
   * Fabricate a heavy-duty L-bracket or face-mount fixture plate to bolt down the non-rotating outer chassis of the AKE90.

4. **Coaxial Alignment:**
   * Mount the AKE90 assembly onto the baseplate.
   * Use precision shim stock or adjust the height of the mounting block so that the center rotational axis of the AKE90 is perfectly collinear with the Alxion shaft.
   * Tighten all mounting hardware securely to prevent structural deflection under peak torque loading (up to 170 N·m).

---

## 📊 Stage 2: Torque & Speed Measurement (Recommended)

To achieve master's-level data precision, relying solely on motor phase current to estimate torque introduces errors due to thermal resistance changes and magnetic saturation.

* **Option A — Inline Torque Transducer (Preferred):**
  Insert an inline rotary torque sensor (e.g., Futek / Lorenz / HBM) between the AKE90's custom shaft adapter and the flexible coupling to measure true shaft torque.
* **Option B — Reaction Torque Arm (Alternative):**
  Mount the AKE90 chassis on a pivot bearing with a static load cell acting as a reaction torque arm to measure mechanical reaction torque directly.

---

## 🔌 Stage 3: Electrical & Drive Wiring

You are managing two completely separate electrical loops: a high-voltage AC/DC generating loop (Alxion) and a low-voltage DC driving/regenerating loop (AKE90).

```
[DC Power Supply / Battery Sim] <---> [AKE90 Motor Controller] <---> AKE90
                                                                      |
                                                             (Flexible Coupling)
                                                                      |
[Braking Resistors] <---> [4-Quadrant PM Servo Drive] <-----------> Alxion 300STK
```

### 1. Wiring the AKE90 (Drive Under Test)
* Connect the AKE90 phase leads to its low-voltage BLDC/PMSM inverter board.
* Power the controller using a bidirectional DC power supply or a heavy-duty battery simulator (configured for 48V or 96V).
* **Important:** Standard one-way lab power supplies will trip or shut down when the AKE90 regenerates power back into the DC bus. If a bidirectional supply is unavailable, place a massive 48V/96V battery bank or a brake chopper circuit in parallel across the DC bus to absorb transient regen energy safely.

### 2. Wiring the Alxion 300STK (Active Load Motor)
* Connect the three AC phase lines and internal feedback sensor (resolver or encoder) from the Alxion enclosure to your 4-quadrant permanent magnet servo drive.
* **Energy Dissipation:** Connect a properly sized dynamic braking resistor bank to the DC bus of the Alxion servo drive to safely dump the energy generated when loading the AKE90.

---

## 🎮 Stage 4: Control System Commissioning

To perform both tests successfully, toggle which drive controls **Speed** and which drive controls **Torque**.

### Test 1: Mapping the Operating Points (Standard Load Profile)

* **AKE90 Drive Configuration:** Set the AKE90 controller to **Speed Control Mode**. Program a test script to step through target velocities (e.g., 30 rpm, 60 rpm, 90 rpm, 120 rpm).
* **Alxion Drive Configuration:** Set the Alxion servo drive to **Torque Control Mode**. Program it to command a precise resisting counter-torque (e.g., 10 N·m scaling up to 55 N·m).
* **Data Logging:** At each steady-state step, record the AKE90's DC bus voltage, DC current, true shaft torque, and shaft speed to calculate the total electromechanical efficiency map.

### Test 2: Testing AKE90 Regeneration (4-Quadrant Mode)

* **Alxion Drive Configuration:** Set the Alxion drive to **Speed Control Mode**. The Alxion acts as the prime mover, forcing the entire drivetrain to spin at a fixed velocity.
* **AKE90 Drive Configuration:** Switch the AKE90 controller to **Torque Control Mode** and command a negative torque value (opposing the direction of rotation).
* **Result:** The Alxion backdrives the AKE90 joint. The AKE90 acts as a generator/brake, converting mechanical kinetic energy from the Alxion back into electrical DC current. Measure the power coming out of the AKE90 phase lines and flowing back into the DC bus to compute pure regenerative efficiency.
