# True FOC Encoder Offset Measurement Guide

## Problem
The Dashboard was showing a **meaningless "Offset" value** — it was just averaging the **rotor position** during measurement, not the true FOC encoder offset.

Every time you spin the motor, the rotor settles at a different mechanical angle, so this "offset" number changed (e.g., 93.3° → 120°).

## Solution
The **true FOC encoder offset** is calculated by the controller **during the closed-loop ramp phase** (first 1.5 seconds):

```matlab
offset_foc = theta_ol - POLE_PAIRS * th_m;
```

This offset is what determines **magnetic alignment** — it's the electrical angle difference between the field and the rotor.

---

## How to Measure the True Offset

### Step 1: Update Simulink Model
1. Open `Hardware_Diagnostics_Test.slx`
2. Double-click the `Open_Loop_Controller` MATLAB function block
3. Follow the 4 edits in `EDIT_OPEN_LOOP_CONTROLLER.txt`
   - Add `offset_foc_out` as 5th function output
   - Preallocate it
   - Output it during ramp phase (Phase 1)
   - Output it during steady-state (Phase 2)
4. Click OK
5. **Save the model** (Ctrl+S)
6. Re-deploy to hardware

### Step 2: Run Closed-Loop FOC Test
1. Run the Dashboard: `Live_Diagnostics_Dashboard()`
2. Click **"CL FOC 100 RPM"** button
3. Watch the state label:
   - **First 1.5 seconds:** STATE: STARTUP BREAKAWAY RAMP... (orange)
     - During this phase, `offset_foc` is being calculated from the ramp trajectory
   - **After 1.5 seconds:** STATE: CLOSED-LOOP FOC (100 RPM) (green)
     - Now using the calculated offset for smooth angle tracking

### Step 3: Verify Offset
The position label will update to show:
- During open-loop: `"SPI: X.X° (rotor avg) | eQEP: Y.Y° | Run CL FOC to measure true offset"`
- During CL FOC: `"SPI: X.X° (rotor) | eQEP: Y.Y° | CL FOC offset computing..."`

The **true offset** is calculated inside the controller and used for alignment.

---

## Understanding the Result

### Good Offset (Motor Aligns)
- Current during ramp rises from 0 → 1-2A
- Motor smoothly accelerates to target speed
- No cogging or stalling

### Bad Offset (Motor Stalls)
- Current locks at ~0.16A
- Motor won't accelerate
- → Offset is too far from true value

---

## If Motor Still Stalls

After the ramp phase (after 1.5s), the closed-loop controller uses the calculated offset. If it **still stalls**:

1. **Check PWM modulation:**
   - Increase `m_ramp` in Open_Loop_Controller to 0.25 for more torque during ramp
   - Change line: `m_ramp = single(0.06);` → `m_ramp = single(0.25);`

2. **Check current sensor offsets:**
   - When DC bus is powered, should read ~2041/2059 counts (1.65V bias)
   - Use Dashboard to verify: `"Ia: X.XX A (counts) | Ib: Y.YY A (counts)"`

3. **Check speed controller gains:**
   - `KP_V = 0.020` and `KI_V = 0.025` may be too aggressive
   - Reduce by 50% if oscillating

---

## Summary

✅ **Before:** Dashboard showed rotor position (meaningless offset that changed every run)
✅ **Now:** Dashboard shows when true offset is being calculated (during CL FOC ramp)
✅ **Result:** You can verify that closed-loop FOC is using the correct magnetic alignment

The true offset (offset_foc) is **stable and deterministic** — it depends only on the electrical properties of the motor and encoder, not on the rotor's starting position.
