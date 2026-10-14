# Antigravity MCP prompts

Run the shared setup first, then one study at a time. Each study inherits the setup requirements.

## Shared setup prompt

Work in NicCusinato/96v-gan-humanoid-drive. Inspect README.md, phase0/specs/, phase1/simulation/VirtualSpeedDyno.slx, VirtualSpeedDynoParams.m, gait scripts, existing FOC examples, and phase1/simulation/pcb_validation/mcp/ before changing anything.

Discover the MATLAB/Simulink, PSIM, and LTspice MCP servers that are actually configured in this Antigravity environment. Use only tools advertised by those MCP servers. Do not invent tool names, simulate an MCP response, or claim a run without a returned artifact or log.

Read phase1/simulation/pcb_validation/mcp/README.md and the per-tool requirement files. If a required MCP server or tool is unavailable, search online only for reputable, actively maintained MCP implementations and report installation requirements, supported tools, license implications, and security considerations. Do not install or configure anything without user approval. If no suitable MCP is available, generate runnable MATLAB/Simulink, PSIM, and LTspice source files plus automation scripts, and mark affected simulations NOT EXECUTED.

Preserve existing models. Create new studies under phase1/simulation/pcb_validation/<study>/.

Read pcb_validation/README.md. Build a shared parameter/provenance manifest identifying measured, supplier-provided, datasheet-derived, and assumed values. The motor is the custom 96 V rewound AKE90, not the stock winding. Treat 10 A RMS continuous, approximately 25 A transient, 84-100 V bus, and 100 kHz PWM as provisional. Define phase RMS, instantaneous peak, dq scaling, and DC input current explicitly. Do not hardcode a 30 A trip threshold. Confirm device voltage ratings and maximum bus including regeneration before proposing parts.

Use MATLAB for automation, post-processing and control models; PSIM for inverter/motor and energy studies; LTspice for device-level gate-drive and switching studies where validated models exist. Export CSV plus native model files. Record model hashes or Git revisions, solver and timestep settings, parameter provenance, units, energy balance and convergence checks. Keep transient simulation data distinct from measured bench data. Every study must produce RESULTS.md with completed runs, limitations, plots, and a PCB decision table: requirement, evidence, uncertainty, recommended value, and unresolved hardware test. Do not commit automatically; present the diff for review.

## 08 Voltage margin and motor envelope - run first

Create 08_voltage_margin_back_emf. Using MATLAB/Simulink and/or PSIM, audit resistance/inductance/back-EMF conventions, pole pairs, encoder angle, gear ratio, and torque constant. Use measured rewind parameters if available; otherwise run labelled sensitivity scenarios. Generate the torque-speed envelope at 48 V and 96 V on the same rewound motor, showing modulation limits, current limits, voltage saturation, field-weakening assumptions, and thermal constraints. Keep stock-48-V versus rewind-96-V comparison separate. Export required phase voltage, phase RMS/peak current, dq current, and DC current. Identify missing supplier/measurement inputs before approving PCB current ratings.

## 01 GaN device selection

Create 01_gan_device_selection. Verify actual datasheet voltage ratings and model availability for EPC2302, EPC2218, EPC2305 and suitable higher-voltage alternatives. Reject candidates without adequate maximum-bus plus overshoot margin. In PSIM compare conduction, switching, dead-time reverse-conduction, gate-drive and capacitor losses over the validated motor envelope. Use temperature-dependent RDS(on), nonlinear output-charge behavior and suitable switching-energy data; identify model limitations. Use LTspice for representative half-bridge checks if validated vendor models exist. Do not invent missing models or extrapolate transient ratings into operating ratings. Produce a ranked device table with uncertainty and a shortlist, not an automatic final approval.

## 02 Switching frequency

Create 02_switching_frequency_sweep. Sweep 25, 50, 75, 100 and 150 kHz across representative torque-speed points using the selected admissible device candidates. Separate averaged gait-energy runs from switching-resolved current-ripple runs. Hold motor parameters and operating commands consistent. Sweep timestep until ripple and losses converge. Report current ripple, RMS current, inverter losses, gate-drive losses, predicted motor losses where supported, and junction temperature. Recommend frequency based on the combined tradeoff, not minimum ripple alone.

## 03 Dead time

Create 03_dead_time_sweep. Use PSIM and LTspice to sweep 25, 50, 75, 100, 150 and 200 ns where driver timing permits. Include propagation-delay mismatch, PWM resolution, gate resistance, reverse conduction and parasitic uncertainty. Evaluate positive/negative current and zero crossings. Report voltage distortion, current ripple, reverse-conduction energy, ringing and timing margin. Do not call GaN reverse conduction body-diode conduction. Flag shoot-through risk and distinguish commanded from effective dead time. Recommend a safe initial setting pending measured timing validation.

## 04 DC link

Create 04_dc_link_sizing. In PSIM model local ceramic/film capacitors, bulk capacitance, capacitor ESR/ESL, source impedance, cable inductance and regenerative operation. Examine steady PWM ripple, load steps and four-drive bus interaction where meaningful. Include ceramic DC-bias derating and capacitor RMS-current limits. A non-sinking lab supply must not be assumed to absorb regeneration. Derive local capacitance and central bulk requirements separately. Use LTspice for high-frequency loop transients if appropriate. Report ripple, bus peak, capacitor heating, discharge/precharge requirements and unresolved brake-chopper or regenerative-supply needs.

## 05 Current sensing

Create 05_current_sensing. Compare inline shunt, low-side shunt and Hall alternatives with real candidate specifications. Use MATLAB/Simulink for PWM-triggered sampling, reconstruction windows, offsets, noise, delay and loop stability. Use LTspice for amplifier/filter settling and common-mode behavior only if the model supports it. Include three phase currents and DC input current as separate functions. Report measurement range, shunt dissipation, gain, ADC headroom, anti-alias filtering, blanking/sample timing, calibration and hardware-trip requirements. Assess instrument accuracy for efficiency measurements separately from control-sensor adequacy.

## 06 Gate drive

Create 06_gate_drive. Using LTspice MCP, compare suitable bootstrap and isolated-bias architectures with verified GaN-compatible drivers. Apply the selected FET gate limits, delay/mismatch specifications, supply tolerances and realistic gate/power-loop parasitics. Check startup, duty extremes, bootstrap refresh, UVLO, ringing, false turn-on and disabled-state behavior. Do not impose an unsupported absolute delay below 10 ns; derive requirements from effective dead-time and protection timing. Report gate resistor/bias/decoupling selections, timing margins and constraints that must be confirmed on hardware.

## 07 Coupled thermal model

Create 07_thermal_electro_thermal. Use MATLAB/Simulink and PSIM loss outputs for a coupled motor-winding, stator, housing, PCB, heat-spreader and GaN thermal network. Include motor heating of the shared housing and temperature-dependent losses. Use measured or datasheet-supported thermal impedances; show sensitivity for unknown interfaces rather than claiming precise temperatures. Evaluate continuous loading, defined-duration peaks and representative gait cycles. Report temperature limits, loss allocation, derating and sensor placements. If 3D thermal FEA is unavailable, clearly state that the lumped model does not establish layout thermal performance.

## 09 Fair efficiency comparisons

Create 09_efficiency_vs_48v_si. Inspect existing baseline models and define separate experiments: same rewound motor at 48/96 V to study voltage effects, comparable Si/GaN stages at equal bus where possible to study device effects, and stock-48-V versus rewind-96-V system comparison only when motor data support it. Match delivered torque-speed trajectories, tracking error and temperature assumptions. Report voltage-limited cases separately. Higher bus voltage alone must not be assumed to halve phase current at the same torque and unchanged winding. Account for motoring/regeneration and use energy/loss metrics near zero mechanical power. State whether efficiency includes inverter only, motor, gearbox or auxiliaries. Export traceable maps and loss budgets with uncertainty; do not infer per-switch switching energy from an averaged ideal-switch model.

## Final synthesis prompt

After completed studies, produce PCB_DECISION_MATRIX.md and PROTOFLOW_INPUTS.md in pcb_validation/. Summarize verified versus provisional device ratings, maximum bus, phase-current definitions, PWM/dead time, DC-link values, sensing, driver, thermal limits and unresolved tests. Add a TI software/peripheral compatibility audit for F28069M versus a selected F28003x part: CAN versus CAN FD, ADC/PWM timing, encoder interface, debugger and SDK migration. Do not imply MotorWare/InstaSPIN binaries or LaunchPad pinouts migrate unchanged. Include a pin/ADC/channel budget for research telemetry, a hardware synchronization pulse for instruments, decimated telemetry versus triggered local buffers, and low-capacitance oscilloscope probing requirements. Avoid adding large gate/switch-node test stubs that compromise the GaN loop. Provide the diff for review and list which outputs are suitable as inputs to ProtoFlow electrical design and layout runs.
