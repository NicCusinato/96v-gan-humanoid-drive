# LTspice MCP

## Purpose

Use for device-level validation: GaN half-bridge switching, gate-drive behavior, bootstrap/isolated bias operation, parasitic sensitivity, ringing, and dead-time timing checks.

## Minimum tool contract

Antigravity must discover the actual tool names at runtime. Expected categories:

- Open/create/run .asc schematics or SPICE netlists
- Modify component values and simulation commands
- Run transient simulations
- Export waveforms to CSV
- Substitute device models and parameters
- Run parameter sweeps where supported

## Required outputs

For every run, retain:

- .asc or netlist file
- Device/driver model source and version
- Simulation settings
- Parameter manifest
- Raw waveform exports
- Convergence and timestep information
- RESULTS.md entry

## Validation rules

- Use only verified GaN FET and gate-driver models
- Do not invent missing vendor models
- Do not extrapolate transient ratings into continuous operating ratings
- Do not add large gate/switch-node test stubs that compromise the GaN power loop
- Treat LTspice results as device/circuit-level evidence, not a substitute for PSIM system-level efficiency maps
