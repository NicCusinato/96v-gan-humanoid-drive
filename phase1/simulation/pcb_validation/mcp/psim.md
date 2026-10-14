# PSIM MCP

## Purpose

Use for three-phase GaN inverter loss models, motor/load studies, DC-link ripple, dead-time effects, switching-frequency sweeps, and gait-profile energy analysis.

## Minimum tool contract

Antigravity must discover the actual tool names at runtime. Expected categories:

- Open/create/run PSIM schematics
- Modify component values and simulation settings
- Configure inverter, motor, load, measurement, and scope blocks
- Run transient simulations
- Export waveforms and loss data
- Run parameter sweeps where supported

## Required outputs

For every run, retain:

- PSIM schematic file
- Simulation settings
- Parameter manifest
- Raw waveform/loss exports
- Energy-balance check
- Timestep-convergence evidence
- RESULTS.md entry

## Modeling rules

- Use averaged models for gait energy analysis
- Use switching-resolved models for ripple, dead time, and switching waveforms
- Do not infer per-switch switching energy from ideal-switch averaged models
- Include dead-time reverse conduction, gate-drive loss, and capacitor loss where supported
- Do not assume a non-sinking lab supply absorbs regeneration
