# MATLAB / Simulink MCP

## Purpose

Use for VirtualSpeedDyno execution, motor-envelope analysis, parameter sweeps, FOC/control studies, loss post-processing, efficiency mapping, and figure/data export.

## Minimum tool contract

Antigravity must discover the actual tool names at runtime. Expected categories:

- List/open MATLAB and Simulink files
- Run a Simulink model with parameter overrides
- Read/write MATLAB scripts and MAT/CSV data
- Create or modify Simulink blocks/signals
- Execute MATLAB functions or scripts
- Export plots and numerical results

## Required outputs

For every run, retain:

- Model file or Git revision
- Parameter manifest
- Solver settings and timestep
- Raw CSV/MAT output
- Figures
- Energy-balance and convergence checks
- RESULTS.md entry

## Safety rules

- Do not overwrite VirtualSpeedDyno.slx or existing FOC examples
- Create study-specific copies under the relevant pcb_validation study folder
- Treat custom rewound AKE90 parameters as measured or clearly labelled sensitivity assumptions
- Do not claim a simulation ran without a returned artifact or log
