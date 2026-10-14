# Simulation MCP requirements

Antigravity should use locally configured MCP servers for MATLAB/Simulink, PSIM, and LTspice. This folder defines expected capabilities and configuration contracts. It must not contain secrets, licenses, or machine-specific installation paths.

## Required MCP servers

### MATLAB / Simulink

Expected capabilities:
- Open and inspect .slx models
- Run Simulink simulations
- Create and edit MATLAB scripts and Simulink models
- Read and write .mat or CSV results
- Run parameter sweeps
- Export figures and numerical results

### PSIM

Expected capabilities:
- Open, create, and run PSIM schematics
- Configure inverter, motor, load, and measurement blocks
- Run transient simulations
- Export waveform and loss data to CSV or MATLAB-compatible formats
- Support parameter sweeps where available

### LTspice

Expected capabilities:
- Open, create, and run .asc schematics or netlists
- Run transient simulations
- Export waveforms to CSV
- Support component/model substitution and parameter sweeps

## Runtime rules for Antigravity

- Discover actual MCP tool names before calling them
- Do not assume a tool exists
- Do not fabricate simulation results
- If a tool is unavailable, generate runnable source files and mark the study NOT EXECUTED
- Save native model files and CSV exports under the relevant study's models/ and results/ folders
- Record solver settings, timestep, model revision, parameter provenance, and convergence checks

## Security

- Never commit license files, API keys, tokens, or machine-specific credentials
- Keep real MCP configuration in a local, git-ignored file
- Use mcp_config.example.json only as a template
