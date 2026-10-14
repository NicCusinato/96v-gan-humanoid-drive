# PCB validation simulations

Motor: custom 96 V rewound AKE90. Do not substitute stock winding parameters.

Use one folder per study, with tool-specific subfolders inside each study. Do not duplicate models across parallel tool-based trees. Existing virtual dyno and FOC files must remain unchanged unless a reviewed patch is justified.

## Study folders

- 01_gan_device_selection
- 02_switching_frequency_sweep
- 03_dead_time_sweep
- 04_dc_link_sizing
- 05_current_sensing
- 06_gate_drive
- 07_thermal_electro_thermal
- 08_voltage_margin_back_emf
- 09_efficiency_vs_48v_si

Each study should contain README.md, models/matlab/, models/psim/, models/ltspice/ as needed, scripts/, results/, and RESULTS.md. Create folders when adding real files; do not claim an empty folder is tracked in Git.

RESULTS.md must identify model version, parameter provenance, solver settings, convergence checks, completed runs, limitations, and PCB implications. Separate simulated predictions from hardware measurements.

## Provisional targets, not approved specifications

- Bus: 96 V nominal; 84-100 V provisional operating range. Confirm supply maximum and regeneration behavior.
- Current: 10 A RMS continuous and approximately 25 A transient provisional. Define whether transient current means phase instantaneous peak, RMS, or dq current before sizing.
- No automatic 30 A hardware trip threshold: derive from motor limits, sensor errors, delay, and device protection requirements.
- PWM: 100 kHz nominal; investigate 25-150 kHz.
- Current-loop execution: at least 20 kHz provisional; not equivalent to closed-loop bandwidth.
- Dead time: sweep around 25-200 ns where hardware supports it; validate timing tolerance.
- Junction temperature: 125 C provisional design ceiling.
- Loss and efficiency targets must use a stated output-power boundary. A 5 W loss target and 96% efficiency are not interchangeable.

## Critical design gates

Do not approve a 100 V-rated FET for a bus approaching 100 V. Check actual candidate ratings, bus maximum, overshoot, regenerative rise, and derating; include suitable higher-voltage candidates. Do not use transient ratings as normal operating margin.

Rewind resistance, inductance, back-EMF, pole pairs, insulation, and thermal parameters require supplier evidence or measurements. Ideal doubled-turns scaling is a sensitivity scenario only.

Use averaged models for gait and energy analysis, and switching-resolved models for ripple, dead time, and switching waveforms. Check timestep convergence. Do not claim switching losses from ideal-switch waveforms.

Onboard telemetry supports control and repeatability; calibrated external power/torque instruments are needed for defensible efficiency and loss validation.

See ANTIGRAVITY_PROMPTS.md for execution instructions.
