# 96V GaN Humanoid Leg Drive

> **MSc Research Project — Queen's University, Kingston, Ontario**
> Target: Higher efficiency than 48V Silicon MOSFET baseline for humanoid robot joint drives

[![Jira](https://img.shields.io/badge/Jira-KAN_Board-0052CC?logo=jira)](https://cusinatoqueens.atlassian.net/jira/software/projects/KAN/list)

---

## Project Overview

This project designs a **96V GaN-based 3-phase motor drive system** for the lower half of a humanoid robot (2 legs, 6+ joints). The hypothesis is that moving from a 48V Silicon MOSFET baseline to 96V GaN inverters reduces total drive system losses through:
- Higher voltage → lower RMS current for the same power
- GaN zero-reverse-recovery → lower switching losses at higher frequency
- Higher switching frequency → smaller passive components, lower current ripple

Control uses **downloaded gait trajectories** (not custom locomotion control). Focus is entirely on power electronics design and characterization.

---

## Timeline

| Phase | Period | Focus |
|-------|--------|-------|
| Phase 0 | Apr 2026 (Month 1) | Specs, literature review, 48V Si baseline characterization, gait trajectory extraction |
| Phase 1 | May–Jul 2026 (Months 2–4) | Box-mounted 96V GaN inverter PCB, FOC firmware, efficiency benchmarking |
| Phase 2 | Aug–Nov 2026 (Months 5–8) | Custom mechanical leg + miniaturized in-joint GaN drives |
| Phase 3 | Dec 2026–Feb 2027 (Months 9–11) | System characterization, efficiency maps, thesis writing |
| Phase 4 | Mar–Apr 2027 (Month 12) | Custom motor co-optimized with 96V GaN drive |

---

## Repository Structure

```
96v-gan-humanoid-drive/
├── phase0/               # Setup, baseline, specs
│   ├── specs/            # Electrical and mechanical target specs
│   ├── literature/       # Paper notes and references
│   ├── baseline_48v_si/  # 48V Si loss models and characterization data
│   └── gait_data/        # Downloaded joint trajectory datasets
├── phase1/               # Box-mounted 96V GaN inverter
│   ├── hardware/         # PCB schematics (KiCad .asc), BOM
│   ├── simulation/       # PSIM/PLECS loss models, LTspice (.asc files)
│   ├── firmware/         # MCU FOC code (C/Python)
│   ├── measurements/     # Efficiency test results (CSV, plots)
│   └── matlab/           # MATLAB scripts for analysis
├── phase2/               # Custom leg + in-joint GaN drives
│   ├── mechanical/       # CAD exports (drawings, STEP files for reference)
│   ├── hardware/         # Miniaturized in-joint PCB
│   ├── simulation/       # Electro-thermal co-simulation
│   └── measurements/     # Long-duration efficiency and thermal tests
├── phase3/               # System characterization + thesis
│   ├── efficiency_maps/  # Full efficiency map data and plots
│   ├── sensitivity/      # Switching freq / dead time sensitivity data
│   └── writing/          # Thesis chapter drafts
├── phase4/               # Custom motor design
│   ├── motor_fea/        # Maxwell/FEMM simulation exports
│   ├── winding/          # Winding design scripts
│   └── measurements/     # Custom motor + GaN test results
├── docs/                 # Shared documentation, reports
├── README.md
├── RESEARCH_LOG.md       # Running lab notebook — update with every key result
└── .gitignore
```

---

## Tool Stack

| Tool | Purpose |
|------|---------|
| PSIM / PLECS | Power circuit simulation, loss modelling |
| LTspice | Gate driver and auxiliary circuit simulation |
| MATLAB / Simulink | Data analysis, FOC modelling |
| Python | Data processing, plotting, automation |
| KiCad | PCB schematic and layout |
| Fusion 360 | Mechanical CAD (cloud-versioned, not in Git) |
| FEMM / Maxwell | Motor FEA (Phase 4) |
| ROS2 / Gazebo | Gait trajectory reference |
| Git / GitHub | Version control |
| Jira (KAN) | Task tracking |

---

## Git Workflow

Branch naming convention tied to Jira:
```
feature/KAN-XX-short-description
fix/KAN-XX-short-description
analysis/KAN-XX-short-description
```

Commit message convention:
```
KAN-XX: Brief description of what changed
```

This auto-links commits to Jira tickets via the GitHub for Jira integration.

---

## Key Targets (updated as defined)

| Parameter | Value |
|-----------|-------|
| Bus voltage | 96V |
| Joints per leg | TBD |
| Peak joint current | TBD A |
| Switching frequency | TBD kHz |
| GaN device candidates | TBD |
| 48V Si baseline device | TBD |
| Target efficiency improvement | TBD % |

---

*See [RESEARCH_LOG.md](RESEARCH_LOG.md) for dated entries of decisions, results, and blockers.*
