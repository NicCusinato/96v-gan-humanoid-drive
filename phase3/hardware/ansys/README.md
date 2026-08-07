# phase1/hardware/ansys

Ansys simulation studies for Phase 1 (thermal, EM, structural).

## Structure
```
ansys/
  thermal_inverter_YYYY_QX/   # one folder per study
    setup_notes.md            # inputs, mesh settings, solver config
    *.wbpj or *.aedt          # project file (commit this)
    results/                  # ignored by .gitignore (too large)
  em_field_YYYY_QX/
```

## Rules
- Commit project/setup files and `setup_notes.md` with solver parameters
- Ignore `*_files/`, `*.rst`, `*.rth` result databases (see root `.gitignore`)
- If a solve took >4 hrs: store result on NAS and note the path in `setup_notes.md`
- Use timestamped subfolder names: `thermal_inverter_2026_Q2`
