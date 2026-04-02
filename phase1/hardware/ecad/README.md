# phase1/hardware/ecad

Electronic CAD files for the Phase 1 leg inverter hardware (schematic + layout).

## Supported tools
- **Altium Designer** — place project under `altium/<board_name>/`
- **KiCad** — place project under `kicad/<board_name>/`

## Structure
```
ecad/
  altium/
    leg_inverter_v1/        # .PrjPcb, .SchDoc, .PcbDoc
      out/
        YYYY-MM-DD_revX_fab/  # gerbers, assembly files, PDF schematics
  kicad/
    leg_inverter_v1/        # .kicad_pro, .kicad_sch, .kicad_pcb
      out/
        YYYY-MM-DD_revX_fab/
```

## Rules
- Commit source files; ignore generated outputs (see root `.gitignore`)
- Store fab releases in timestamped `out/` subfolders
- Use Altium's built-in VCS diff support or KiCad schematic diff for reviews
