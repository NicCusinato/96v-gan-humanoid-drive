# phase1/hardware/mech

Mechanical CAD files for Phase 1 hardware (heatsink, enclosure, mounting).

## Supported tools
- **Fusion 360** — cloud-versioned; place exported archives (`.f3z`) here for milestones only
- **SolidWorks / Inventor** — place native files here, one assembly per subfolder

## Structure
```
mech/
  heatsink_v1/            # native CAD + exports
    exports/
      YYYY-MM-DD_concept/ # STEP, drawings PDF
  enclosure_v1/
    exports/
```

## Rules
- Never hand-rename files in OS; use CAD tool rename to preserve references
- Commit native files and milestone STEP exports; ignore STL/3MF (see root `.gitignore`)
- Fusion 360: commit `.f3z` archive at major milestones; daily work stays in Fusion cloud
