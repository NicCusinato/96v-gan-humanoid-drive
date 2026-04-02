# data_raw

Raw experiment outputs: simulation exports, oscilloscope captures, power analyser dumps.

## Naming convention

One subfolder per experiment, named: `YYYY-MM-DD_<phase>_<short_description>`

```
data_raw/
  2026-04-07_phase0_si_baseline_fs_sweep/
    README.md          # inputs, what was swept, where to find processed data
    *.csv              # raw waveform / efficiency exports
    *.mat              # MATLAB workspace snapshots
  2026-05-14_phase1_gan_thermal_bench/
    README.md
    *.csv
```

## Rules
- Never hand-edit raw files; always copy, then process in MATLAB to `data_proc/`
- The `README.md` inside each folder is mandatory — log inputs and Jira ticket (KAN-XX)
- Large files (>50 MB total per experiment): store on NAS, note the path in the folder README
- CSV files in `phaseX/measurements/` are tracked (see `.gitignore` exception); bulk raw data here is not

## Processed data

Post-processed results (plots, summary tables) go in `data_proc/` with matching subfolder names.
