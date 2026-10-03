# 96V GaN Humanoid Drive: Clean FOC Deployment Package

This package consolidates all verified simulation models, low-level C drivers, calibration routines, and engineering guides for the **TI LAUNCHXL-F28069M + EPC9147B/EPC91200 GaN Inverter + CubeMars AKE80-8 KV30 Actuator** drive system.

---

## Directory Organization

```
FOC_DEPLOYMENT_PACKAGE/
│
├── calibration/        # Hardware Bring-Up & Offset Calibration (Workflows 1 & 2)
│   ├── EncoderOffsetCalibrationF28069mLaunchPad.slx  # QEP/SPI electrical offset target model
│   ├── OffsetCalibrationF28069mHost.slx              # Real-time QEP/SPI telemetry host model
│   ├── OffsetCalibrationCallback.m                   # Model parameter callback
│   ├── setup_offset_calculations.m                   # Master workspace setup script
│   ├── OpenloopMotorControlF28069mLaunchPad.slx      # Open-loop V/f scalar spin target model
│   ├── OpenloopMotorControlHost.slx                  # Open-loop host model
│   └── as5047p_spi.* / qep_spi_sync.*                # Synchronized driver sources
│
├── foc/                # Field-Oriented Control (FOC) Execution (Workflow 3)
│   ├── mcb_pmsm_foc_qep_f28069LaunchPad.slx         # Hardware deployment FOC target model
│   ├── mcb_host_model_f28069m.slx                   # Real-time closed-loop FOC host dashboard
│   ├── mcb_pmsm_foc_qep_f28069LaunchPad_data.m      # FOC parameter initialization script
│   ├── foc_qep_data.m                               # Master FOC data script with calibrated gains/offsets
│   ├── mcb_pmsm_foc.slx / mcb_pmsm_foc_system.slx   # Modular FOC simulation components
│   ├── current_control_algorithm.slx                # Exportable 20 kHz current controller
│   ├── speed_control_algorithm.slx                  # Exportable 2 kHz speed controller
│   └── as5047p_spi.* / qep_spi_sync.*               # SPI absolute pre-seeding drivers
│
├── drivers/            # Clean Standalone Embedded C Drivers
│   ├── as5047p_spi.c / .h                            # 14-bit AS5047P SPI driver (20 kHz, odd parity)
│   └── qep_spi_sync.c / .h                           # Hardware eQEP boot pre-seeding (spi_raw >> 2)
│
└── docs/               # Complete Engineering Reference Guides
    ├── AS5047P_SPI_QEP_HARDWARE_SYNC_GUIDE.md        # Full theory, math, wiring & 4-stage test proofs
    ├── CONFIGURATION_TO_CUSTOM_HARDWARE_GUIDE.md     # Step-by-step hardware adaptation manual
    ├── HARDWARE_MAPPING_AND_PROJECT_GUIDE.md         # Full BoosterPack pin mapping & register guide
    └── OFFSET_CALCULATIONS_README.md                 # Calibration procedures & bench notes
```

---

## Calibrated Hardware Bench Constants

* **ADC Zero-Current Biases (48V Bus Active):**
  * Phase A ($I_a$): `2058 counts` ($1.658\text{ V}$)
  * Phase B ($I_b$): `2057 counts` ($1.657\text{ V}$)
* **Electrical Position Offset:**
  * Measured Value: `0.0234 PU` ($8.42^\circ\text{ mechanical}$)
* **AS5047P SPI Absolute Sync:**
  * Bitwise Pre-Seed: `EQep1Regs.QPOSCNT = spi_raw >> 2`
  * Validated Tracking Error: $< 0.87^\circ\text{ mechanical}$ on cold boot
