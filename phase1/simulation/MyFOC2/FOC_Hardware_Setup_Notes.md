# FOC hardware setup notes

## Intended bring-up path

The virtual speed dyno is the simulation-side test system for the real FOC controller. The initial hardware target is:

- EPC91200 three-phase GaN inverter using the EPC2305 power stage
- CubeMars AKE80-8 KV30 actuator/motor with its 8:1 planetary reduction
- TI LAUNCHXL-F28069M / TMS320F28069M controller

The separate hardware model is `SAFECOPYFOC_Motor_Control_Hardware.slx`. The intended position path is:

1. Use SPI to read the absolute position sensor while the inverter is disabled.
2. Establish the electrical-angle offset from the absolute mechanical position and pole-pair count.
3. Enable the QEP path only after the offset is known and validated.
4. Use QEP for the time-critical FOC angle/speed feedback during normal operation.
5. Keep host-model telemetry and commands available for calibration, limits, and monitoring.

This avoids using an uncontrolled alignment pulse or a random motor jerk as the first position reference. The SPI sensor part number, frame format, zero offset, and encoder location still need to be confirmed from the actual actuator and wiring.

## Simulation parameter decisions

`VirtualSpeedDynoParams.m` is now a single short, non-destructive simulation. It passes all model variables through `Simulink.SimulationInput` and does not clear the MATLAB workspace or save changes into the model.

The Simscape PMSM block is parameterized with the following AKE80-8 values:

| Quantity | Value | Basis |
|---|---:|---|
| DC bus | 48 V | AKE80-8 rated voltage |
| Pole pairs | 21 | Manufacturer specification |
| Kt | 0.32 N*m/A | Manufacturer specification |
| Phase resistance | 0.435 ohm | 0.870 ohm line-to-line / 2, star winding |
| `Ld = Lq` | 495 uH | 990 uH line-to-line / 2, first-pass SPMSM model |
| PM flux linkage | 0.01016 Wb | `Kt/(1.5*p)` using the model's peak-current convention |
| Inertia | 1.47175e-4 kg*m^2 | 1471.75 g*cm^2 conversion |
| First test speed | 60 rpm rotor | Conservative fixed-speed dyno point |
| First torque command | 0.25 N*m | Small current-loop test, not external load |
| Switching frequency | 20 kHz | Conservative first F28069M/EPC91200 test point |

The 495 uH / 0.435 ohm pair gives an electrical time constant of about 1.14 ms, agreeing with the published 1.13 ms. The model's ideal angular-velocity source receives `w_ref` in **rad/s** because its Simulink-PS Converter is configured with `Unit=1` (SI base units). The PMSM initial-condition field is separately displayed with an `rpm` unit, so the script keeps both `speed_ref_rpm` and `speed_ref_rad_s` explicit. The old sweep script's speed values were numerically compatible with the source, but its comments and labels were ambiguous; the bring-up script removes that ambiguity.

For the EPC91200/EPC2305 model, nominal values use 2.2 mOhm typical `RDS(on)`, 1.1 V typical threshold, and 0.2 K/W junction-to-case. The second thermal resistance is set to 21 K/W as an EVB-level placeholder. `E_on` and `E_off` use a tiny positive numerical floor because the detailed converter requires values greater than zero; they are not a board-level switching-energy characterization. Do not use this run for inverter-loss or thermal claims.

The first successful run completed for 0.100 s with `simlog` and `tout`, held 60 rpm, and produced approximately 0.807 A q-axis current for the 0.781 A command (0.69 A phase RMS). The logged PMSM torque was approximately -0.258 N*m for `T_e_ref = +0.25 N*m`; this is a sign-convention check item for the Park transform, phase order, and mechanical sensor orientation. Resolve it against the intended positive rotation direction before enabling hardware torque. The model structural check also reports one pre-existing warning: the output of `VirtualSpeedDyno/PID Controller2` named `T_e_ref` has no destination. It does not prevent this current-loop run, but should be cleaned up when the speed-loop model is revised.

## Hardware safety gates before power

- Keep the first DC bus at the AKE80-8's 48 V rating; do not use the EPC91200's 96 V intended operating point with this motor without a separately validated motor/insulation/overspeed plan.
- Start with inverter hardware disabled and verify PWM polarity, dead time, ADC offsets, current-sense sign, bus-voltage scaling, and over-current shutdown.
- Confirm whether the absolute sensor and QEP are on the motor rotor side or gearbox output side. The 8:1 ratio changes the electrical-angle relationship and speed conversion.
- The current CubeMars AKE80-8 product page lists the bare AKE80-8 as having no built-in encoder, while CubeMars separately advertises a compatible driver board with a 14-21 bit single-turn absolute encoder. Confirm which sensor/driver-board variant is physically installed before assigning QEP lines or SPI framing.
- Set software current limits below the motor's published 12 A peak / 4.8 A rated values until thermal and sensor calibration are verified.
- Validate the SPI-derived absolute angle against QEP counts at standstill before enabling torque.

## Primary references checked

- EPC91200 product page: https://epc-co.com/epc/products/evaluation-boards/epc91200
- EPC91200 quick-start guide: https://epc-co.com/epc/Portals/0/epc/documents/guides/EPC91200_qsg.pdf
- EPC2305 datasheet: https://epc-co.com/epc/portals/0/epc/documents/datasheets/EPC2305_datasheet.pdf
- CubeMars AKE80-8 KV30 specification: https://www.cubemars.com/jp/goods-1165-AKE80-8.html
- TI TMS320F28069M product page: https://www.ti.com/product/TMS320F28069M
- TI LAUNCHXL-F28069M product page: https://www.ti.com/product/LAUNCHXL-F28069M/part-details/LAUNCHXL-F28069M
- MathWorks F28069M QEP offset calibration: https://www.mathworks.com/help/mcb/gs/quadrature-encoder-offset-calibration-pmsm-motor.html
- MathWorks F280049C QEP offset calibration: https://www.mathworks.com/help/ti-c2000/ug/QEP-Offset-calibration-example.html
- MathWorks custom-hardware position offset workflow: https://www.mathworks.com/help/mcb/ug/position-sensor-offset-calibration-for-quadrature-encoder-and-hall-sensor-in-foc.html

## QEP offset calibration compatibility audit

The MathWorks examples were imported under `reference_examples` for local comparison:

- `EncoderOffsetCalibrationF28069mLaunchPad.slx` is the correct reference target for the LAUNCHXL-F28069M.
- `mcb_pmsm_qep_offset_f280049C.slx` is a separate F280049C target example and is not a drop-in F28069M model.
- The shared algorithm contains the same open-loop rotor-position/QEP-index offset calculation and the same QEP decoder path. Both reference models use an associated host model over SCI and report a `PositionOffset` result.

The compatibility script is `QEP_Offset_Compatibility_Report.m`. Its current verified result is:

| Item | F28069M reference | Current hardware model | Result |
|---|---|---|---|
| Processor board | TI Piccolo F28069M LaunchPad | TI Piccolo F28069M LaunchPad | Match |
| QEP module/mode | eQEP1, 2x | eQEP1, 2x | Match |
| QEP counter range | `2^16-1` | `4*1024-1` | Encoder-dependent; confirm |
| QEP input polarity | A/B/index polarity enabled in reference | Polarity options disabled | Must reconcile |
| PWM hardware modules | ePWM1/ePWM2/ePWM3 | ePWM4/ePWM5/ePWM6 | Custom adapter mapping |
| ADC trigger/input setup | ePWM1 ADCSOCA, ADCINA0/ADCINB0 | Software trigger, ADCINA0/ADCINB7 paths | Must adapt |
| Offset algorithm in current model | Present in reference | Not integrated | Required |
| SPI absolute sensor driver | Not part of reference | Not present | Required |

The current MATLAB Function `Feedback_Processing` still uses a hard-coded `TH_OFFSET = 1.790708` rad, a hard-coded 21 pole-pair multiplier, and a hard-coded 4096-count mechanical scale. The offset must become a calibrated parameter/input; the QEP denominator must be derived from the actual encoder line count and selected 2x/4x mode. With the current 2x block setting, 4096 counts/rev implies a 2048-line encoder, not a 1024-line encoder.

The checked-in `EPC9147B_Schematic.pdf` is an EPC9147B B5217 Rev. 2.0 controller-interface drawing. It labels the digital interface as PWMH1/PWM1, PWMH2/PWM2, PWMH3/PWM3 and nEN, and the analog interface as Vbus, ADCIN_1/2/3 and IA/IB/IC. It is useful for the adapter audit, but it is not sufficient to prove the LAUNCHXL GPIO/ePWM4-6 and ADCIN assignments without the actual harness/pin map. The adapter must be treated as custom hardware until those nets are traced end-to-end.

Therefore the safe integration path is to reuse the F28069M calibration algorithm and host workflow, then replace only its hardware-driver layer with the existing EPC91200 adapter mappings. Do not deploy the F280049C target model unchanged, and do not enable torque until the encoder line count, index signal, A/B/index polarity, phase order, PWMH/PWM mapping, nEN logic, ADC channels, and current/bus-voltage scaling are confirmed.

The reference target models pass the Simulink structural audit. The current hardware model reports one real position-interface warning: `Feedback/eQEP` output `y2` (the index-latch output needed by the offset decoder) is currently unconnected. This is why the imported offset algorithm cannot simply be dropped into the current model yet. The custom `reference_examples/qep_algorithm.slx` shows the intended signal contract: QEP position count, QEP index latch, and `pmsm.PositionOffset` feed the mechanical-to-electrical position and speed path.

## Proposed F28069M/EPC adapter settings for offset calibration

Do not convert the F280049C model by changing only the top-level hardware board. Use `EncoderOffsetCalibrationF28069mLaunchPad.slx` as the target baseline and port the EPC91200 adapter driver layer into it. The following are the settings to use for the first F28069M target build:

| Area | Setting |
|---|---|
| Target board | `TI Piccolo F28069M LaunchPad` |
| Controller data type | `single` for first bring-up |
| PWM frequency | 20 kHz; `Ts = 50 us`; use `target.PWM_Counter_Period` (2250 counts at 90 MHz, up-down) |
| PWM modules | Keep the custom adapter map ePWM4/ePWM5/ePWM6 if those are the wired phase outputs; do not copy the F280049C ePWM6/ePWM5/ePWM3 map |
| PWM waveform | C2000 Type 1-4, up-down, CMPA from input, complementary A/B, active-high complementary dead-band |
| PWM dead-band | Existing model/reference value is 5 clock cycles for both rising and falling edges; retain only after confirming the EPC gate-driver requirement |
| Gate enable | Existing model selects GPIO bit 2 of GPIO50-GPIO55, i.e. GPIO52. The EPC9147B connector labels the line `nEN`, so verify/invert the logic if the adapter is active-low |
| Current ADC | F28069 C2000 Type 1, ADC module 1, simultaneous sample, 12-bit single-ended, SOC0/SOC1, acquisition window 7, triggered by the PWM master SOCA |
| Current ADC trigger | For the current ePWM4/5/6 map, use `ePWM4_ADCSOCA` if ePWM4 is the synchronized master; the present `Software` trigger is not the desired FOC sampling configuration |
| Current ADC channels | Assign SOC0/SOC1 to the physical IA/IB adapter nets. The existing model's ADCINA0 plus ADCINA3/ADCINB3 selection is only a candidate until continuity is checked against the EPC9147B analog connector |
| Bus-voltage ADC | Configure separately on the verified Vbus ADC pin; it is not a substitute for the IA/IB pair needed by the offset algorithm |
| QEP module | `eQEP1` |
| QEP mode | `Quadrature-count`, `2x resolution` |
| QEP position output | On; index-latch output on; connect both QEP outputs, including the index latch, into the position algorithm |
| QEP reset | `Reset on the first index event`, software initialization on, initial count 0 |
| QEP counter maximum | Use `2^16-1` for the F28069M offset target; let the QEP-slits parameter define the mechanical scale |
| QEP slits | Set to the encoder's actual A/B line count. With 2x mode, 4096 counts/rev implies a 2048-line encoder; do not assume 1024 lines |
| QEP input polarity | The imported F28069M reference has A/B/index inversion on/on/on, while the current custom model has off/off/off. Match the proven adapter wiring and validate positive count direction; this cannot be inferred from the 49C model |
| QEP index edge | Match the actual index waveform. The reference uses falling-edge latch; the current custom model uses rising-edge latch |
| Target SCI | SCI-A; use the F28069M SCI-A RX interrupt and ADCINT1 hardware interrupt scheduling from the reference target |
| Offset target SCI RX | `uint16`, one value, no header/tail, blocking on, timeout `0.019444 s` |
| Offset target SCI TX | SCI-A, frame size 1, blocking on |
| Host serial setup | 5.625e6 baud, 8 data bits, no parity, one stop bit, little-endian; this is the F28069M host setting, not the F280049C host's 1.5e6 baud |
| Calibration values | 21 pole pairs, 60 rpm calibration speed, start `Vd Ref = 0.05 pu`; increase only in small steps if the rotor does not move, while limiting current |

The ADC channel names, QEP polarity/edge, PWM phase order, and `nEN` logic are hardware facts rather than values that can be safely guessed from the 49C example. The offset example itself is a target code-generation workflow, not a normal simulation model; use the virtual dyno to validate the FOC algorithm, then validate these driver settings with the motor mechanically secured and the DC bus limited to 48 V.

## F280049C Site-2 peripheral map test harness

The current target is the LAUNCHXL-F280049C installed on Site 2 of the EPC9147B adapter. The small model `F280049C_Hardware_Map_Tests.slx` and its MATLAB helpers are the first hardware-facing artifact for this target. They intentionally test only one peripheral map at a time and keep GPIO34/nEN driven high so the EPC91200 gate driver remains disabled.

Use these files:

- `F280049C_Hardware_Map_Tests.slx` — target model with `Test_PWM`, `Test_ADC`, `Test_QEP`, `Test_SPI`, `Test_SCI`, and `Test_GPIO` subsystems.
- `F280049C_Hardware_Map_Test_Config.m` — single source of the Site-2 settings.
- `F280049C_Hardware_Map_Test_Select.m` — comments out all but the selected test and updates the model.
- `F280049C_Hardware_Map_Test_Validate.m` — static parameter audit; it does not flash or power hardware.
- `F280049C_Hardware_Map_Test_Readme.md` — bring-up order and expected measurements.

The verified model map is:

| Area | F280049C Site-2 setting |
|---|---|
| PWM | ePWM1/U = GPIO0/1, ePWM4/V = GPIO6/7, ePWM2/W = GPIO2/3; 20 kHz, TBPRD 2500, ePWM1 SOCA at CTR=PRD |
| ADC | ADCA SOC0 ADCINA6 = Vbus; ADCB SOC0 ADCINB6 = Ic; ADCC SOC0 ADCINC1 = Ia; ADCC SOC1 ADCINC14 = Ib; all triggered by ePWM1 SOCA |
| QEP | Native eQEP1 on LaunchPad J12, GPIO35/37/59; quadrature 2x; first-index reset; index latch falling edge |
| SPI | SPI-B on GPIO22/24/31 with GPIO27 chip select; AS5047P electrical test uses 16-bit MSB-first mode 1 at 1 MHz |
| SCI | SCI-A, one `uint16` RX/TX value; 0.0175 s receive timeout; the imported F280049C host example uses 1.5 Mbaud, 8-N-1, little-endian |
| Gate/OCP | GPIO34/nEN is driven high for disable; GPIO59/OCPn must be handled through ePWM TZ2/GPTRIP2, not the generic 49C Digital Input block |

The PWM map now exercises both complementary A/B outputs with 5 clock cycles of dead time. The map harness receives a four-word SCI command `[gate_enable, duty_U, duty_V, duty_W]`, keeps nEN high until `gate_enable=1`, and transmits a three-word 1 ms telemetry frame `[QEP_count, QEP_index, SPI_raw]`. `F280049C_Hardware_Map_Host.m` provides the direct MATLAB host commands. The SPI test repeats `0xFFFF`, the parity-correct AS5047P read command for address `0x3FFF`; the first response is a pipeline warm-up word, and the model still reports the raw 16-bit response rather than applying the 14-bit angle mask.
