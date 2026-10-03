# F280049C Site-2 hardware-map bring-up

`F280049C_Hardware_Map_Tests.slx` is a deliberately small peripheral test model for the LAUNCHXL-F280049C on Site 2 of the EPC9147B adapter. It is not an FOC controller and it does not perform encoder offset calibration.

The model contains separate `Test_PWM`, `Test_ADC`, `Test_QEP`, `Test_SPI`, `Test_SCI`, and `Test_GPIO` subsystems. Use the selector to leave only one map active:

```matlab
F280049C_Hardware_Map_Test_Select('GPIO')
F280049C_Hardware_Map_Test_Select('PWM')
F280049C_Hardware_Map_Test_Select('ADC')
F280049C_Hardware_Map_Test_Select('QEP')
F280049C_Hardware_Map_Test_Select('SPI')
F280049C_Hardware_Map_Test_Select('QEP_SPI')
F280049C_Hardware_Map_Test_Select('SCI')
F280049C_Hardware_Map_Test_Validate()
```

`GPIO` is kept active for every mode. It drives GPIO34/nEN high, which is the safe disabled state for the active-low gate-enable line. The selector also keeps PWM active during the ADC test because the ADC SOCs are triggered by ePWM1 SOCA.

Use `QEP_SPI` for hand-spinning: it keeps native eQEP1 and SPI-B active together while leaving PWM and ADC inactive.

The SCI command frame is four little-endian `uint16` words:

```text
[gate_enable, duty_U_percent, duty_V_percent, duty_W_percent]
```

`gate_enable=0` holds nEN high and disables the gate driver. `gate_enable=1` drives nEN low and requests enable. Duty values are 0–100. The target starts with all four received words at zero, so it powers up disabled.

The target telemetry frame is three little-endian `uint16` words sent every 1 ms:

```text
[QEP_position_count, QEP_index_latch, SPI_raw_word]
```

For a direct MATLAB host connection, use [F280049C_Hardware_Map_Host.m](<C:/96v_gan_humanoid_drive/phase1/simulation/MyFOC%232/F280049C_Hardware_Map_Host.m>):

```matlab
F280049C_Hardware_Map_Host('COM7', 'command', 0, 25, 50, 75) % PWM present, gate disabled
t = F280049C_Hardware_Map_Host('COM7', 'read')                  % read QEP/SPI once
F280049C_Hardware_Map_Host('COM7', 'watch', 10)                 % monitor hand rotation
F280049C_Hardware_Map_Host('COM7', 'command', 1, 25, 50, 75) % request gate enable
F280049C_Hardware_Map_Host('COM7', 'disable')                  % immediately disable again
F280049C_Hardware_Map_Host('COM7', 'close')
```

## Target settings

| Peripheral | Setting in the test model |
|---|---|
| Board | `TI Piccolo F280049C LaunchPad` |
| System clock | 100 MHz |
| Fixed step | 50 us |
| PWM | 20 kHz, up-down, TBPRD 2500 |
| PWM modules | ePWM1 = U, ePWM4 = V, ePWM2 = W |
| PWM pins | GPIO0/1, GPIO6/7, GPIO2/3 |
| PWM test duty | 25%, 50%, 75% for U/V/W |
| PWM dead time | 5 clock cycles for the first electrical test |
| ADC trigger | ePWM1 SOCA at CTR=PRD |
| ADC channels | ADCA SOC0/ADCINA6, ADCB SOC0/ADCINB6, ADCC SOC0/ADCINC1, ADCC SOC1/ADCINC14 |
| QEP | Native eQEP1 on the LaunchPad J12 interface: GPIO35/37/59, quadrature 2x, first-index reset |
| SPI | SPI-B, GPIO22/24/31/27, 16-bit MSB-first mode 1 at 1 MHz; repeated `0xFFFF` read-angle command |
| SCI | SCI-A, four `uint16` command words, 0.0175 s timeout; host reference is 1.5 Mbaud, 8-N-1 little-endian |
| Gate enable | GPIO34, driven high by default |
| OCPn | GPIO59 through the ePWM TZ2/GPTRIP2 path; it is not a generic Digital Input block |

## LaunchPad and EPC jumper/switch setup

For the USB host test, use the LAUNCHXL-F280049C XDS110 virtual COM route. Set boot switch `S2` positions 1 and 2 **OFF/open** for Flash boot. Leave `S6=0` and `S8=0` so SCI-A uses the default GPIO28/GPIO29 connection to the XDS110 virtual COM port. Install the RXD and TXD shunts on `J101`; keep the TMS and TCK shunts installed for debugging. The Windows port name must be `XDS110 Class Application/User UART`.

For the native eQEP1 encoder route, connect EncA/EncB/EncI to the LaunchPad J12 encoder interface. Set the GPIO35/GPIO37/GPIO59 selector on `S3` to `0/down` for J12 and set `S4=1/up`. Do not use the EPC9147B Site-2 J60 encoder path at the same time. Keep `JP1`, `JP2`, and `JP3` installed when USB powers the LaunchPad. Use `JP8` only when there is no second 3.3 V/5 V source on Site 2. Keep EPC91200 `JOCPn` installed for independent over-current PWM blocking, but note that GPIO59 is then shared with the EPC9147B OCPn route.

The SPI subsystem sends `0xFFFF` repeatedly. For AS5047P, this is the parity-correct 16-bit read command for the `0x3FFF` angle register; the returned data is the response word from the preceding frame, so discard the first word after startup. The model reports the raw word and does not yet extract the 14-bit angle or convert it into FOC electrical angle.

## Safe bring-up order

1. With the EPC91200 bus unpowered and the motor disconnected, select `GPIO`, build, and verify GPIO34/nEN is high.
2. Select `PWM`, keep the gate driver disabled, and measure the three A/B pairs at the adapter or controller-side test points. Confirm 20 kHz, complementary polarity, phase mapping, and dead time.
3. Select `QEP` and turn the encoder by hand. Run the host `watch` command. Confirm QEP count changes in the expected direction and index latch occurs once per revolution.
4. Select `SPI` with the AS5047P powered from the correct 3.3 V interface. Run `watch`; verify CSn, SCK, MOSI, and MISO on a logic analyzer and confirm the raw word changes as the motor is hand-rotated.
5. Select `PWM`. Send the disabled command first and measure ePWM1/ePWM4/ePWM2 A/B pairs on the oscilloscope. Only with the motor disconnected and the power stage disabled, send the enable command and confirm nEN changes state.
6. Select `ADC`. Use only safe 0–3.3 V test signals at the ADC inputs; do not put the 96 V bus on the ADC pin. ADC outputs remain terminated in this first bench model.
7. Only after these pass should the FOC hardware model replace the map-test processing with current/position control.

The C2000Ware toolchain must be configured locally with `c2000setup` before `slbuild` or flashing. A Simulink diagram update, structural audit, code generation, and target load pass when the build wrapper below is used.

Use `F280049C_Hardware_Map_Build` to build and deploy after renaming or moving this project. It redirects Simulink cache/code-generation output to a temporary path so GNU Make cannot misinterpret special characters in the source-folder name.
