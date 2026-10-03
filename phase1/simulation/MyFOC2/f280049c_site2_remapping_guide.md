# Hardware and Software Remapping Guide: LAUNCHXL-F28069M to LAUNCHXL-F280049C
**Target:** EPC9147B Interface Board + EPC91200 96V GaN Inverter + CubeMars AKE80-8 Motor  
**Mounting Location:** BoosterPack **Site 2** (Headers J5, J7, J8, J6)

---

## 1. Executive Summary & Architectural Differences

Migrating from the C2000 Piccolo **TMS320F28069M** to the **TMS320F280049C** brings several architectural improvements and necessary timing/peripheral updates:

1. **System Clock ($f_{SYS}$):**
   - **F28069M:** $90\text{ MHz}$
   - **F280049C:** **$100\text{ MHz}$**
2. **PWM Period Count ($TBPRD$) @ 20 kHz Center-Aligned:**
   - **F28069M:** $TBPRD = \frac{90\text{ MHz}}{2 \times 20\text{ kHz}} = 2250\text{ counts}$
   - **F280049C:** $TBPRD = \frac{100\text{ MHz}}{2 \times 20\text{ kHz}} = \mathbf{2500\text{ counts}}$
3. **ADC Subsystem:**
   - **F28069M:** Dual sample-and-hold sharing a single converter sequencer.
   - **F280049C:** **Three independent 12-bit ADCs (ADCA, ADCB, ADCC)** with Type 4 SOCs, allowing simultaneous 3-phase sampling with zero skew.
4. **Site 2 Master PWM:**
   - On Site 2 of the F280049C, **Phase U is assigned to `ePWM1`**, which acts as the native chip-level sync master.

---

## 2. BoosterPack Site 2 Hardware & Pin Remapping Table

### A. Gate PWM Drives (EPC9147B Digital Connector J2 $\rightarrow$ LaunchPad Header J8)
The EPC9147B J2 connector pinout starts at Pin 1 with **PWMH1** and maps down the header:

| Inverter Phase | EPC9147B Signal | EPC9147B J2 Pin | F28069M LaunchPad | **F280049C LaunchPad (Site 2)** | Simulink Peripheral Instance |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Phase U (High-Side)** | **PWMH1** | J2 Pin 1 | GPIO-06 (ePWM4A) | **GPIO-00 (PWM1A)** | **`ePWM1`** (Master) |
| **Phase U (Low-Side)** | **PWML1** | J2 Pin 3 | GPIO-07 (ePWM4B) | **GPIO-01 (PWM1B)** | **`ePWM1`** |
| **Phase V (High-Side)** | **PWMH2** | J2 Pin 5 | GPIO-08 (ePWM5A) | **GPIO-06 (PWM4A)** | **`ePWM4`** (Slave) |
| **Phase V (Low-Side)** | **PWML2** | J2 Pin 7 | GPIO-09 (ePWM5B) | **GPIO-07 (PWM4B)** | **`ePWM4`** |
| **Phase W (High-Side)** | **PWMH3** | J2 Pin 9 | GPIO-10 (ePWM6A) | **GPIO-02 (PWM2A)** | **`ePWM2`** (Slave) |
| **Phase W (Low-Side)** | **PWML3** | J2 Pin 11 | GPIO-11 (ePWM6B) | **GPIO-03 (PWM2B)** | **`ePWM2`** |

> [!TIP]
> **PWM Synchronization Architecture:**
> - Set **`ePWM1`** as the **Master Time Base**:
>   - Counter mode: Up-Down Count (`TBPRD = 2500`).
>   - Sync Output (`SYNCO`): Generated at `CTR = ZERO` (or `CTR = PRD`).
>   - SOCA Trigger: Generated at `CTR = PRD` (midpoint of low-side conduction).
> - Set **`ePWM4`** and **`ePWM2`** as **Slaves**:
>   - Counter mode: Up-Down Count (`TBPRD = 2500`).
>   - Phase Enable (`PHSEN`): Enabled (`1`), Phase Shift (`TBPHS`) = 0.
>   - Sync In: Driven from `EPWM1SYNCOUT`.

---

### B. Current & Voltage Feedback (EPC9147B Analog Connector J1 $\rightarrow$ LaunchPad Header J7)
On Site 2 of the F280049C, currents and bus voltage are routed across separate ADC converter blocks:

| Signal Function | EPC9147B Net | EPC9147B J1 Pin | F28069M Channel | **F280049C Pin & Channel** | Simulink ADC Block Configuration |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **DC Bus Voltage ($V_{dc}$)** | **Vdc** | J1 Pin 6 | ADC-B7 | **ADCINA6** (J7 Pin 63) | **ADC-A**, SOC0, Channel 6 |
| **Phase W Current ($I_c$)** | **Isns3** | J1 Pin 18 | ADC-A4 | **ADCINB6** (J7 Pin 64) | **ADC-B**, SOC0, Channel 6 |
| **Phase V Current ($I_b$)** | **Isns2** | J1 Pin 16 | ADC-B3 | **ADCINC14** (J7 Pin 65) | **ADC-C**, SOC1, Channel 14 |
| **Phase U Current ($I_a$)** | **Isns1** | J1 Pin 14 | ADC-A3 | **ADCINC1** (J7 Pin 66) | **ADC-C**, SOC0, Channel 1 |
| **Overcurrent Trip** | **OCPn** | J1 Pin 8 | GPIO-15 (TZ1) | **GPIO-59 (TZ2)** | TripZone Submodule Input |

> [!NOTE]
> All SOC triggers (`ADCA SOC0`, `ADCB SOC0`, `ADCC SOC0`, `ADCC SOC1`) should be set to trigger on **`ePWM1_SOCA`**.

---

### C. Position Sensor Interfaces (Header J8 & J6 / J5)

#### 1. Quadrature Incremental Encoder (native eQEP1 on the TI LaunchPad)
The actual encoder is connected to the F280049C LaunchPad J12 eQEP1 header, not remapped through the EPC9147B Site-2 BoosterPack headers:

| Encoder Signal | LaunchPad J12 | F28069M Pin | **F280049C GPIO** | Peripheral Module |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **EncA (Phase A)** | J12 EQEPA | GPIO-20 (EQEP1A) | **GPIO-35 (EQEP1A)** | **`eQEP1`** |
| **EncB (Phase B)** | J12 EQEPB | GPIO-21 (EQEP1B) | **GPIO-37 (EQEP1B)** | **`eQEP1`** |
| **EncI (Index)** | J12 EQEPI | GPIO-23 (EQEP1I) | **GPIO-59 (EQEP1I)** | **`eQEP1`** |

#### 2. Magnetic Absolute Encoder (AS5047P SPI)
Routed through Headers J5 and J6 on Site 2:

| SPI Signal | F28069M Pin | **F280049C Pin & Function (Site 2)** | Peripheral Module |
| :---: | :---: | :---: | :---: |
| **SPI CLK** | GPIO-26 (SPICLKB) | **GPIO-22 (SPIBCLK)** (J5 Pin 47) | **`SPI-B`** |
| **SPI MOSI (Data In to Encoder)** | GPIO-25 (SPISIMOB) | **GPIO-24 (SPIBSIMO)** (J6 Pin 75) | **`SPI-B`** |
| **SPI MISO (Data Out to MCU)** | GPIO-24 (SPISOMIB) | **GPIO-31 (SPIBSOMI)** (J6 Pin 74) | **`SPI-B`** |
| **SPI CSn (Chip Select)** | GPIO-19 / GPIO-22 | **GPIO-27 (SPIBSTE)** (J6 Pin 79) | **`SPI-B`** (or GPIO Out) |

---

### D. Gate Driver Control & Status Signals

| Signal | Function | EPC9147B Pin | F28069M Pin | **F280049C Pin (Site 2)** |
| :---: | :---: | :---: | :---: | :---: |
| **nEN** | Gate Driver Enable | J2 Pin 18 | GPIO-13 | **GPIO-34** (or GPIO-27) |
| **Status LED** | Firmware Heartbeat | On-board | GPIO-34 / 39 | **GPIO-23 (Blue) / GPIO-34 (Red)** |

---

## 3. MATLAB / Simulink Software Implementation Steps

### Step 1: Update Master Parameter Script (`FOC_Motor_Control_Params.m`)
Update the clocking and PWM periods to reflect the 100 MHz F280049C core:

```matlab
%% Target & Clock Configuration
target.ClockFreq_Hz = 100e6;        % 100 MHz for F280049C (was 90 MHz)
inverter.PWM_Freq   = 20e3;         % 20 kHz switching frequency

% Center-aligned Up-Down Counter:
% TBPRD = f_sys / (2 * f_pwm) = 100 MHz / 40 kHz = 2500 counts
inverter.PWM_Period = target.ClockFreq_Hz / (2 * inverter.PWM_Freq); % 2500 counts

% Update Sample Times
Ts = 1 / inverter.PWM_Freq;         % 50 microseconds (20 kHz)
```

### Step 2: Configure Model Hardware Settings
1. Open your Simulink Model (`FOC_Motor_Control_Hardware.slx` or `Hardware_Diagnostics_Test.slx`).
2. Press `Ctrl + E` to open **Model Configuration Parameters**.
3. Under **Hardware Implementation**:
   - Change **Hardware board** to: `TI Piccolo F28004x`.
4. Under **Target Hardware Resources**:
   - **Clocking**:
     - System clock frequency (MHz): `100.0`
     - OSC Clock frequency (MHz): `20.0` (standard on LAUNCHXL-F280049C)
   - **ePWM**:
     - Verify ePWM1, ePWM2, ePWM4 time-base period counts are set to `2500`.

### Step 3: Reassign Simulink Hardware Peripheral Blocks
1. **PWM Blocks:**
   - Change Phase U block to: **`ePWM1`** (Channel A/B)
   - Change Phase V block to: **`ePWM4`** (Channel A/B)
   - Change Phase W block to: **`ePWM2`** (Channel A/B)
2. **ADC Blocks:**
   - In place of the dual-sequencer block, insert/configure:
     - `ADC-A`: SOC0 $\rightarrow$ Channel 6 ($V_{dc}$)
     - `ADC-B`: SOC0 $\rightarrow$ Channel 6 ($I_c$)
     - `ADC-C`: SOC0 $\rightarrow$ Channel 1 ($I_a$), SOC1 $\rightarrow$ Channel 14 ($I_b$)
     - Trigger for all SOCs: **`ePWM1_SOCA`**
3. **Position Blocks:**
   - If using QEP: Use native **`eQEP1`** on GPIO35/GPIO37/GPIO59 and the LaunchPad J12 encoder header.
   - If using SPI: Change module from `SPI-B` on F28069 pins to **`SPI-B` on F280049 pins (GPIO 22/24/31/27)**.

---

## 4. Hardware Verification & LaunchPad Switch Settings

1. **Boot Mode Switch (`S2` on the F280049C LaunchPad):**
   - Set both positions **OFF/open** for Boot from Flash: GPIO32 = 1 and GPIO24 = 1.
   - Do not use the older `SW1 1-1` shorthand; the F280049C LaunchPad manual identifies this switch as `S2`.
2. **SCI-A to the USB virtual COM port:**
   - Leave the default GPIO28/GPIO29 route selected: **S6 = 0, S8 = 0**.
   - Install the TXD and RXD shunts on **J101**. Keep the TMS and TCK shunts installed for the on-board XDS110 debugger.
   - Use the Windows port named **XDS110 Class Application/User UART**, not an arbitrary COM number.
3. **Native eQEP1 on LaunchPad J12:**
   - Set the GPIO35/GPIO37/GPIO59 selector on **S3 = 0/down** to route eQEP1 to J12.
   - Set **S4 = 1/up** as required for GPIO35/GPIO37 eQEP1 routing.
   - Do not connect the EPC9147B Site-2 J60 encoder signals to the same eQEP1 test. GPIO59 is also the Site-2 OCPn route, so keep the inverter bus and gate test disabled while validating the encoder.
4. **Power jumpers for initial USB logic testing:**
   - Keep **JP1, JP2, JP3 installed** when USB powers the LaunchPad MCU and XDS110.
   - Keep **JP8 installed** only when the LaunchPad is the sole 3.3 V/5 V source for Site 2. Remove JP8 if the EPC9147B or another BoosterPack is also driving those rails, to prevent supply contention.
5. **EPC91200 protection and encoder supply:**
   - Keep **JOCPn installed** so the inverter over-current circuit can independently block PWM.
   - For a bare 3.3 V AS5047P interface, use the EPC91200 3.3 V encoder-supply option (R82), not the default 5 V option (R81). Follow the encoder interface board's rating if it includes its own level shifting.
6. **BoosterPack Placement:**
   - Seat the EPC9147B firmly into **Site 2 (Headers J5, J7, J8, J6)**.
   - Verify Pin 1 aligns with the top of the header as marked on the LaunchPad silkscreen.
7. **Power-On Sequence:**
   - Connect LaunchPad USB first (debugger power and logic reference).
   - Verify 3.3V / 5V rails with a DMM on header pins J7-61 and J5-41.
   - Apply low DC bus voltage (e.g. 12V–24V) to the EPC91200 inverter with a current-limited supply before full 96V testing.
