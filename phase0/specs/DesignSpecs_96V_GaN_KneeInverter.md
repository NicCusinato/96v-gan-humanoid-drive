# 96V GaN Humanoid Knee Inverter - Design Specifications

**Phase 0 | Status: Draft | Date: 2026-05-12**
**Related KANs:** [KAN-9](https://cusinatoqueens.atlassian.net/browse/KAN-9) | [KAN-10](https://cusinatoqueens.atlassian.net/browse/KAN-10) | [KAN-11](https://cusinatoqueens.atlassian.net/browse/KAN-11) | [KAN-12](https://cusinatoqueens.atlassian.net/browse/KAN-12)
**Literature basis:** [Phase0LitReview.docx](../literature/Phase0LitReview.docx) in `phase0/literature/`

---

## 1. Electrical Requirements

| Parameter | Target Value | Rationale / Literature Basis |
|---|---|---|
| DC Bus Voltage | 96 V nominal (84-100 V operating range) | Higher bus reduces RMS current for same power; GaN rated for 100 V class (KAN-9, KAN-11) |
| Continuous Phase Current | 10 A RMS | From humanoid gait torque profiles (KAN-15, KAN-16); EPC91118 ref: ~7 A no heatsink, ~15 A with motor housing cooling |
| Peak Phase Current | 30 A (transient, <1 s) | Humanoid joint peak torque demands; consistent with TUM/Fraunhofer reference designs |
| Switching Frequency | 50-100 kHz (target 100 kHz) | TI TIDA-00909 validated at 100 kHz; EPC humanoid reference uses 100 kHz; reduces passive sizes (KAN-10) |
| Control Update Rate | >=20 kHz current loop | Required for high-bandwidth FOC in humanoid joint control |
| Topology | 3-phase half-bridge (6 switches) | Standard for PMSM/BLDC drives; consistent with all reference designs |
| Dead Time | <=100 ns | GaN zero-reverse-recovery allows shorter dead time vs Si; critical for efficiency at 100 kHz |

---

## 2. GaN Device Selection Targets

| Parameter | Target | Basis |
|---|---|---|
| Voltage Rating | 100 V class | 96 V bus + margin; EPC2302 / EPC2218 class devices |
| RDS(on) per switch | <10 mOhm | Minimise I^2*R conduction loss at 10 A RMS; primary selection criterion |
| Gate charge (Qg) | <10 nC | Enables fast switching at 100 kHz without excessive gate drive power |
| Package | LGA or chip-scale (no wire bonds) | Required for low parasitic inductance (Lloop <1 nH target); humanoid compactness |
| Coss | Minimise | Reduces switching losses at 96 V; key GaN advantage over Si |

---

## 3. Gate Driver Requirements

| Parameter | Target | Basis |
|---|---|---|
| Gate drive voltage | 5 V on / 0 V off (eGaN) | EPC enhancement-mode devices; no negative turn-off needed (KAN-12) |
| dV/dt immunity | >50 V/ns | 96 V bus, fast GaN switching; TI SNOAAB3 specifies this requirement |
| Propagation delay | <10 ns, matched high/low | Prevents shoot-through at short dead time; matched delay for current-loop symmetry |
| Isolation | Bootstrap or isolated (TBD) | Bootstrap acceptable for grounded-low side; isolated preferred for noise immunity at 96 V |
| Integration | Prefer GaN IC (driver + FET monolithic) | EPC GaN ICs reduce layout parasitics and simplify design |

---

## 4. Thermal Requirements

| Parameter | Target | Basis |
|---|---|---|
| Max junction temperature (Tj) | <125 C | Industry standard GaN derating limit |
| Ambient operating temperature | 25-60 C | Humanoid joint environment |
| Thermal resistance (junction-to-board) | <5 C/W per device | PCB-on-DBC and IMMD references achieve <3 C/W with optimised copper pour |
| Cooling method | Passive PCB copper + conduction to motor housing | EPC 32mm inverter: 7 A no heatsink, 15 A with motor housing coupling; no active fan |
| Power loss budget | <5 W total at 10 A RMS, 96 V, 100 kHz | Target >=97% efficiency at rated load |

---

## 5. PCB / Mechanical Requirements

| Parameter | Target | Basis |
|---|---|---|
| Board area | <=50 mm x 50 mm | Humanoid knee joint packaging constraint; EPC 32 mm reference is lower bound |
| Layer count | 4-6 layer PCB | Required for power/ground planes and current routing at 10-30 A |
| Power loop inductance | <5 nH | Critical for GaN at 100 kHz; ringing/overvoltage risk; TI/EPC layout guidelines (KAN-12) |
| DC-link capacitor placement | Adjacent to power devices, minimise loop area | PCB layout rule from TI SNOAAB3 and IEEE high-current GaN motor drive paper |
| Connector / bus interface | Bolt-down bus bar or soldered pads (no high-inductance connectors on power loop) | GaN layout best practice |

---

## 6. Efficiency and Performance Targets

| Parameter | Target | Basis |
|---|---|---|
| Peak inverter efficiency | >=97% | GaN advantage over 48V Si baseline (simulation: ~2-3% improvement); TUM humanoid drive paper |
| Full-load efficiency at 96V, 10A, 100kHz | >=96% | Working design target; to be validated in Phase 1 characterisation |
| Comparison baseline | 48V Si MOSFET (KAN-13 characterised) | Project hypothesis: 96V GaN > 48V Si in efficiency for same delivered power |

---

## 7. Open Items / TBD

- [ ] Final GaN FET / GaN IC device selection (EPC2302, EPC2218, EPC91118 -- to be narrowed in KAN-11 / KAN-12)
- [ ] Current sensing method (shunt inline vs. shunt low-side vs. Hall effect)
- [ ] DC-link capacitor type and value (film vs. ceramic; to be calculated from ripple spec)
- [ ] Gate drive topology: bootstrap vs. isolated; finalize after KAN-12 lit review
- [ ] Final PCB stack-up and copper weight
- [ ] Connector / harness spec for knee joint integration
- [ ] EMC / EMI targets (to be defined in Phase 1)

---

## 8. Key Literature Backing These Specs

| Spec Section | Primary Reference(s) |
|---|---|
| 96V bus / GaN device selection | EPC -- "GaN ICs for Humanoid Robot Joint Inverters" (2024); IEEE -- "Future Perspectives on GaN" |
| Switching frequency 100 kHz | TI TIDA-00909 48V GaN reference (100 kHz validated); EPC91118 humanoid reference |
| Gate driver dV/dt, dead time | TI SNOAAB3 -- "Application of GaN FET in Humanoid Robots"; Fraunhofer/Perera gate driver paper |
| Current targets | KAN-15/16 gait trajectory extraction; TUM humanoid drive thesis |
| Thermal / PCB layout | IEEE -- "Design of High Current GaN Motor Drive" (9955273); PCB-on-DBC GaN module paper |
| Efficiency target | 48V Si vs 96V GaN simulation (internal Phase0 baseline); Mohamed et al. IMMD paper |

---

*This is a living spec. Update as KAN-9, KAN-10, KAN-11, and KAN-12 are resolved.*
