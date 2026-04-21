# Remote Multi-Channel Capacitance Control System
### End-to-End MATLAB Simulation — Blocks A to H

A complete MATLAB simulation of a 48-channel remote capacitance control system. The system transmits digitally encoded bias voltage commands over a BPSK wireless link, recovers them at the receiver, and synthesises the required DC bias voltages to drive 48 independent BB135 varactor diodes.

---

## System Overview

The signal chain is divided into eight functional blocks:

| Block | Function | Key Detail |
|-------|----------|------------|
| **A** | Transmitter Input Stage | 48 target capacitances + BB135 C-V inversion |
| **B** | ADC Encoding | 10-bit quantisation, frame assembly, CRC-16 |
| **C** | BPSK Modulation | 10 kHz carrier, 10 kbps symbol rate |
| **D** | AWGN Channel | Noise model at SNR = 5–25 dB |
| **E** | Demodulation | Coherent correlator + BER sweep |
| **F** | DAC Reconstruction | Voltage recovery from received ADC codes |
| **G** | PWM Bias Generation | 20 kHz PWM + RC low-pass filter |
| **H** | Varactor Verification | C-V model validation + end-to-end error analysis |

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Channels | 48 |
| Varactor Model | BB135 — C₀ = 18 pF, φ = 0.7 V, γ = 0.45 |
| ADC Resolution | 10-bit (dV = 9.7752 mV/LSB) |
| Frame Size | 512 bits (sync 16 + data 480 + CRC-16) |
| Modulation | BPSK (coherent) |
| Carrier Frequency | 10 kHz |
| Symbol Rate | 10 kbps |
| PWM Frequency | 20 kHz |
| RC Filter Cutoff | 1 kHz |
| Capacitance Range | 5.52 pF → 16.66 pF |
| Bias Voltage Range | 0.14 V → 8.99 V |

---

## Results Summary

| Metric | Result | Status |
|--------|--------|--------|
| BER @ 25 dB SNR (simulated) | 0 errors in 512 bits | ✅ PASS |
| CRC-16 | Passed | ✅ PASS |
| Max Voltage Reconstruction Error | 4.870 mV (< 0.5 LSB) | ✅ PASS |
| RMS Voltage Reconstruction Error | 2.529 mV | ✅ PASS |
| Max PWM Ripple | 2278.6 mV pp vs 50 mV spec | ❌ FAIL |
| Max Capacitance Error | 9.69 pF — 58.38% (Ch. 48) | ❌ FAIL |
| Mean Capacitance Error | 34.28% across all 48 channels | ❌ FAIL |

**The digital chain (Blocks A–E) performs correctly. The analog PWM bias stage (Block G) requires redesign.**

---

## Root Cause of Failure

The single-pole RC filter (f_c = 1 kHz) attenuates the 20 kHz PWM fundamental by only −26 dB, leaving up to **2.28 V of ripple** on the DC output. This voltage error is then amplified by the non-linear BB135 C-V characteristic, producing capacitance errors as large as 58% in low duty-cycle channels (<10%).

---

## Recommendations

1. **Lower the RC cutoff** to ≤ 100 Hz (e.g., C = 1.6 µF, R = 1 kΩ) to meet the ≤ 50 mV ripple specification
2. **Use a 2nd-order Sallen-Key active LPF** for −40 dB/decade roll-off
3. **Increase PWM frequency** to 200 kHz to reduce the required filter order
4. **Add closed-loop feedback** via a capacitance-to-digital converter (e.g., AD7747)
5. **Use a dedicated voltage DAC** (e.g., DAC8568) for channels with duty cycles < 5%
6. **Upgrade to 12-bit ADC** (dV ≈ 2.4 mV/LSB) for finer voltage control precision

---

## Files

| File | Description |
|------|-------------|
| `Yash_CDS_New_Code.m` | Main MATLAB simulation script (Blocks A–H) |
| `CDS_Project_Report.pdf` | Full technical report with figures and analysis |
| `CDS_Project_PPT.pdf` | Faculty presentation slides |
| `Yash_CDS_Output.pdf` | Simulation output plots |

---

## How to Run

1. Open `Yash_CDS_New_Code.m` in MATLAB (R2020a or later recommended)
2. Run the script — all 8 blocks execute sequentially
3. Figures for each block are generated automatically

No additional toolboxes are required beyond base MATLAB.

---

## License

This project was developed for academic purposes as part of a Communication & Digital Systems course (2025).
