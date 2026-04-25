# WiFi Sense: Passive Human Presence Detection Through WiFi Signal Analysis

**Sadik Erisen** | April 2026

---

## Abstract

WiFi Sense is an open-source system that detects human presence, motion, breathing patterns, and room occupancy using only WiFi signals already present in indoor environments. Unlike camera-based surveillance or wearable sensors, WiFi Sense requires no line of sight, no user cooperation, and no modification to existing WiFi infrastructure. The system operates in two modes: an RSSI-based mode using any standard WiFi adapter in monitor mode, and a CSI-based mode using an ESP32-S3 microcontroller that extracts per-subcarrier amplitude and phase data from WiFi frames. Both modes employ a multi-engine spectral analysis pipeline — FFT, multi-taper (DPSS), and continuous wavelet transform (CWT) — fused into a unified classification framework. The CSI mode additionally performs phase-domain vital sign detection, subcarrier correlation analysis, and motion direction estimation. Field testing in a residential apartment environment demonstrated reliable presence detection, motion classification at 100% confidence, breathing-band signal extraction, and room occupancy estimation, all from passive observation of ambient WiFi traffic.

---

## 1. Introduction

Every indoor environment is filled with WiFi signals. These signals propagate through walls, reflect off surfaces, and — critically — are disturbed by human bodies. When a person stands, walks, or breathes in a WiFi-covered space, their body absorbs and reflects the 2.4/5 GHz electromagnetic energy, creating measurable changes in the received signal.

This phenomenon has been studied extensively in academic research. Carnegie Mellon University's "DensePose From WiFi" work demonstrated that WiFi Channel State Information (CSI) can reconstruct full human body pose. However, most research systems require synchronized cameras for training data, controlled lab environments, and expensive research-grade hardware.

WiFi Sense takes a practical approach: build a working sensing system from commodity hardware, demonstrate real-world results in an uncontrolled residential environment, and release the complete implementation as open source.

### 1.1 Design Goals

- **No cameras** — operate entirely from radio signals
- **No cooperation** — detect people who are not wearing or carrying any device
- **No infrastructure changes** — use existing WiFi traffic passively
- **Commodity hardware** — standard WiFi adapters ($30) or ESP32-S3 microcontrollers ($8)
- **Real-time** — classify presence, motion, and vitals within seconds
- **Open source** — complete implementation published for reproducibility

### 1.2 Contributions

1. A dual-mode sensing system (RSSI and CSI) with a unified spectral analysis pipeline
2. A three-engine spectral fusion approach (FFT + multi-taper + wavelet) that compensates for each method's blind spots
3. Phase-domain vital sign detection from CSI data without requiring WiFi association
4. A self-contained ESP32-S3 firmware with on-device classification and LCD display
5. Field validation in an uncontrolled residential environment

---

## 2. Background

### 2.1 WiFi Signal Propagation

WiFi signals at 2.4 GHz (wavelength ~12.5 cm) and 5 GHz (wavelength ~6 cm) propagate through indoor environments via multiple paths: direct line-of-sight, reflections off walls and furniture, and diffraction around obstacles. The received signal is a superposition of all these paths.

When a human body (primarily water, which strongly absorbs microwave energy) enters or moves within this multipath environment, it creates two effects:

1. **Absorption**: The body absorbs some signal energy, reducing received power
2. **Reflection/scattering**: The body reflects energy in new directions, creating additional multipath components

These effects are measurable at two levels of granularity:

### 2.2 RSSI vs CSI

**RSSI (Received Signal Strength Indicator)** is a single scalar value per received frame — the total received power. It's available on every WiFi adapter. RSSI captures gross signal changes but lacks the resolution to distinguish between different types of disturbances.

**CSI (Channel State Information)** provides per-subcarrier amplitude and phase data. In an OFDM system like WiFi, the channel is divided into multiple frequency subcarriers (typically 52-256). Each subcarrier experiences the multipath environment differently, so CSI provides a rich, high-dimensional view of the channel. CSI is available on specific hardware: ESP32-S3 (via ESP-IDF), Intel 5300 NIC (via custom firmware), and Atheros NICs.

| Property | RSSI | CSI |
|----------|------|-----|
| Data per frame | 1 scalar | 64+ complex values |
| Information | Total power | Per-frequency amplitude + phase |
| Breathing detection | Marginal | Reliable (via phase) |
| Motion resolution | Coarse | Fine-grained |
| Hardware | Any WiFi adapter | ESP32-S3, Intel 5300, Atheros |

### 2.3 Frequency Bands of Interest

Human activities produce signal variations at characteristic frequencies:

| Activity | Frequency Range | Mechanism |
|----------|----------------|-----------|
| Breathing | 0.1 – 0.5 Hz (6-30 BPM) | Chest expansion/contraction modulates signal path |
| Heartbeat | 0.8 – 2.0 Hz (48-120 BPM) | Micro-vibrations from cardiac activity |
| Walking | 0.5 – 5.0 Hz | Periodic body displacement at step cadence |
| Entering/leaving | Transient | Abrupt signal level shift |

---

## 3. System Architecture

### 3.1 RSSI Mode

The RSSI pipeline operates with any WiFi adapter in monitor mode:

```
Monitor-mode adapter
    → All-frame capture (beacons + probes + data, ~7 Hz)
    → Channel hopping (ch 1/6/11, 500ms dwell)
    → RSSI time-series per BSSID
    → Adaptive baseline calibration (8s)
    → Spectral analysis (3 engines)
    → Cross-channel correlation
    → CUSUM change-point detection
    → Fused classifier
```

**All-frame capture**: Unlike tools that only capture beacon frames (~3 Hz), WiFi Sense captures management, probe response, and data frames from the target AP, increasing the sample rate to ~7 Hz. This is sufficient for detecting motion (up to 3.5 Hz Nyquist) but marginal for breathing detection.

**Channel hopping**: Cycling through channels 1, 6, and 11 provides three independent signal paths through the environment. Cross-channel correlation distinguishes real human movement (which affects all channels) from noise (which is channel-specific). Negative cross-channel correlation indicates multipath diversity — different frequencies see different reflections off the body.

### 3.2 CSI Mode

The CSI pipeline uses an ESP32-S3 in promiscuous mode:

```
ESP32-S3 (WIFI_MODE_NULL, promiscuous)
    → CSI callback (64 I/Q subcarrier pairs per frame)
    → Rate-limited to ~20 Hz
    → USB serial output (hex-encoded)
    → Python receiver on host
        → Per-subcarrier amplitude + phase extraction
        → Top-K subcarrier selection (8 most variant)
        → Amplitude-domain spectral analysis
        → Phase-domain vital sign extraction
        → Subcarrier correlation matrix
        → Fused classifier
```

**Promiscuous CSI capture**: The ESP32-S3 runs in `WIFI_MODE_NULL` — it does not associate with any network. It simply enables promiscuous mode and captures CSI from all WiFi frames in the air. This means the system works without any cooperation from the WiFi infrastructure.

**Top-K subcarrier selection**: Not all 64 subcarriers are equally informative. The system tracks per-subcarrier variance over time and selects the 8 most variable subcarriers for spectral analysis. These are the subcarriers whose signal paths are most affected by human movement.

**Phase-domain analysis**: Phase is more sensitive to small movements than amplitude. By computing the unwrapped phase difference between consecutive frames and applying spectral analysis to this derivative signal, the system can detect breathing patterns that are invisible in the amplitude domain.

### 3.3 On-Device Processing

The ESP32-S3 firmware performs classification locally with results displayed on a 1.69" LCD:

- Per-subcarrier variance tracking (exponential moving average)
- State classification: EMPTY / PRESENCE / STILL / MOTION
- Occupancy estimation (0-3+ people)
- Motion direction detection (LEFT/RIGHT/TOWARD/AWAY)
- IMU fusion (pauses sensing when device itself is moved)
- Multi-page touch UI with 5 views

---

## 4. Spectral Analysis Pipeline

### 4.1 Three-Engine Fusion

The system runs three independent spectral analysis engines on the same input signal and fuses their results:

**FFT (Fast Fourier Transform)**: Standard frequency decomposition with a Hanning window. Fast and effective for steady-state signals, but assumes stationarity (constant frequency content) and suffers from spectral leakage between frequency bins.

**Multi-Taper (DPSS/Slepian)**: Uses 5 orthogonal Discrete Prolate Spheroidal Sequence tapers. Each taper produces an independent spectral estimate; averaging them reduces variance and spectral leakage. The result: cleaner frequency peaks with better separation between breathing and motion bands.

**Continuous Wavelet Transform (CWT)**: Uses a Morlet wavelet at 32 scales. Unlike FFT, CWT provides time-frequency resolution — it can identify when frequency content changes (e.g., "breathing started at t=10, walking started at t=25"). This is critical for non-stationary signals like intermittent human movement.

### 4.2 Why Fusion?

Each engine has a blind spot:

| Engine | Strength | Blind Spot |
|--------|----------|------------|
| FFT | Fast, simple | Smears transient events, spectral leakage |
| Multi-taper | Clean peaks, low leakage | Still assumes stationarity |
| Wavelet | Catches transients | Computationally heavier, scale/frequency tradeoff |

The classifier combines evidence from all three: a transient event that FFT misses gets caught by the wavelet engine, and a steady breathing pattern that the wavelet under-resolves is precisely captured by multi-taper.

### 4.3 CUSUM Change-Point Detection

The Cumulative Sum (CUSUM) algorithm detects abrupt signal transitions — someone entering or leaving a room. It tracks a running sum of deviations from the mean:

```
s_positive += (signal[i] - mean) - drift
s_negative += -(signal[i] - mean) - drift

if s_positive > 3σ or s_negative > 3σ:
    change_point detected
    reset accumulators
```

This is more robust than threshold-based detection because it accumulates evidence over time, catching gradual transitions that a single threshold would miss.

### 4.4 Classification

The final state is determined by fusing scores from all engines:

```
motion_score = average(
    FFT_motion / 500,
    multi_taper_motion / 500,
    wavelet_motion / 5000,
    baseline_deviation / 5
)

breathing_score = average(
    FFT_breathing / 200,
    multi_taper_breathing / 200,
    wavelet_breathing / 2000
)

Decision:
    variance < 0.3 AND motion_score < 0.1   → EMPTY ROOM
    motion_score > 0.6 OR variance > 2.0     → MOTION DETECTED
    breathing_score > 0.3 AND motion < 0.4   → PERSON STILL
    else                                      → PRESENCE LIKELY
```

---

## 5. Hardware Implementation

### 5.1 RSSI Mode Hardware

- **Netgear A8000** (MT7921AU chipset): USB WiFi 6E adapter, 2x2 MIMO, monitor mode via mt7921u driver. Tested on Kali Linux. Cost: ~$30.
- Any WiFi adapter supporting monitor mode works. The `iw` utility puts the adapter in monitor mode: `sudo iw dev wlan0 set type monitor`.

### 5.2 CSI Mode Hardware

- **ESP32-S3 Touch LCD 1.69"** (Waveshare): Dual-core LX7 @ 240 MHz, 8MB flash, 1.69" IPS LCD (ST7789V2, 240x280), capacitive touch (CST816D), 6-axis IMU (QMI8658). Cost: ~$12.
- WiFi CSI is enabled via `CONFIG_ESP_WIFI_CSI_ENABLED=y` in ESP-IDF sdkconfig.
- No external antenna needed — the onboard PCB antenna provides sufficient sensitivity for room-level sensing.

### 5.3 Firmware Architecture

The firmware is split into 8 focused modules (1,700+ lines of C):

| Module | Responsibility | Lines |
|--------|---------------|-------|
| `main.c` | Task orchestration, WiFi/IMU init | 125 |
| `lcd.c` | ST7789V2 SPI driver, font, primitives | 188 |
| `csi_engine.c` | CSI processing, classification | 290 |
| `ui_pages.c` | 5-page display rendering | 600+ |
| `ap_tracker.c` | Multi-AP tracking (top 5) | 117 |
| `channel_hop.c` | Channel cycling timer | 76 |
| `touch.c` | CST816D gesture detection | 160 |
| `imu.c` | QMI8658 accel/gyro driver | 74 |

---

## 6. Field Results

### 6.1 Test Environment

Testing was conducted in a residential apartment in Philadelphia, PA. The environment included:
- 40+ WiFi access points visible (dense urban apartment building)
- Multiple walls between the sensing device and neighboring APs
- Normal household activity (people walking, sitting, cooking)
- No controlled conditions — all results are from live, uncontrolled sensing

### 6.2 RSSI Mode Results

Using a Netgear A8000 in monitor mode, capturing from the strongest AP (TomGlitter, -43 dBm):

- **Packet capture rate**: 2,978 packets across 43 BSSIDs in 30 seconds
- **Sample rate**: ~7 Hz (beacons + data frames)
- **Motion detection**: 100% confidence when occupant walking in the room
- **Change-point detection**: 20 transitions detected in 30 seconds during active movement
- **RSSI variance**: 0.91 dBm std (still) to 2.02 dBm std (active movement)
- **Cross-channel correlation**: -0.17 (divergent — confirming multipath diversity)

### 6.3 CSI Mode Results

Using ESP32-S3 in promiscuous mode, no WiFi association:

- **Subcarrier count**: 64 I/Q pairs per frame
- **Frame rate**: 11-20 Hz (rate-limited from 100+ Hz raw callback rate)
- **SNR**: 35 dB on close APs, 8-10 dB on distant ones
- **Subcarrier variance**: mean 15.65, max 31.88 during motion — hottest subcarriers sc5, sc6, sc26 identified
- **Phase-domain breathing power**: 3,278 (0.1-0.5 Hz band)
- **Phase-domain heartbeat power**: 10,634 (0.8-2.0 Hz band)
- **Motion classification**: 100% confidence during active movement
- **Occupancy estimation**: correctly differentiated 0, 1, and 2+ occupants

### 6.4 Comparison

| Metric | RSSI Mode | CSI Mode |
|--------|-----------|----------|
| Data points per second | ~7 | ~1,280 (64 SC × 20 Hz) |
| Motion detection | Reliable | Reliable |
| Breathing detection | Marginal | Detected (phase domain) |
| Occupancy counting | Not possible | 0-3+ estimation |
| Direction detection | Not possible | LEFT/RIGHT/TOWARD/AWAY |
| Hardware cost | ~$30 | ~$12 |

---

## 7. Limitations and Future Work

### 7.1 Current Limitations

- **No pose estimation**: Body pose reconstruction requires multiple synchronized CSI nodes and ML models. WiFi Sense performs presence/motion/vitals classification only.
- **Single node**: One ESP32-S3 provides temporal signal variation but no spatial triangulation. Room-level localization would require 3+ nodes.
- **Heartbeat detection**: Phase-domain heartbeat power is measurable but noisy. Reliable heartbeat extraction requires the subject to be within ~2 meters and mostly still.
- **Neighbor interference**: In dense apartment environments, neighboring residents' movement creates signal variations indistinguishable from local activity without BSSID filtering.
- **Calibration dependency**: The baseline calibration assumes an "empty room" state during the first 5-8 seconds. If someone is present during calibration, the baseline is biased.

### 7.2 Future Directions

- **Multi-node mesh**: Deploy 3-6 ESP32-S3 nodes for spatial triangulation and DensePose-style body reconstruction
- **ML classification**: Replace rule-based classifier with a trained neural network for higher accuracy
- **Through-wall sensing**: Optimize for scenarios where the sensing device is in a different room than the target
- **Activity recognition**: Classify specific activities (sitting, standing, cooking, sleeping) from CSI patterns
- **Integration with smart home**: MQTT/Home Assistant integration for automated presence-based control

---

## 8. Related Work

- **DensePose From WiFi** (Geng et al., CMU 2022): Demonstrated WiFi CSI to full body UV map reconstruction using synchronized camera-WiFi training. WiFi Sense uses similar CSI principles but targets detection/classification rather than pose reconstruction.
- **RuView** (ruvnet): Open-source edge AI perception system using ESP32 CSI for pose estimation, breathing, and heartbeat detection. WiFi Sense was inspired by RuView's architecture and adapted its sensing pipeline concepts for both RSSI and CSI modes.
- **Wi-Fi Sensing with ESP32** (Espressif): ESP-IDF provides native CSI extraction via `esp_wifi_set_csi_rx_cb()`. WiFi Sense uses this API in promiscuous mode without requiring WiFi association.
- **SpotFi** (Kotaru et al., SIGCOMM 2015): AoA estimation from CSI for indoor localization. WiFi Sense's subcarrier group analysis for direction detection is conceptually related.

---

## 9. Conclusion

WiFi Sense demonstrates that practical human presence sensing is achievable with commodity hardware and passive WiFi observation. The dual-mode architecture (RSSI for broad compatibility, CSI for precision) makes the system deployable in any WiFi-covered environment without infrastructure modifications.

The three-engine spectral fusion approach (FFT + multi-taper + wavelet) provides robust classification by compensating for each method's limitations. Phase-domain analysis of CSI data enables vital sign detection — breathing and heartbeat — that amplitude analysis alone cannot achieve.

The complete system — firmware, Python analysis tools, and documentation — is released as open source at [github.com/francose/wifi_sense](https://github.com/francose/wifi_sense).

---

## References

1. Geng, J., et al. "DensePose From WiFi." Carnegie Mellon University, 2022.
2. RuView. "WiFi DensePose — See through walls with WiFi + AI." github.com/ruvnet/RuView.
3. Espressif Systems. "ESP-IDF Wi-Fi Channel State Information." docs.espressif.com.
4. Kotaru, M., et al. "SpotFi: Decimeter Level Localization Using WiFi." ACM SIGCOMM, 2015.
5. Thomson, D.J. "Spectrum estimation and harmonic analysis." Proceedings of the IEEE, 1982. (Multi-taper method)
6. Mallat, S. "A Wavelet Tour of Signal Processing." Academic Press, 1999.

---

*WiFi Sense is open source under the MIT License. Source code and firmware available at [github.com/francose/wifi_sense](https://github.com/francose/wifi_sense).*
