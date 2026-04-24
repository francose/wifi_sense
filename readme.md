# wifi_sense

Passive human presence and motion detection using WiFi signal analysis. No cameras, no wearables.

Two modes of operation:

- **RSSI mode** — works with any monitor-mode WiFi adapter (tested with Netgear A8000 / MT7921AU). Tracks signal strength fluctuations to detect presence and motion.
- **CSI mode** — uses an ESP32-S3 to extract per-subcarrier amplitude and phase data (64 subcarriers at ~20 Hz over USB serial). Enables reliable breathing detection, heartbeat sensing, and fine-grained motion classification that RSSI alone can't do.

### Tools

| Script | What it does | Hardware |
|--------|-------------|----------|
| `rssi_finder.py` | Scans all nearby APs, classifies presence/motion on each | Any monitor-mode adapter |
| `wifi_sense.py` | Locks onto one AP, deep spectral analysis (3 engines + spectrogram) | Any monitor-mode adapter |
| `csi_receiver.py` | Reads real CSI from ESP32-S3 over USB, full subcarrier analysis | ESP32-S3 with CSI firmware |

## How it works

When a person stands or moves between a WiFi access point and your adapter, their body absorbs and reflects the 2.4/5 GHz signal. Different activities produce different RSSI fluctuation patterns:

- **Walking** creates ~1 Hz oscillations matching step cadence
- **Breathing** produces subtle 0.1–0.5 Hz variations from chest movement
- **Entering/leaving** causes abrupt RSSI level shifts
- **Empty room** shows flat, low-variance signal

The tool captures these fluctuations and runs three independent spectral analysis engines in parallel:

| Engine | What it does | Why it helps |
|--------|-------------|--------------|
| **FFT** | Standard frequency decomposition with Hanning window | Fast, good baseline for steady-state signals |
| **Multi-taper (DPSS)** | Uses 5 orthogonal Slepian tapers, averages their spectra | Reduces spectral leakage, cleaner frequency peaks |
| **Wavelet (CWT)** | Morlet wavelet at 32 scales, gives time-frequency resolution | Catches transient events that FFT smears out |

Results from all three are fused into a single classification decision. A spectrogram (sliding-window STFT) shows how frequency content evolves over time, and CUSUM change-point detection flags abrupt signal transitions.

Channel hopping across 1/6/11 adds multi-path diversity — each channel sees different reflections off the human body, and cross-channel correlation confirms whether disturbances are real or noise.

## Requirements

### RSSI mode (rssi_finder.py, wifi_sense.py)

- Linux with a WiFi adapter that supports **monitor mode**
- Python 3.8+
- Root/sudo (for raw packet capture and channel switching)

```
pip install scapy numpy scipy PyWavelets
```

### CSI mode (csi_receiver.py)

- ESP32-S3 board flashed with CSI firmware (see below)
- USB connection to the ESP32-S3
- Python 3.8+

```
pip install pyserial numpy scipy PyWavelets
```

PyWavelets is optional in both modes — skips the wavelet engine if not installed.

### Tested hardware

| Hardware | Type | Notes |
|----------|------|-------|
| Netgear A8000 | USB WiFi adapter (MT7921AU) | RSSI mode, 2x2 MIMO, WiFi 6E |
| ESP32-S3 Touch LCD 1.69" | Microcontroller | CSI mode, 64 subcarriers, USB serial |
| Any monitor-capable adapter | USB WiFi | RSSI mode, anything `iw` can put in monitor mode |

## Setup

### RSSI mode

Put your adapter into monitor mode:

```bash
sudo ip link set wlan0 down
sudo iw dev wlan0 set type monitor
sudo ip link set wlan0 up
```

### CSI mode — ESP32-S3 firmware

The ESP32-S3 needs firmware that captures CSI in promiscuous mode and streams it over USB serial. A minimal firmware is included in `firmware/csi_serial/`.

Requirements: [ESP-IDF v5.2+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)

```bash
# install ESP-IDF (one-time)
mkdir -p ~/esp && cd ~/esp
git clone --depth 1 -b v5.2.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3

# build and flash
source ~/esp/esp-idf/export.sh
cd firmware/csi_serial
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

The firmware runs in `WIFI_MODE_NULL` with promiscuous mode — it doesn't connect to any network. It captures CSI from all WiFi frames in the air and prints them over USB serial at ~20 Hz.

Serial format: `CSI,<rssi>,<channel>,<noise_floor>,<iq_len>,<hex_iq_data>\n`

## Usage

### RSSI tools

```bash
# scan all APs in the area
sudo python3 rssi_finder.py --iface wlan0

# deep analysis on one AP — auto-detect strongest, channel hop across 1/6/11
sudo python3 wifi_sense.py --iface wlan0

# lock onto a specific AP
sudo python3 wifi_sense.py --iface wlan0 --bssid AA:BB:CC:DD:EE:FF

# stay on one channel (no hopping)
sudo python3 wifi_sense.py --iface wlan0 --no-hop

# custom channels and dwell time
sudo python3 wifi_sense.py --iface wlan0 --channels 1,6,11,36 --dwell 0.3

# run for 5 minutes with 1-second refresh
sudo python3 wifi_sense.py --iface wlan0 --duration 300 --interval 1
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--iface` | `wlan0` | Monitor-mode interface |
| `--bssid` | auto | Target AP (auto-picks strongest if omitted) |
| `--duration` | `180` | How long to run in seconds |
| `--interval` | `2.0` | Display refresh interval in seconds |
| `--no-hop` | off | Disable channel hopping |
| `--channels` | `1,6,11` | Channels to cycle through |
| `--dwell` | `0.5` | Seconds to stay on each channel |

### What the output means

The display shows real-time analysis with:

- **State** — `EMPTY ROOM`, `PRESENCE LIKELY`, `PERSON STILL (breathing?)`, or `MOTION DETECTED` with confidence percentage
- **Signal stats** — current RSSI, mean, standard deviation, range
- **Baseline deviation** — how far current readings are from the calibrated "empty room" baseline (first 8 seconds)
- **Spectral bars** — power in breathing (0.1–0.5 Hz), heartbeat (0.8–2.0 Hz), and motion (0.5–5.0 Hz) bands across all three engines
- **Spectrogram heatmap** — frequency band power over time using block characters
- **RSSI waveform** — sparkline of the last 50 samples
- **Channel correlation** — cross-correlation between channels (negative = good multipath diversity)
- **Change-points** — CUSUM-detected abrupt signal transitions

### CSI receiver (ESP32-S3)

```bash
# read CSI from ESP32-S3 over USB
sudo python3 csi_receiver.py --port /dev/ttyACM0

# custom duration and refresh
sudo python3 csi_receiver.py --port /dev/ttyACM0 --duration 300 --interval 1
```

The CSI receiver shows everything the RSSI tools show, plus:

- **64-subcarrier variance heatmap** — which subcarriers are most disturbed by movement
- **Phase-domain vitals** — breathing and heartbeat detection from phase differences (far more sensitive than amplitude)
- **Subcarrier cross-correlation** — high correlation = real human movement, low = noise
- **Top-K subcarrier tracking** — focuses spectral analysis on the 8 most variable subcarriers

### Calibration

The first 5-8 seconds are calibration. Leave the room or stay completely still during this period so the tool learns what "empty" looks like. After calibration, baseline deviation scores tell you how different the current signal is from that reference.

## Architecture

### RSSI mode

```
WiFi adapter (monitor mode)
    │
    ├── All-frame capture (beacons + probes + data → ~7 Hz)
    ├── Channel hopper (ch 1/6/11)
    ├── Adaptive baseline (8s calibration)
    ├── Spectral: FFT + Multi-taper + Wavelet
    ├── Spectrogram + CUSUM change-points
    ├── Cross-channel correlation
    └── Fused classifier → EMPTY / PRESENCE / STILL / MOTION
```

### CSI mode

```
ESP32-S3 (promiscuous mode, no WiFi association)
    │
    ├── CSI callback → 64 I/Q subcarrier pairs per frame
    ├── Rate-limited to ~20 Hz
    ├── Hex-encoded serial output over USB
    │
    ▼
Python receiver (csi_receiver.py)
    │
    ├── Per-subcarrier amplitude + phase extraction
    ├── Top-K variance tracking (8 most active subcarriers)
    ├── Amplitude-domain: FFT + Multi-taper + Wavelet
    ├── Phase-domain: unwrapped phase diff → breathing/heartbeat
    ├── Subcarrier cross-correlation matrix
    ├── Spectrogram + CUSUM change-points
    ├── Adaptive baseline (5s calibration)
    └── Fused classifier (amplitude + phase + correlation)
```

### RSSI vs CSI comparison

| | RSSI | CSI |
|---|---|---|
| Data per frame | 1 value (signal strength) | 64 complex I/Q pairs (amplitude + phase) |
| Sample rate | ~7 Hz | ~20 Hz |
| Breathing detection | Marginal | Reliable (phase domain) |
| Heartbeat detection | Unreliable | Detectable at close range |
| Spatial resolution | None | Per-subcarrier frequency selectivity |
| Hardware | Any monitor-mode adapter | ESP32-S3 |

## Limitations

- **No pose estimation** — body pose reconstruction (DensePose from WiFi) requires multiple synchronized CSI nodes and ML models. This project does presence/motion/vitals only.
- **Single ESP32** — with one node you get temporal signal variation but no spatial triangulation. Multiple nodes would enable room-level localization.
- **Heartbeat detection** — works best within ~2m with minimal ambient motion. Phase domain helps but it's still noisy compared to contact sensors.
- **Neighbor interference** — dense apartment environments have signal variation from other people's movement, not just yours.
- **ESP32 LCD unused** — the CSI firmware doesn't drive the display. Would need board-specific pin configuration for LCD output.

## Acknowledgments

Inspired by [RuView](https://github.com/ruvnet/RuView), an edge AI perception system that uses WiFi CSI for full body pose estimation. This project adapts RuView's sensing pipeline concepts for both consumer WiFi adapters (RSSI) and ESP32-S3 CSI hardware.

## License

MIT
