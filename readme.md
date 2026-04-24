# wifi_sense

Passive human presence and motion detection using WiFi signal analysis. No cameras, no wearables, no special hardware beyond a monitor-mode WiFi adapter.

Listens to beacon, probe, and data frames from a target access point, tracks RSSI fluctuations over time, and uses spectral analysis to classify what's happening in the room: empty, someone standing still, or active movement.

Tested with a Netgear A8000 (MT7921AU) on Kali Linux.

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

- Linux with a WiFi adapter that supports **monitor mode**
- Python 3.8+
- Root/sudo (for raw packet capture and channel switching)

```
pip install scapy numpy scipy PyWavelets
```

PyWavelets is optional — the tool runs without it but skips the wavelet engine.

### Tested hardware

| Adapter | Chipset | Monitor mode | Notes |
|---------|---------|:---:|-------|
| Netgear A8000 | MT7921AU | Yes | 2x2 MIMO, WiFi 6E, works well |
| Any monitor-capable adapter | Various | Yes | Anything `iw` can put in monitor mode |

## Setup

Put your adapter into monitor mode:

```bash
sudo ip link set wlan0 down
sudo iw dev wlan0 set type monitor
sudo ip link set wlan0 up
```

## Usage

```bash
# auto-detect strongest AP, channel hop across 1/6/11
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

### Calibration

The first 8 seconds are calibration. Leave the room or stay completely still during this period so the tool learns what "empty" looks like. After calibration, baseline deviation scores tell you how different the current signal is from that reference.

## Architecture

```
WiFi adapter (monitor mode)
    │
    ├── All-frame capture (beacons + probes + data → ~7 Hz sample rate)
    │
    ├── Channel hopper (cycles ch 1/6/11 for multipath diversity)
    │
    ├── Adaptive baseline (8s calibration → deviation scoring)
    │
    ├── Spectral analysis
    │   ├── FFT (Hanning window)
    │   ├── Multi-taper PSD (5× DPSS tapers)
    │   └── Wavelet CWT (Morlet, 32 scales)
    │
    ├── Spectrogram (32-sample sliding window, 50% overlap)
    │
    ├── CUSUM change-point detection (3σ threshold)
    │
    ├── Cross-channel correlation (Pearson across ch 1/6/11)
    │
    └── Fused classifier → EMPTY / PRESENCE / STILL / MOTION
```

## Limitations

- **RSSI only** — consumer adapters don't expose per-subcarrier CSI data. You get one signal strength value per frame, not 52+ subcarrier amplitudes/phases. This limits resolution to presence/motion detection. Full body pose estimation (like DensePose from WiFi) requires CSI-capable hardware (ESP32-S3, Intel 5300, etc).
- **Heartbeat detection is unreliable** — the 0.8–2.0 Hz band is measured but RSSI doesn't have the sensitivity to reliably detect heartbeat at normal distances. Treat it as experimental.
- **Single antenna perspective** — even with 2x2 MIMO, the driver reports a single combined RSSI. No spatial diversity within a single adapter.
- **Neighbor interference** — dense apartment environments have lots of signal variation from other people's movement, not just yours.

## License

MIT
