#!/usr/bin/env python3
"""
WiFi Sense  — Advanced Spectral Analysis
author : Jynx (sadik erisen)

Improvements over v2:
  1. Multi-taper (DPSS/Slepian) spectral estimation — lower leakage
  2. Sliding-window spectrogram — see frequency content change over time
  3. Adaptive baseline calibration — auto-learns "empty room" signature
  4. Wavelet decomposition (CWT) — time-frequency for non-stationary signals
  5. Channel hopping — correlate RSSI across ch 1/6/11 for multi-path sensing
  6. All-frame capture — beacons + probes + data frames for higher sample rate

Usage:
  sudo python3 wifi_sense.py --iface wlan0
  sudo python3 wifi_sense.py --iface wlan0 --bssid AA:BB:CC:DD:EE:FF
  sudo python3 wifi_sense.py --iface wlan0 --no-hop   # stay on one channel
"""

import argparse, signal, sys, time, threading, subprocess, warnings
warnings.filterwarnings("ignore")

from collections import deque, defaultdict
from datetime import datetime
from enum import Enum
import numpy as np
from scipy import fft as scipy_fft
from scipy import signal as scipy_signal

try:
    import pywt
    HAS_WAVELET = True
except ImportError:
    HAS_WAVELET = False

import logging
logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
from scapy.all import sniff, Dot11, Dot11Beacon, Dot11ProbeResp, RadioTap, Dot11QoS


# ══════════════════════════════════════════════════════════════════════
# Multi-taper spectral estimation (Slepian/DPSS)
# ══════════════════════════════════════════════════════════════════════

class MultitaperPSD:
    """
    Computes PSD using Discrete Prolate Spheroidal Sequences (DPSS).
    Lower spectral leakage than single-window FFT, better freq resolution.
    """
    def __init__(self, nw=3.5, k_tapers=5):
        self.nw = nw          # time-bandwidth product
        self.k_tapers = k_tapers

    def estimate(self, signal_data, sample_rate):
        n = len(signal_data)
        if n < 16:
            return np.array([]), np.array([])

        # Generate DPSS tapers
        tapers = scipy_signal.windows.dpss(n, self.nw, self.k_tapers)
        freqs = scipy_fft.rfftfreq(n, d=1.0 / sample_rate)

        # Compute FFT for each taper and average
        psd_sum = np.zeros(len(freqs))
        for taper in tapers:
            windowed = signal_data * taper
            fft_vals = scipy_fft.rfft(windowed)
            psd_sum += np.abs(fft_vals) ** 2

        psd = psd_sum / self.k_tapers
        return freqs, psd


# ══════════════════════════════════════════════════════════════════════
# Sliding-window spectrogram
# ══════════════════════════════════════════════════════════════════════

class Spectrogram:
    """
    Short-Time FFT spectrogram — shows how frequency content evolves.
    Each column is a windowed FFT snapshot.
    """
    def __init__(self, window_samples=32, overlap_frac=0.5):
        self.window_samples = window_samples
        self.overlap_frac = overlap_frac

    def compute(self, signal_data, sample_rate):
        n = len(signal_data)
        win = self.window_samples
        step = max(1, int(win * (1 - self.overlap_frac)))

        if n < win:
            return np.array([]), np.array([]), np.array([[]])

        times = []
        freq_bins = scipy_fft.rfftfreq(win, d=1.0 / sample_rate)
        columns = []

        hann = np.hanning(win)
        for start in range(0, n - win + 1, step):
            chunk = signal_data[start:start + win]
            windowed = (chunk - np.mean(chunk)) * hann
            fft_mag = np.abs(scipy_fft.rfft(windowed)) ** 2
            columns.append(fft_mag)
            times.append(start / sample_rate)

        spec = np.array(columns).T  # shape: (n_freqs, n_times)
        return np.array(times), freq_bins, spec


# ══════════════════════════════════════════════════════════════════════
# Adaptive baseline calibration
# ══════════════════════════════════════════════════════════════════════

class AdaptiveBaseline:
    """
    Learns the 'empty room' RSSI signature during calibration phase.
    Subsequent readings are scored against the baseline.
    """
    def __init__(self, calibration_seconds=10):
        self.calibration_seconds = calibration_seconds
        self.baseline_mean = None
        self.baseline_std = None
        self.baseline_psd = None
        self.is_calibrated = False
        self._cal_samples = []
        self._cal_start = None

    def feed(self, rssi_val, timestamp):
        if self.is_calibrated:
            return
        if self._cal_start is None:
            self._cal_start = timestamp
        if timestamp - self._cal_start < self.calibration_seconds:
            self._cal_samples.append(rssi_val)
        else:
            self._finalize()

    def _finalize(self):
        if len(self._cal_samples) < 10:
            return
        arr = np.array(self._cal_samples)
        self.baseline_mean = float(np.mean(arr))
        self.baseline_std = float(np.std(arr))
        self.is_calibrated = True

    def deviation_score(self, current_mean, current_std):
        """How far current signal deviates from baseline (in sigma units)."""
        if not self.is_calibrated:
            return 0.0
        mean_dev = abs(current_mean - self.baseline_mean) / max(self.baseline_std, 0.01)
        std_dev = abs(current_std - self.baseline_std) / max(self.baseline_std, 0.01)
        return mean_dev + std_dev


# ══════════════════════════════════════════════════════════════════════
# Wavelet decomposition (CWT)
# ══════════════════════════════════════════════════════════════════════

class WaveletAnalyzer:
    """
    Continuous Wavelet Transform — better time-frequency resolution
    than FFT for non-stationary signals like human movement.
    """
    def __init__(self, wavelet="morl", scales_count=32):
        self.wavelet = wavelet
        self.scales_count = scales_count

    def analyze(self, signal_data, sample_rate):
        if not HAS_WAVELET or len(signal_data) < 16:
            return {"breathing_energy": 0, "motion_energy": 0,
                    "heartbeat_energy": 0, "time_freq_map": None,
                    "peak_scale_freq": 0}

        centered = signal_data - np.mean(signal_data)

        # Scales corresponding to frequencies of interest
        # scale = central_freq * sample_rate / desired_freq
        central_freq = pywt.central_frequency(self.wavelet)
        min_freq = 0.05  # Hz
        max_freq = min(5.0, sample_rate / 2.1)
        freqs = np.linspace(max_freq, min_freq, self.scales_count)
        scales = central_freq * sample_rate / freqs

        coefficients, frequencies = pywt.cwt(centered, scales, self.wavelet,
                                              sampling_period=1.0 / sample_rate)
        power = np.abs(coefficients) ** 2

        # Band energies from wavelet domain
        breathing_mask = (frequencies >= 0.1) & (frequencies <= 0.5)
        heartbeat_mask = (frequencies >= 0.8) & (frequencies <= 2.0)
        motion_mask = (frequencies >= 0.5) & (frequencies <= 5.0)

        breathing_energy = float(np.sum(power[breathing_mask, :])) if breathing_mask.any() else 0
        heartbeat_energy = float(np.sum(power[heartbeat_mask, :])) if heartbeat_mask.any() else 0
        motion_energy = float(np.sum(power[motion_mask, :])) if motion_mask.any() else 0

        # Peak frequency
        total_by_scale = np.sum(power, axis=1)
        peak_idx = np.argmax(total_by_scale)
        peak_freq = float(frequencies[peak_idx])

        return {
            "breathing_energy": breathing_energy,
            "motion_energy": motion_energy,
            "heartbeat_energy": heartbeat_energy,
            "time_freq_map": power,
            "peak_scale_freq": peak_freq,
            "frequencies": frequencies,
        }


# ══════════════════════════════════════════════════════════════════════
# Channel hopper
# ══════════════════════════════════════════════════════════════════════

class ChannelHopper:
    """Cycles through WiFi channels to capture multi-path signal diversity."""
    def __init__(self, iface, channels=(1, 6, 11), dwell_time=0.5):
        self.iface = iface
        self.channels = channels
        self.dwell_time = dwell_time
        self._running = False
        self.current_channel = channels[0]

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._hop_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _hop_loop(self):
        idx = 0
        while self._running:
            ch = self.channels[idx % len(self.channels)]
            try:
                subprocess.run(
                    ["iw", "dev", self.iface, "set", "channel", str(ch)],
                    capture_output=True, timeout=2
                )
                self.current_channel = ch
            except Exception:
                pass
            idx += 1
            time.sleep(self.dwell_time)


# ══════════════════════════════════════════════════════════════════════
# Enhanced sensor — all-frame capture + multi-channel
# ══════════════════════════════════════════════════════════════════════

class WifiSensorV3:
    def __init__(self, iface="wlan0", target_bssid=None, history_len=500):
        self.iface = iface
        self.target_bssid = target_bssid
        self.rssi_history = deque(maxlen=history_len)
        self.ts_history = deque(maxlen=history_len)
        self.channel_history = deque(maxlen=history_len)
        self.all_aps = {}
        self._running = False
        self._lock_phase = target_bssid is not None
        self._pkt_count = 0
        self._data_pkt_count = 0
        self._target_ssid = ""
        self._current_channel = 1

        # Analysis engines
        self.multitaper = MultitaperPSD(nw=3.5, k_tapers=5)
        self.spectrogram = Spectrogram(window_samples=32, overlap_frac=0.5)
        self.baseline = AdaptiveBaseline(calibration_seconds=8)
        self.wavelet = WaveletAnalyzer(wavelet="morl", scales_count=32)

        # Per-channel RSSI tracking for cross-channel correlation
        self.channel_rssi = defaultdict(lambda: deque(maxlen=200))

        # State tracking
        self._prev_state = "INITIALIZING"
        self._state_since = time.time()
        self._alerts = deque(maxlen=10)

    def _handle_pkt(self, pkt):
        if not pkt.haslayer(RadioTap):
            return
        try:
            rssi = float(pkt[RadioTap].dBm_AntSignal)
        except (AttributeError, TypeError):
            return

        if not pkt.haslayer(Dot11):
            return

        dot11 = pkt[Dot11]
        frame_type = dot11.type
        bssid = None

        # Extract BSSID based on frame type
        # Type 0 = mgmt (beacons, probes), Type 2 = data
        if frame_type == 0:  # Management
            bssid = dot11.addr3
        elif frame_type == 2:  # Data
            # To-DS: addr1=BSSID, From-DS: addr2=BSSID
            to_ds = dot11.FCfield & 0x1
            from_ds = dot11.FCfield & 0x2
            if to_ds and not from_ds:
                bssid = dot11.addr1
            elif not to_ds and from_ds:
                bssid = dot11.addr2
            elif not to_ds and not from_ds:
                bssid = dot11.addr3
            self._data_pkt_count += 1

        if not bssid:
            return
        self._pkt_count += 1

        # Grab SSID from beacons
        ssid = ""
        if pkt.haslayer(Dot11Beacon):
            try:
                ssid = pkt[Dot11Beacon].network_stats().get("ssid", "")
            except Exception:
                pass

        # Scan phase
        if not self._lock_phase:
            if bssid not in self.all_aps:
                self.all_aps[bssid] = {"ssid": ssid, "rssi": [], "count": 0}
            self.all_aps[bssid]["rssi"].append(rssi)
            self.all_aps[bssid]["count"] += 1
            if ssid:
                self.all_aps[bssid]["ssid"] = ssid
            return

        # Locked phase — track target across ALL frame types
        if bssid != self.target_bssid:
            return

        now = time.time()
        self.rssi_history.append(rssi)
        self.ts_history.append(now)
        self.channel_history.append(self._current_channel)
        self.channel_rssi[self._current_channel].append(rssi)
        self.baseline.feed(rssi, now)
        if ssid:
            self._target_ssid = ssid

    def scan_and_lock(self, scan_seconds=5):
        if self.target_bssid:
            self._target_ssid = self.target_bssid
            return
        print(f"  Scanning for {scan_seconds}s to find best AP...")
        self._lock_phase = False
        time.sleep(scan_seconds)
        if not self.all_aps:
            print("  No APs found!")
            sys.exit(1)
        best = None
        best_score = -999
        for bssid, data in self.all_aps.items():
            if len(data["rssi"]) < 3:
                continue
            med = float(np.median(data["rssi"]))
            if med > best_score:
                best_score = med
                best = bssid
        self.target_bssid = best
        self._target_ssid = self.all_aps[best].get("ssid", "") or "???"
        self._lock_phase = True
        print(f"  Locked: {best} ({self._target_ssid}) @ {best_score:.0f} dBm")

    def analyze(self):
        rssi = np.array(self.rssi_history)
        ts = np.array(self.ts_history)
        n = len(rssi)

        r = {
            "n_samples": n, "state": "COLLECTING", "confidence": 0.0,
            "rssi_now": rssi[-1] if n > 0 else 0,
            "rssi_mean": 0, "rssi_std": 0, "rssi_min": 0, "rssi_max": 0,
            "sample_rate": 0,
            # FFT
            "fft_breathing": 0, "fft_motion": 0, "fft_heartbeat": 0,
            "fft_dominant_freq": 0,
            # Multi-taper
            "mt_breathing": 0, "mt_motion": 0, "mt_heartbeat": 0,
            "mt_dominant_freq": 0,
            # Wavelet
            "wt_breathing": 0, "wt_motion": 0, "wt_heartbeat": 0,
            "wt_peak_freq": 0,
            # Spectrogram summary
            "spec_trend": "",  # frequency trend over time
            # Baseline
            "baseline_calibrated": self.baseline.is_calibrated,
            "baseline_deviation": 0,
            # Channel correlation
            "channel_correlation": 0,
            "channels_active": 0,
            # Change-points
            "change_points": 0,
            # Display
            "sparkline": "",
            "spec_heatmap": "",
        }

        if n < 20:
            return r

        duration = ts[-1] - ts[0]
        if duration < 1:
            return r
        sr = n / duration
        r["sample_rate"] = sr
        centered = rssi - np.mean(rssi)

        # ── Basic stats ──
        r["rssi_mean"] = float(np.mean(rssi))
        r["rssi_std"] = float(np.std(rssi))
        r["rssi_min"] = float(np.min(rssi))
        r["rssi_max"] = float(np.max(rssi))

        # ── 1) Multi-taper PSD ──
        mt_freqs, mt_psd = self.multitaper.estimate(centered, sr)
        if len(mt_freqs) > 1:
            bm = (mt_freqs >= 0.1) & (mt_freqs <= 0.5)
            hm = (mt_freqs >= 0.8) & (mt_freqs <= 2.0)
            mm = (mt_freqs >= 0.5) & (mt_freqs <= 5.0)
            r["mt_breathing"] = float(np.sum(mt_psd[bm])) if bm.any() else 0
            r["mt_heartbeat"] = float(np.sum(mt_psd[hm])) if hm.any() else 0
            r["mt_motion"] = float(np.sum(mt_psd[mm])) if mm.any() else 0
            dom_idx = np.argmax(mt_psd[1:]) + 1
            r["mt_dominant_freq"] = float(mt_freqs[dom_idx])

        # ── Standard FFT for comparison ──
        hann = np.hanning(n)
        fft_freqs = scipy_fft.rfftfreq(n, d=1.0 / sr)
        fft_mag = np.abs(scipy_fft.rfft(centered * hann)) ** 2
        if len(fft_freqs) > 1:
            bm = (fft_freqs >= 0.1) & (fft_freqs <= 0.5)
            hm = (fft_freqs >= 0.8) & (fft_freqs <= 2.0)
            mm = (fft_freqs >= 0.5) & (fft_freqs <= 5.0)
            r["fft_breathing"] = float(np.sum(fft_mag[bm])) if bm.any() else 0
            r["fft_heartbeat"] = float(np.sum(fft_mag[hm])) if hm.any() else 0
            r["fft_motion"] = float(np.sum(fft_mag[mm])) if mm.any() else 0
            dom_idx = np.argmax(fft_mag[1:]) + 1
            r["fft_dominant_freq"] = float(fft_freqs[dom_idx])

        # ── 2) Spectrogram ──
        spec_times, spec_freqs, spec_data = self.spectrogram.compute(centered, sr)
        if spec_data.size > 1 and spec_data.shape[1] > 1:
            # Frequency trend: which band dominates each time slice
            trend_chars = []
            for col_idx in range(spec_data.shape[1]):
                col = spec_data[:, col_idx]
                # Sum power in bands
                bm = (spec_freqs >= 0.1) & (spec_freqs <= 0.5)
                mm = (spec_freqs >= 0.5) & (spec_freqs <= 5.0)
                bp = float(np.sum(col[bm])) if bm.any() else 0
                mp = float(np.sum(col[mm])) if mm.any() else 0
                total = bp + mp + 0.001
                if mp / total > 0.7:
                    trend_chars.append("M")
                elif bp / total > 0.5:
                    trend_chars.append("b")
                else:
                    trend_chars.append("·")
            r["spec_trend"] = "".join(trend_chars[-50:])

            # Build mini heatmap (3 rows: motion/heartbeat/breathing × time)
            heatmap_rows = []
            blocks = " ░▒▓█"
            for band_mask_range in [(0.5, 5.0), (0.8, 2.0), (0.1, 0.5)]:
                mask = (spec_freqs >= band_mask_range[0]) & (spec_freqs <= band_mask_range[1])
                if not mask.any():
                    heatmap_rows.append("?" * min(50, spec_data.shape[1]))
                    continue
                band_power = np.sum(spec_data[mask, :], axis=0)
                bp_max = band_power.max() if band_power.max() > 0 else 1
                row = ""
                for v in band_power[-50:]:
                    idx = min(4, int(v / bp_max * 4))
                    row += blocks[idx]
                heatmap_rows.append(row)
            r["spec_heatmap"] = heatmap_rows

        # ── 3) Baseline deviation ──
        if self.baseline.is_calibrated:
            r["baseline_deviation"] = self.baseline.deviation_score(
                r["rssi_mean"], r["rssi_std"])

        # ── 4) Wavelet (CWT) ──
        wt = self.wavelet.analyze(centered, sr)
        r["wt_breathing"] = wt["breathing_energy"]
        r["wt_motion"] = wt["motion_energy"]
        r["wt_heartbeat"] = wt["heartbeat_energy"]
        r["wt_peak_freq"] = wt["peak_scale_freq"]

        # ── 5) Cross-channel correlation ──
        active_channels = {ch for ch in self.channel_rssi if len(self.channel_rssi[ch]) >= 10}
        r["channels_active"] = len(active_channels)
        if len(active_channels) >= 2:
            ch_list = sorted(active_channels)
            correlations = []
            for i in range(len(ch_list)):
                for j in range(i + 1, len(ch_list)):
                    a = np.array(self.channel_rssi[ch_list[i]])
                    b = np.array(self.channel_rssi[ch_list[j]])
                    min_len = min(len(a), len(b))
                    if min_len >= 10:
                        corr = float(np.corrcoef(a[-min_len:], b[-min_len:])[0, 1])
                        if not np.isnan(corr):
                            correlations.append(corr)
            if correlations:
                r["channel_correlation"] = float(np.mean(correlations))

        # ── CUSUM change-points ──
        std = r["rssi_std"]
        if std > 0.01:
            threshold = 3.0 * std
            drift = 0.5 * std
            s_pos = s_neg = 0.0
            cps = 0
            for i in range(1, n):
                diff = rssi[i] - r["rssi_mean"]
                s_pos = max(0, s_pos + diff - drift)
                s_neg = max(0, s_neg - diff - drift)
                if s_pos > threshold or s_neg > threshold:
                    cps += 1
                    s_pos = s_neg = 0.0
            r["change_points"] = cps

        # ── Classification (fused from all engines) ──
        var = r["rssi_std"] ** 2

        # Combine evidence from all three spectral methods
        motion_score = (
            r["mt_motion"] / 500 +
            r["fft_motion"] / 500 +
            r["wt_motion"] / 5000 +
            r["baseline_deviation"] / 5
        ) / 4

        breathing_score = (
            r["mt_breathing"] / 200 +
            r["fft_breathing"] / 200 +
            r["wt_breathing"] / 2000
        ) / 3

        if var < 0.3 and motion_score < 0.1:
            state = "EMPTY ROOM"
            conf = min(1.0, 1.0 - motion_score)
        elif motion_score > 0.6 or var > 2.0 or r["change_points"] > 5:
            state = "MOTION DETECTED"
            conf = min(1.0, motion_score)
        elif breathing_score > 0.3 and motion_score < 0.4:
            state = "PERSON STILL (breathing?)"
            conf = min(1.0, breathing_score)
        elif motion_score > 0.15 or breathing_score > 0.15:
            state = "PRESENCE LIKELY"
            conf = min(1.0, max(motion_score, breathing_score))
        else:
            state = "EMPTY ROOM"
            conf = min(1.0, 1.0 - motion_score)

        r["state"] = state
        r["confidence"] = max(0.0, min(1.0, conf))

        # Alert on state change
        if state != self._prev_state and self._prev_state != "INITIALIZING":
            dur = time.time() - self._state_since
            self._alerts.appendleft(
                f"  {datetime.now().strftime('%H:%M:%S')} │ {self._prev_state} → {state} (after {dur:.0f}s)"
            )
            self._state_since = time.time()
        self._prev_state = state

        # Sparkline
        sparks = " ▁▂▃▄▅▆▇█"
        tail = rssi[-50:]
        mn, mx = tail.min(), tail.max()
        rng = mx - mn if mx > mn else 1
        r["sparkline"] = "".join(sparks[min(8, int((v - mn) / rng * 8))] for v in tail)

        return r

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._sniff_loop, daemon=True)
        self._thread.start()

    def _sniff_loop(self):
        sniff(iface=self.iface, prn=self._handle_pkt, store=False,
              stop_filter=lambda _: not self._running)

    def stop(self):
        self._running = False


# ══════════════════════════════════════════════════════════════════════
# Display
# ══════════════════════════════════════════════════════════════════════

def display(sensor, hopper, r):
    print("\033[2J\033[H")
    state_colors = {
        "EMPTY ROOM": "\033[1;32m",
        "MOTION DETECTED": "\033[1;31m",
        "PERSON STILL (breathing?)": "\033[1;33m",
        "PRESENCE LIKELY": "\033[1;36m",
        "COLLECTING": "\033[0;37m",
    }
    c = state_colors.get(r["state"], "\033[0m")
    R = "\033[0m"
    DIM = "\033[2m"
    W = 70

    print(f"{'═' * W}")
    print(f"  RuView WiFi Sensing v3 — Advanced Spectral Analysis")
    ch_str = f"ch{hopper.current_channel}" if hopper else "fixed"
    print(f"  Target: {sensor.target_bssid} ({sensor._target_ssid})")
    print(f"  {datetime.now().strftime('%H:%M:%S')}  │  Samples: {r['n_samples']}  │  "
          f"Pkts: {sensor._pkt_count} (data: {sensor._data_pkt_count})  │  {ch_str}  │  "
          f"SR: {r['sample_rate']:.1f} Hz")
    print(f"{'═' * W}")

    # State
    print(f"\n  {c}{'━' * 54}")
    print(f"  ┃  {r['state']:^50s}┃")
    print(f"  ┃  Confidence: {r['confidence']:>5.0%}{'':>37s}┃")
    print(f"  {'━' * 54}{R}\n")

    # Signal
    print(f"  {DIM}Signal{R}")
    print(f"  ├─ Now: {r['rssi_now']:.0f} dBm  │  Mean: {r['rssi_mean']:.1f}  │  "
          f"σ={r['rssi_std']:.2f}  │  Range: {r['rssi_min']:.0f} to {r['rssi_max']:.0f}")
    cal_str = f"deviation={r['baseline_deviation']:.1f}σ" if r["baseline_calibrated"] else "calibrating..."
    print(f"  └─ Baseline: {cal_str}")

    # Spectral comparison (3 engines side by side)
    def bar(val, mx=500, width=15):
        filled = min(width, int(val / max(mx, 1) * width))
        return "█" * filled + "░" * (width - filled)

    print(f"\n  {DIM}Spectral Analysis — 3 engines{R}")
    print(f"  {'':>22s} {'FFT':>15s}  {'Multi-taper':>15s}  {'Wavelet':>15s}")
    print(f"  Breathing (0.1-0.5Hz) {bar(r['fft_breathing'])} "
          f"{bar(r['mt_breathing'])} {bar(r['wt_breathing'], 5000)}")
    print(f"  Heartbeat (0.8-2.0Hz) {bar(r['fft_heartbeat'])} "
          f"{bar(r['mt_heartbeat'])} {bar(r['wt_heartbeat'], 5000)}")
    print(f"  Motion    (0.5-5.0Hz) {bar(r['fft_motion'])} "
          f"{bar(r['mt_motion'])} {bar(r['wt_motion'], 5000)}")
    print(f"  Dom freq:             {r['fft_dominant_freq']:>7.3f} Hz      "
          f"{r['mt_dominant_freq']:>7.3f} Hz      {r['wt_peak_freq']:>7.3f} Hz")

    # Spectrogram heatmap
    if r.get("spec_heatmap") and len(r["spec_heatmap"]) == 3:
        print(f"\n  {DIM}Spectrogram (time →){R}")
        labels = ["  Motion  ", "  Heartbt ", "  Breathe "]
        for label, row in zip(labels, r["spec_heatmap"]):
            print(f"  {label} │{row}│")

    # Frequency trend
    if r["spec_trend"]:
        print(f"  {DIM}  Trend:    │{r['spec_trend']}│{R}")
        print(f"  {DIM}            (M=motion  b=breathing  ·=quiet){R}")

    # Cross-channel
    if r["channels_active"] >= 2:
        print(f"\n  {DIM}Channel Correlation{R}")
        print(f"  ├─ Active channels: {r['channels_active']}")
        print(f"  └─ Cross-corr: {r['channel_correlation']:+.3f}  "
              f"({'correlated' if r['channel_correlation'] > 0.5 else 'divergent — multi-path'})")

    # RSSI waveform
    if r["sparkline"]:
        print(f"\n  {DIM}RSSI Waveform (last 50 samples){R}")
        print(f"  {r['sparkline']}")

    # Change-points
    print(f"\n  Change-points: {r['change_points']}")

    # Alerts
    if sensor._alerts:
        print(f"\n  {DIM}State Changes{R}")
        for a in list(sensor._alerts)[:5]:
            print(a)

    print(f"\n{'─' * W}")
    wt_str = " + Wavelet(CWT)" if HAS_WAVELET else " (install pywt for wavelets)"
    print(f"  {DIM}FFT + Multi-taper(DPSS){wt_str}{R}")
    print(f"  {DIM}Move / stand still / leave room to see changes  │  Ctrl+C to stop{R}")


# ══════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="RuView WiFi Sensing v3")
    parser.add_argument("--iface", default="wlan0")
    parser.add_argument("--bssid", default=None)
    parser.add_argument("--duration", type=int, default=180)
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--no-hop", action="store_true", help="Disable channel hopping")
    parser.add_argument("--channels", default="1,6,11", help="Channels to hop (comma-sep)")
    parser.add_argument("--dwell", type=float, default=0.5, help="Dwell time per channel (sec)")
    args = parser.parse_args()

    channels = tuple(int(c) for c in args.channels.split(","))

    print(f"[*] RuView WiFi Sensing v3")
    print(f"    Engines: FFT + Multi-taper(DPSS) + {'Wavelet(CWT)' if HAS_WAVELET else 'NO wavelet (pip install PyWavelets)'}")
    print(f"    Channel hop: {'OFF' if args.no_hop else f'channels {channels}, dwell {args.dwell}s'}")
    print(f"    All-frame capture: beacons + probes + data")

    sensor = WifiSensorV3(iface=args.iface, target_bssid=args.bssid)
    hopper = None
    if not args.no_hop:
        hopper = ChannelHopper(args.iface, channels=channels, dwell_time=args.dwell)

    def shutdown(sig, frame):
        sensor.stop()
        if hopper:
            hopper.stop()
        print("\n[*] Stopped.")
        sys.exit(0)
    signal.signal(signal.SIGINT, shutdown)

    sensor.start()
    if hopper:
        hopper.start()
    sensor.scan_and_lock(scan_seconds=5)

    end = time.time() + args.duration
    while time.time() < end and sensor._running:
        if hopper:
            sensor._current_channel = hopper.current_channel
        r = sensor.analyze()
        display(sensor, hopper, r)
        time.sleep(args.interval)

    sensor.stop()
    if hopper:
        hopper.stop()

if __name__ == "__main__":
    main()
