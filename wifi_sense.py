#!/usr/bin/env python3
"""
wifi_sense — passive human presence/motion detection via WiFi RSSI spectral analysis.

Captures beacon, probe, and data frames from a target AP in monitor mode,
tracks RSSI over time, and runs three parallel spectral engines (FFT,
multi-taper DPSS, wavelet CWT) to classify room state:
empty, still occupant, or active movement.

Requires: scapy, numpy, scipy
Optional: PyWavelets (for CWT engine)

Author: sadik erisen
"""

import argparse
import logging
import signal
import subprocess
import sys
import threading
import time
import warnings
from collections import defaultdict, deque
from datetime import datetime

warnings.filterwarnings("ignore")

import numpy as np
from scipy import fft as sp_fft
from scipy.signal.windows import dpss

try:
    import pywt
    _HAS_WAVELETS = True
except ImportError:
    _HAS_WAVELETS = False

logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
from scapy.all import (
    Dot11,
    Dot11Beacon,
    Dot11ProbeResp,
    RadioTap,
    sniff,
)

# frequency bands of interest (Hz)
BREATHING_BAND = (0.1, 0.5)    # 6–30 BPM
HEARTBEAT_BAND = (0.8, 2.0)    # 48–120 BPM
MOTION_BAND    = (0.5, 5.0)    # walking, gestures


# ---------------------------------------------------------------------------
# spectral engines
# ---------------------------------------------------------------------------

class MultitaperEstimator:
    """PSD via Discrete Prolate Spheroidal Sequences (Slepian tapers).

    Averages spectra across multiple orthogonal tapers to reduce spectral
    leakage without sacrificing frequency resolution.
    """

    def __init__(self, time_bandwidth=3.5, n_tapers=5):
        self.time_bandwidth = time_bandwidth
        self.n_tapers = n_tapers

    def estimate(self, x, fs):
        n = len(x)
        if n < 16:
            return np.array([]), np.array([])

        tapers = dpss(n, self.time_bandwidth, self.n_tapers)
        freqs = sp_fft.rfftfreq(n, d=1.0 / fs)

        psd = np.zeros(len(freqs))
        for tap in tapers:
            psd += np.abs(sp_fft.rfft(x * tap)) ** 2
        psd /= self.n_tapers

        return freqs, psd


class SpectrogramEngine:
    """Short-time FFT spectrogram for tracking frequency evolution."""

    def __init__(self, win_size=32, overlap=0.5):
        self.win_size = win_size
        self.step = max(1, int(win_size * (1 - overlap)))

    def compute(self, x, fs):
        n = len(x)
        if n < self.win_size:
            return np.array([]), np.array([]), np.empty((0, 0))

        freqs = sp_fft.rfftfreq(self.win_size, d=1.0 / fs)
        window = np.hanning(self.win_size)
        cols = []
        times = []

        for start in range(0, n - self.win_size + 1, self.step):
            chunk = x[start : start + self.win_size]
            windowed = (chunk - chunk.mean()) * window
            cols.append(np.abs(sp_fft.rfft(windowed)) ** 2)
            times.append(start / fs)

        return np.array(times), freqs, np.array(cols).T


class WaveletEngine:
    """Continuous Wavelet Transform using Morlet wavelet.

    Better time-frequency resolution than FFT for non-stationary signals
    like intermittent human movement.
    """

    def __init__(self, wavelet="morl", n_scales=32):
        self.wavelet = wavelet
        self.n_scales = n_scales

    def analyze(self, x, fs):
        empty = {"breathing": 0.0, "motion": 0.0, "heartbeat": 0.0, "peak_freq": 0.0}
        if not _HAS_WAVELETS or len(x) < 16:
            return empty

        cf = pywt.central_frequency(self.wavelet)
        max_freq = min(5.0, fs / 2.1)
        target_freqs = np.linspace(max_freq, 0.05, self.n_scales)
        scales = cf * fs / target_freqs

        coeffs, actual_freqs = pywt.cwt(
            x, scales, self.wavelet, sampling_period=1.0 / fs
        )
        power = np.abs(coeffs) ** 2

        def band_energy(lo, hi):
            mask = (actual_freqs >= lo) & (actual_freqs <= hi)
            return float(np.sum(power[mask, :])) if mask.any() else 0.0

        peak_idx = np.argmax(np.sum(power, axis=1))
        return {
            "breathing": band_energy(*BREATHING_BAND),
            "motion":    band_energy(*MOTION_BAND),
            "heartbeat": band_energy(*HEARTBEAT_BAND),
            "peak_freq": float(actual_freqs[peak_idx]),
        }


# ---------------------------------------------------------------------------
# adaptive baseline
# ---------------------------------------------------------------------------

class BaselineCalibrator:
    """Learns the quiet-room RSSI signature, then scores deviation."""

    def __init__(self, duration=8):
        self.duration = duration
        self.mean = None
        self.std = None
        self.calibrated = False
        self._samples = []
        self._start = None

    def feed(self, rssi, ts):
        if self.calibrated:
            return
        if self._start is None:
            self._start = ts
        if ts - self._start < self.duration:
            self._samples.append(rssi)
        elif len(self._samples) >= 10:
            arr = np.array(self._samples)
            self.mean = float(arr.mean())
            self.std = float(arr.std())
            self.calibrated = True

    def deviation(self, current_mean, current_std):
        if not self.calibrated:
            return 0.0
        s = max(self.std, 0.01)
        return abs(current_mean - self.mean) / s + abs(current_std - self.std) / s


# ---------------------------------------------------------------------------
# channel hopper
# ---------------------------------------------------------------------------

class ChannelHopper:
    """Cycles through WiFi channels for multipath signal diversity."""

    def __init__(self, iface, channels=(1, 6, 11), dwell=0.5):
        self.iface = iface
        self.channels = channels
        self.dwell = dwell
        self.current = channels[0]
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _loop(self):
        idx = 0
        while self._running:
            ch = self.channels[idx % len(self.channels)]
            try:
                subprocess.run(
                    ["iw", "dev", self.iface, "set", "channel", str(ch)],
                    capture_output=True, timeout=2,
                )
                self.current = ch
            except Exception:
                pass
            idx += 1
            time.sleep(self.dwell)


# ---------------------------------------------------------------------------
# main sensor
# ---------------------------------------------------------------------------

class WifiSensor:
    """Captures RSSI from a target AP and runs spectral analysis."""

    def __init__(self, iface="wlan0", target_bssid=None, history=500):
        self.iface = iface
        self.target_bssid = target_bssid
        self.target_ssid = ""

        self.rssi = deque(maxlen=history)
        self.timestamps = deque(maxlen=history)
        self.chan_rssi = defaultdict(lambda: deque(maxlen=200))

        self._aps = {}
        self._locked = target_bssid is not None
        self._running = False
        self._channel = 1
        self.pkt_count = 0
        self.data_pkt_count = 0

        # analysis engines
        self._mt = MultitaperEstimator()
        self._spec = SpectrogramEngine()
        self._baseline = BaselineCalibrator()
        self._wavelet = WaveletEngine()

        # state tracking
        self._prev_state = None
        self._state_since = time.time()
        self.alerts = deque(maxlen=10)

    # -- packet handler -----------------------------------------------------

    def _on_packet(self, pkt):
        if not pkt.haslayer(RadioTap):
            return
        try:
            rssi_val = float(pkt[RadioTap].dBm_AntSignal)
        except (AttributeError, TypeError):
            return
        if not pkt.haslayer(Dot11):
            return

        dot11 = pkt[Dot11]
        bssid = self._extract_bssid(dot11)
        if not bssid:
            return
        self.pkt_count += 1

        ssid = self._extract_ssid(pkt)

        # scan phase: collect all APs to pick the best one
        if not self._locked:
            if bssid not in self._aps:
                self._aps[bssid] = {"ssid": ssid, "rssi": []}
            self._aps[bssid]["rssi"].append(rssi_val)
            if ssid:
                self._aps[bssid]["ssid"] = ssid
            return

        if bssid != self.target_bssid:
            return

        now = time.time()
        self.rssi.append(rssi_val)
        self.timestamps.append(now)
        self.chan_rssi[self._channel].append(rssi_val)
        self._baseline.feed(rssi_val, now)
        if ssid:
            self.target_ssid = ssid

    def _extract_bssid(self, dot11):
        if dot11.type == 0:  # management
            return dot11.addr3
        if dot11.type == 2:  # data
            self.data_pkt_count += 1
            flags = dot11.FCfield
            to_ds, from_ds = flags & 0x1, flags & 0x2
            if to_ds and not from_ds:
                return dot11.addr1
            if from_ds and not to_ds:
                return dot11.addr2
            return dot11.addr3
        return None

    def _extract_ssid(self, pkt):
        if pkt.haslayer(Dot11Beacon):
            try:
                return pkt[Dot11Beacon].network_stats().get("ssid", "")
            except Exception:
                pass
        return ""

    # -- lifecycle ----------------------------------------------------------

    def start(self):
        self._running = True
        threading.Thread(target=self._capture, daemon=True).start()

    def stop(self):
        self._running = False

    def _capture(self):
        sniff(
            iface=self.iface,
            prn=self._on_packet,
            store=False,
            stop_filter=lambda _: not self._running,
        )

    def scan_and_lock(self, seconds=5):
        if self.target_bssid:
            self.target_ssid = self.target_bssid
            return

        print(f"  scanning for {seconds}s to find strongest AP...")
        self._locked = False
        time.sleep(seconds)

        if not self._aps:
            print("  no APs found — check monitor mode")
            sys.exit(1)

        best, best_rssi = None, -999
        for bssid, info in self._aps.items():
            if len(info["rssi"]) < 3:
                continue
            median = float(np.median(info["rssi"]))
            if median > best_rssi:
                best_rssi = median
                best = bssid

        self.target_bssid = best
        self.target_ssid = self._aps[best].get("ssid") or "???"
        self._locked = True
        print(f"  locked: {best} ({self.target_ssid}) @ {best_rssi:.0f} dBm")

    # -- analysis -----------------------------------------------------------

    def analyze(self):
        vals = np.array(self.rssi)
        ts = np.array(self.timestamps)
        n = len(vals)

        r = self._empty_result()
        if n < 20:
            return r

        duration = ts[-1] - ts[0]
        if duration < 1:
            return r

        fs = n / duration
        r["sample_rate"] = fs
        r["rssi_now"] = float(vals[-1])
        r["rssi_mean"] = float(vals.mean())
        r["rssi_std"] = float(vals.std())
        r["rssi_min"] = float(vals.min())
        r["rssi_max"] = float(vals.max())
        r["n_samples"] = n

        centered = vals - vals.mean()

        # fft
        self._run_fft(centered, fs, r)
        # multi-taper
        self._run_multitaper(centered, fs, r)
        # wavelet
        self._run_wavelet(centered, fs, r)
        # spectrogram
        self._run_spectrogram(centered, fs, r)
        # baseline
        self._run_baseline(r)
        # channel correlation
        self._run_channel_correlation(r)
        # change-point detection
        r["change_points"] = self._cusum(vals, r["rssi_mean"], r["rssi_std"])
        # classify
        self._classify(r)
        # sparkline
        r["sparkline"] = self._sparkline(vals[-50:])

        return r

    def _empty_result(self):
        return {
            "n_samples": 0, "state": "COLLECTING", "confidence": 0.0,
            "rssi_now": 0, "rssi_mean": 0, "rssi_std": 0,
            "rssi_min": 0, "rssi_max": 0, "sample_rate": 0,
            "fft_breathing": 0, "fft_motion": 0, "fft_heartbeat": 0,
            "fft_dom_freq": 0,
            "mt_breathing": 0, "mt_motion": 0, "mt_heartbeat": 0,
            "mt_dom_freq": 0,
            "wt_breathing": 0, "wt_motion": 0, "wt_heartbeat": 0,
            "wt_peak_freq": 0,
            "spec_trend": "", "spec_heatmap": "",
            "baseline_calibrated": self._baseline.calibrated,
            "baseline_deviation": 0,
            "channel_correlation": 0, "channels_active": 0,
            "change_points": 0, "sparkline": "",
        }

    def _band_power(self, freqs, psd, lo, hi):
        mask = (freqs >= lo) & (freqs <= hi)
        return float(np.sum(psd[mask])) if mask.any() else 0.0

    def _run_fft(self, x, fs, r):
        n = len(x)
        freqs = sp_fft.rfftfreq(n, d=1.0 / fs)
        psd = np.abs(sp_fft.rfft(x * np.hanning(n))) ** 2
        if len(freqs) < 2:
            return
        r["fft_breathing"] = self._band_power(freqs, psd, *BREATHING_BAND)
        r["fft_heartbeat"] = self._band_power(freqs, psd, *HEARTBEAT_BAND)
        r["fft_motion"]    = self._band_power(freqs, psd, *MOTION_BAND)
        r["fft_dom_freq"]  = float(freqs[np.argmax(psd[1:]) + 1])

    def _run_multitaper(self, x, fs, r):
        freqs, psd = self._mt.estimate(x, fs)
        if len(freqs) < 2:
            return
        r["mt_breathing"] = self._band_power(freqs, psd, *BREATHING_BAND)
        r["mt_heartbeat"] = self._band_power(freqs, psd, *HEARTBEAT_BAND)
        r["mt_motion"]    = self._band_power(freqs, psd, *MOTION_BAND)
        r["mt_dom_freq"]  = float(freqs[np.argmax(psd[1:]) + 1])

    def _run_wavelet(self, x, fs, r):
        wt = self._wavelet.analyze(x, fs)
        r["wt_breathing"] = wt["breathing"]
        r["wt_heartbeat"] = wt["heartbeat"]
        r["wt_motion"]    = wt["motion"]
        r["wt_peak_freq"] = wt["peak_freq"]

    def _run_spectrogram(self, x, fs, r):
        times, freqs, data = self._spec.compute(x, fs)
        if data.size < 2 or data.ndim < 2 or data.shape[1] < 2:
            return

        # frequency trend line
        trend = []
        for ci in range(data.shape[1]):
            col = data[:, ci]
            bp = self._band_power(freqs, col, *BREATHING_BAND)
            mp = self._band_power(freqs, col, *MOTION_BAND)
            total = bp + mp + 0.001
            trend.append("M" if mp / total > 0.7 else ("b" if bp / total > 0.5 else "·"))
        r["spec_trend"] = "".join(trend[-50:])

        # heatmap rows (motion / heartbeat / breathing)
        blocks = " ░▒▓█"
        rows = []
        for lo, hi in [MOTION_BAND, HEARTBEAT_BAND, BREATHING_BAND]:
            mask = (freqs >= lo) & (freqs <= hi)
            if not mask.any():
                rows.append("")
                continue
            bp = np.sum(data[mask, :], axis=0)
            mx = bp.max() or 1
            rows.append("".join(blocks[min(4, int(v / mx * 4))] for v in bp[-50:]))
        r["spec_heatmap"] = rows

    def _run_baseline(self, r):
        r["baseline_calibrated"] = self._baseline.calibrated
        if self._baseline.calibrated:
            r["baseline_deviation"] = self._baseline.deviation(
                r["rssi_mean"], r["rssi_std"]
            )

    def _run_channel_correlation(self, r):
        active = {ch for ch, samples in self.chan_rssi.items() if len(samples) >= 10}
        r["channels_active"] = len(active)
        if len(active) < 2:
            return

        pairs = []
        channels = sorted(active)
        for i in range(len(channels)):
            for j in range(i + 1, len(channels)):
                a = np.array(self.chan_rssi[channels[i]])
                b = np.array(self.chan_rssi[channels[j]])
                k = min(len(a), len(b))
                if k >= 10:
                    c = float(np.corrcoef(a[-k:], b[-k:])[0, 1])
                    if not np.isnan(c):
                        pairs.append(c)
        if pairs:
            r["channel_correlation"] = float(np.mean(pairs))

    def _cusum(self, vals, mean, std):
        if std < 0.01:
            return 0
        threshold = 3.0 * std
        drift = 0.5 * std
        sp = sn = 0.0
        count = 0
        for i in range(1, len(vals)):
            d = vals[i] - mean
            sp = max(0.0, sp + d - drift)
            sn = max(0.0, sn - d - drift)
            if sp > threshold or sn > threshold:
                count += 1
                sp = sn = 0.0
        return count

    def _classify(self, r):
        var = r["rssi_std"] ** 2

        motion_score = (
            r["mt_motion"] / 500
            + r["fft_motion"] / 500
            + r["wt_motion"] / 5000
            + r["baseline_deviation"] / 5
        ) / 4

        breathing_score = (
            r["mt_breathing"] / 200
            + r["fft_breathing"] / 200
            + r["wt_breathing"] / 2000
        ) / 3

        if var < 0.3 and motion_score < 0.1:
            state, conf = "EMPTY ROOM", 1.0 - motion_score
        elif motion_score > 0.6 or var > 2.0 or r["change_points"] > 5:
            state, conf = "MOTION DETECTED", motion_score
        elif breathing_score > 0.3 and motion_score < 0.4:
            state, conf = "PERSON STILL (breathing?)", breathing_score
        elif motion_score > 0.15 or breathing_score > 0.15:
            state, conf = "PRESENCE LIKELY", max(motion_score, breathing_score)
        else:
            state, conf = "EMPTY ROOM", 1.0 - motion_score

        r["state"] = state
        r["confidence"] = max(0.0, min(1.0, conf))

        if state != self._prev_state and self._prev_state is not None:
            elapsed = time.time() - self._state_since
            self.alerts.appendleft(
                f"  {datetime.now().strftime('%H:%M:%S')} | "
                f"{self._prev_state} -> {state} (after {elapsed:.0f}s)"
            )
            self._state_since = time.time()
        self._prev_state = state

    @staticmethod
    def _sparkline(arr):
        if len(arr) == 0:
            return ""
        glyphs = " ▁▂▃▄▅▆▇█"
        lo, hi = arr.min(), arr.max()
        span = hi - lo or 1
        return "".join(glyphs[min(8, int((v - lo) / span * 8))] for v in arr)


# ---------------------------------------------------------------------------
# terminal display
# ---------------------------------------------------------------------------

_COLORS = {
    "EMPTY ROOM":              "\033[1;32m",
    "MOTION DETECTED":         "\033[1;31m",
    "PERSON STILL (breathing?)": "\033[1;33m",
    "PRESENCE LIKELY":         "\033[1;36m",
    "COLLECTING":              "\033[0;37m",
}
_RST = "\033[0m"
_DIM = "\033[2m"


def _bar(val, mx=500, width=15):
    filled = min(width, int(val / max(mx, 1) * width))
    return "█" * filled + "░" * (width - filled)


def render(sensor, hopper, r):
    print("\033[2J\033[H")
    c = _COLORS.get(r["state"], _RST)
    ch = f"ch{hopper.current}" if hopper else "fixed"
    w = 70

    print(f"{'═' * w}")
    print(f"  wifi_sense — passive presence detection")
    print(f"  target: {sensor.target_bssid} ({sensor.target_ssid})")
    print(
        f"  {datetime.now().strftime('%H:%M:%S')}  |  "
        f"samples: {r['n_samples']}  |  "
        f"pkts: {sensor.pkt_count} (data: {sensor.data_pkt_count})  |  "
        f"{ch}  |  {r['sample_rate']:.1f} Hz"
    )
    print(f"{'═' * w}")

    # state box
    print(f"\n  {c}{'━' * 54}")
    print(f"  ┃  {r['state']:^50s}┃")
    print(f"  ┃  confidence: {r['confidence']:>5.0%}{'':>37s}┃")
    print(f"  {'━' * 54}{_RST}\n")

    # signal stats
    print(f"  {_DIM}signal{_RST}")
    print(
        f"  ├─ now: {r['rssi_now']:.0f} dBm  |  "
        f"mean: {r['rssi_mean']:.1f}  |  "
        f"σ={r['rssi_std']:.2f}  |  "
        f"range: {r['rssi_min']:.0f} to {r['rssi_max']:.0f}"
    )
    bl = (
        f"deviation={r['baseline_deviation']:.1f}σ"
        if r["baseline_calibrated"]
        else "calibrating..."
    )
    print(f"  └─ baseline: {bl}")

    # spectral comparison
    print(f"\n  {_DIM}spectral analysis — 3 engines{_RST}")
    print(f"  {'':>22s} {'FFT':>15s}  {'multi-taper':>15s}  {'wavelet':>15s}")
    print(
        f"  breathing (0.1-0.5Hz) {_bar(r['fft_breathing'])} "
        f"{_bar(r['mt_breathing'])} {_bar(r['wt_breathing'], 5000)}"
    )
    print(
        f"  heartbeat (0.8-2.0Hz) {_bar(r['fft_heartbeat'])} "
        f"{_bar(r['mt_heartbeat'])} {_bar(r['wt_heartbeat'], 5000)}"
    )
    print(
        f"  motion    (0.5-5.0Hz) {_bar(r['fft_motion'])} "
        f"{_bar(r['mt_motion'])} {_bar(r['wt_motion'], 5000)}"
    )
    print(
        f"  dom freq:             {r['fft_dom_freq']:>7.3f} Hz      "
        f"{r['mt_dom_freq']:>7.3f} Hz      {r['wt_peak_freq']:>7.3f} Hz"
    )

    # spectrogram
    hm = r.get("spec_heatmap")
    if hm and len(hm) == 3:
        print(f"\n  {_DIM}spectrogram (time ->){_RST}")
        for label, row in zip(["  motion  ", "  heartbt ", "  breathe "], hm):
            print(f"  {label} |{row}|")

    if r["spec_trend"]:
        print(f"  {_DIM}  trend:    |{r['spec_trend']}|{_RST}")
        print(f"  {_DIM}            (M=motion  b=breathing  .=quiet){_RST}")

    # channel correlation
    if r["channels_active"] >= 2:
        corr_label = "correlated" if r["channel_correlation"] > 0.5 else "divergent"
        print(f"\n  {_DIM}channel correlation{_RST}")
        print(f"  ├─ active channels: {r['channels_active']}")
        print(f"  └─ cross-corr: {r['channel_correlation']:+.3f}  ({corr_label})")

    # waveform
    if r["sparkline"]:
        print(f"\n  {_DIM}rssi waveform (last 50 samples){_RST}")
        print(f"  {r['sparkline']}")

    print(f"\n  change-points: {r['change_points']}")

    # alerts
    if sensor.alerts:
        print(f"\n  {_DIM}state changes{_RST}")
        for a in list(sensor.alerts)[:5]:
            print(a)

    print(f"\n{'─' * w}")
    wt = " + wavelet(CWT)" if _HAS_WAVELETS else " (pip install PyWavelets for CWT)"
    print(f"  {_DIM}fft + multi-taper(DPSS){wt}{_RST}")
    print(f"  {_DIM}ctrl+c to stop{_RST}")


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="passive WiFi presence/motion detection via RSSI spectral analysis"
    )
    ap.add_argument("--iface", default="wlan0", help="monitor-mode interface")
    ap.add_argument("--bssid", default=None, help="target AP (auto-picks strongest)")
    ap.add_argument("--duration", type=int, default=180, help="run time in seconds")
    ap.add_argument("--interval", type=float, default=2.0, help="refresh interval")
    ap.add_argument("--no-hop", action="store_true", help="stay on one channel")
    ap.add_argument("--channels", default="1,6,11", help="channels to hop (comma-sep)")
    ap.add_argument("--dwell", type=float, default=0.5, help="dwell per channel (sec)")
    args = ap.parse_args()

    channels = tuple(int(c) for c in args.channels.split(","))

    engines = "fft + multi-taper(DPSS)"
    engines += " + wavelet(CWT)" if _HAS_WAVELETS else ""
    hop_info = f"channels {channels}, dwell {args.dwell}s" if not args.no_hop else "off"

    print(f"[*] wifi_sense")
    print(f"    engines: {engines}")
    print(f"    channel hop: {hop_info}")
    print(f"    capture: beacons + probes + data frames")

    sensor = WifiSensor(iface=args.iface, target_bssid=args.bssid)
    hopper = None
    if not args.no_hop:
        hopper = ChannelHopper(args.iface, channels=channels, dwell=args.dwell)

    def shutdown(_sig, _frame):
        sensor.stop()
        if hopper:
            hopper.stop()
        print("\n[*] stopped.")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    sensor.start()
    if hopper:
        hopper.start()
    sensor.scan_and_lock(seconds=5)

    deadline = time.time() + args.duration
    while time.time() < deadline and sensor._running:
        if hopper:
            sensor._channel = hopper.current
        render(sensor, hopper, sensor.analyze())
        time.sleep(args.interval)

    sensor.stop()
    if hopper:
        hopper.stop()


if __name__ == "__main__":
    main()
