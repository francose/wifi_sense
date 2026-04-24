#!/usr/bin/env python3
"""
csi_receiver — real CSI analysis from ESP32-S3 over USB serial.

Reads ADR-018 CSI frames from serial, extracts per-subcarrier amplitude
and phase, and runs the full spectral pipeline: FFT, multi-taper, wavelet,
spectrogram, and subcarrier correlation matrix.

64 subcarriers of complex I/Q per frame at ~20 Hz — this is what enables
actual presence sensing, breathing detection, and motion classification
that RSSI alone can't do.

Author: sadik erisen
"""

import argparse
import signal
import sys
import threading
import time
import warnings
from collections import deque
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

import serial

# frequency bands
BREATHING_BAND = (0.1, 0.5)
HEARTBEAT_BAND = (0.8, 2.0)
MOTION_BAND    = (0.5, 5.0)

# subcarrier groups for spatial analysis
# ESP32-S3 gives 64 subcarriers (128 bytes / 2 = 64 I/Q pairs)
# Group them into regions for spatial diversity
SC_GROUPS = {
    "low":  (0, 16),    # subcarriers 0-15
    "mid":  (16, 48),   # subcarriers 16-47
    "high": (48, 64),   # subcarriers 48-63
}


class CSIFrame:
    """Parsed CSI frame from serial."""
    __slots__ = ["rssi", "channel", "noise_floor", "timestamp",
                 "iq_raw", "amplitude", "phase", "n_subcarriers"]

    def __init__(self, rssi, channel, noise_floor, iq_bytes):
        self.rssi = rssi
        self.channel = channel
        self.noise_floor = noise_floor
        self.timestamp = time.time()

        # Parse I/Q pairs: each pair is (real, imag) as signed int8
        n = len(iq_bytes) // 2
        self.n_subcarriers = n
        self.iq_raw = np.zeros(n, dtype=complex)

        for i in range(n):
            real = iq_bytes[2 * i]
            imag = iq_bytes[2 * i + 1]
            # Convert unsigned to signed
            if real > 127: real -= 256
            if imag > 127: imag -= 256
            self.iq_raw[i] = complex(real, imag)

        self.amplitude = np.abs(self.iq_raw)
        self.phase = np.angle(self.iq_raw)


class CSISerialReader:
    """Reads CSI frames from ESP32-S3 USB serial."""

    def __init__(self, port="/dev/ttyACM0", baud=115200, history=300):
        self.port = port
        self.baud = baud
        self.frames = deque(maxlen=history)
        self._running = False
        self.frame_count = 0
        self.error_count = 0

    def start(self):
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _read_loop(self):
        ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(1)  # let device settle

        while self._running:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line.startswith("CSI,"):
                    continue

                parts = line.split(",")
                if len(parts) < 5:
                    continue

                rssi = int(parts[1])
                channel = int(parts[2])
                noise_floor = int(parts[3])
                iq_len = int(parts[4])
                hex_data = parts[5] if len(parts) > 5 else ""

                # Decode hex I/Q data
                iq_bytes = bytes.fromhex(hex_data[:iq_len * 2])
                if len(iq_bytes) < 4:
                    continue

                frame = CSIFrame(rssi, channel, noise_floor, iq_bytes)
                self.frames.append(frame)
                self.frame_count += 1

            except (ValueError, IndexError):
                self.error_count += 1
            except serial.SerialException:
                time.sleep(0.5)

        ser.close()


class CSIAnalyzer:
    """Full spectral analysis on CSI subcarrier data."""

    def __init__(self):
        self._mt = MultitaperPSD()
        self._baseline_amp = None
        self._baseline_phase = None
        self._cal_frames = []
        self._cal_done = False
        self._cal_start = None
        self._prev_state = None
        self._state_since = time.time()
        self.alerts = deque(maxlen=10)

    def analyze(self, frames):
        n = len(frames)
        r = self._empty()
        if n < 15:
            return r

        # Extract time-series per subcarrier
        n_sc = frames[-1].n_subcarriers
        timestamps = np.array([f.timestamp for f in frames])
        duration = timestamps[-1] - timestamps[0]
        if duration < 1:
            return r

        fs = n / duration
        r["sample_rate"] = fs
        r["n_frames"] = n
        r["n_subcarriers"] = n_sc
        r["rssi_now"] = frames[-1].rssi
        r["noise_floor"] = frames[-1].noise_floor
        r["channel"] = frames[-1].channel

        # Build amplitude matrix: (n_frames, n_subcarriers)
        amp_matrix = np.array([f.amplitude for f in frames])
        phase_matrix = np.array([f.phase for f in frames])

        # RSSI from frames
        rssi_arr = np.array([f.rssi for f in frames])
        r["rssi_mean"] = float(rssi_arr.mean())
        r["rssi_std"] = float(rssi_arr.std())

        # --- Calibration (first 5 seconds) ---
        if not self._cal_done:
            if self._cal_start is None:
                self._cal_start = timestamps[0]
            if timestamps[-1] - self._cal_start < 5:
                self._cal_frames.extend(frames)
                r["state"] = "CALIBRATING"
                return r
            else:
                cal_amps = np.array([f.amplitude for f in self._cal_frames])
                self._baseline_amp = cal_amps.mean(axis=0)
                self._baseline_phase = np.array(
                    [f.phase for f in self._cal_frames]
                ).mean(axis=0)
                self._cal_done = True

        # --- Per-subcarrier amplitude variance ---
        # High variance subcarriers = affected by movement
        sc_variance = np.var(amp_matrix, axis=0)
        r["sc_variance_mean"] = float(sc_variance.mean())
        r["sc_variance_max"] = float(sc_variance.max())
        r["top_variance_scs"] = np.argsort(sc_variance)[-5:][::-1].tolist()

        # --- Baseline deviation per subcarrier ---
        if self._baseline_amp is not None:
            current_amp = amp_matrix.mean(axis=0)
            amp_deviation = np.abs(current_amp - self._baseline_amp)
            r["baseline_deviation"] = float(amp_deviation.mean())
            r["sc_deviation"] = amp_deviation

        # --- Aggregate amplitude time-series (mean across subcarriers) ---
        mean_amp = amp_matrix.mean(axis=1)
        centered = mean_amp - mean_amp.mean()

        # --- Top-K subcarrier analysis ---
        # Pick the 8 most variable subcarriers for spectral analysis
        top_k = min(8, n_sc)
        top_scs = np.argsort(sc_variance)[-top_k:]
        top_amp = amp_matrix[:, top_scs].mean(axis=1)
        top_centered = top_amp - top_amp.mean()

        # --- FFT on top-K subcarriers ---
        self._run_fft(top_centered, fs, r)

        # --- Multi-taper on top-K ---
        self._run_multitaper(top_centered, fs, r)

        # --- Wavelet on top-K ---
        self._run_wavelet(top_centered, fs, r)

        # --- Phase analysis (breathing is strongest here) ---
        # Use phase difference between consecutive frames on top subcarriers
        phase_top = phase_matrix[:, top_scs]
        phase_diff = np.diff(np.unwrap(phase_top, axis=0), axis=0)
        mean_phase_diff = phase_diff.mean(axis=1)
        if len(mean_phase_diff) > 10:
            pd_centered = mean_phase_diff - mean_phase_diff.mean()
            pd_freqs = sp_fft.rfftfreq(len(pd_centered), d=1.0 / fs)
            pd_psd = np.abs(sp_fft.rfft(pd_centered * np.hanning(len(pd_centered)))) ** 2
            bm = (pd_freqs >= 0.1) & (pd_freqs <= 0.5)
            hm = (pd_freqs >= 0.8) & (pd_freqs <= 2.0)
            r["phase_breathing"] = float(np.sum(pd_psd[bm])) if bm.any() else 0
            r["phase_heartbeat"] = float(np.sum(pd_psd[hm])) if hm.any() else 0

        # --- Subcarrier correlation matrix ---
        # If movement is real, nearby subcarriers correlate
        if n_sc >= 8 and n > 20:
            # Sample 8 evenly spaced subcarriers
            sc_indices = np.linspace(0, n_sc - 1, 8, dtype=int)
            sc_sample = amp_matrix[:, sc_indices]
            corr = np.corrcoef(sc_sample.T)
            r["sc_correlation_mean"] = float(
                np.nanmean(corr[np.triu_indices_from(corr, k=1)])
            )
            r["sc_correlation_matrix"] = corr

        # --- Spectrogram on top-K ---
        self._run_spectrogram(top_centered, fs, r)

        # --- CUSUM change-points ---
        r["change_points"] = self._cusum(top_amp)

        # --- Classification ---
        self._classify(r)

        # --- Sparklines ---
        r["sparkline_amp"] = self._sparkline(mean_amp[-50:])
        r["sparkline_rssi"] = self._sparkline(rssi_arr[-50:])

        # Phase sparkline (breathing visible here)
        if len(mean_phase_diff) >= 20:
            r["sparkline_phase"] = self._sparkline(mean_phase_diff[-50:])

        return r

    def _empty(self):
        return {
            "n_frames": 0, "n_subcarriers": 0, "sample_rate": 0,
            "state": "COLLECTING", "confidence": 0.0,
            "rssi_now": 0, "rssi_mean": 0, "rssi_std": 0,
            "noise_floor": 0, "channel": 0,
            "sc_variance_mean": 0, "sc_variance_max": 0,
            "top_variance_scs": [],
            "baseline_deviation": 0, "sc_deviation": None,
            "fft_breathing": 0, "fft_motion": 0, "fft_heartbeat": 0,
            "fft_dom_freq": 0,
            "mt_breathing": 0, "mt_motion": 0, "mt_heartbeat": 0,
            "mt_dom_freq": 0,
            "wt_breathing": 0, "wt_motion": 0, "wt_heartbeat": 0,
            "wt_peak_freq": 0,
            "phase_breathing": 0, "phase_heartbeat": 0,
            "sc_correlation_mean": 0, "sc_correlation_matrix": None,
            "spec_trend": "", "spec_heatmap": "",
            "change_points": 0,
            "sparkline_amp": "", "sparkline_rssi": "", "sparkline_phase": "",
        }

    def _band_power(self, freqs, psd, lo, hi):
        mask = (freqs >= lo) & (freqs <= hi)
        return float(np.sum(psd[mask])) if mask.any() else 0.0

    def _run_fft(self, x, fs, r):
        n = len(x)
        if n < 10:
            return
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
        if not _HAS_WAVELETS or len(x) < 16:
            return
        cf = pywt.central_frequency("morl")
        max_f = min(5.0, fs / 2.1)
        target_f = np.linspace(max_f, 0.05, 32)
        scales = cf * fs / target_f
        coeffs, freqs = pywt.cwt(x, scales, "morl", sampling_period=1.0 / fs)
        power = np.abs(coeffs) ** 2

        def be(lo, hi):
            m = (freqs >= lo) & (freqs <= hi)
            return float(np.sum(power[m, :])) if m.any() else 0.0

        r["wt_breathing"] = be(*BREATHING_BAND)
        r["wt_heartbeat"] = be(*HEARTBEAT_BAND)
        r["wt_motion"]    = be(*MOTION_BAND)
        peak = np.argmax(np.sum(power, axis=1))
        r["wt_peak_freq"] = float(freqs[peak])

    def _run_spectrogram(self, x, fs, r):
        win = 32
        step = 16
        n = len(x)
        if n < win:
            return
        freqs = sp_fft.rfftfreq(win, d=1.0 / fs)
        window = np.hanning(win)
        cols = []
        for s in range(0, n - win + 1, step):
            chunk = x[s:s + win]
            cols.append(np.abs(sp_fft.rfft((chunk - chunk.mean()) * window)) ** 2)
        data = np.array(cols).T
        if data.shape[1] < 2:
            return

        trend = []
        for ci in range(data.shape[1]):
            col = data[:, ci]
            bp = self._band_power(freqs, col, *BREATHING_BAND)
            mp = self._band_power(freqs, col, *MOTION_BAND)
            total = bp + mp + 0.001
            trend.append("M" if mp / total > 0.7 else ("b" if bp / total > 0.5 else "."))
        r["spec_trend"] = "".join(trend[-50:])

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

    def _cusum(self, vals):
        std = float(np.std(vals))
        if std < 0.01:
            return 0
        mean = float(np.mean(vals))
        threshold = 3.0 * std
        drift = 0.5 * std
        sp = sn = 0.0
        count = 0
        for v in vals[1:]:
            d = v - mean
            sp = max(0.0, sp + d - drift)
            sn = max(0.0, sn - d - drift)
            if sp > threshold or sn > threshold:
                count += 1
                sp = sn = 0.0
        return count

    def _classify(self, r):
        # Fuse amplitude-domain and phase-domain evidence
        amp_motion = (
            r["fft_motion"] / 100 +
            r["mt_motion"] / 100 +
            r["wt_motion"] / 1000 +
            r["baseline_deviation"] / 3
        ) / 4

        amp_breathing = (
            r["fft_breathing"] / 50 +
            r["mt_breathing"] / 50 +
            r["wt_breathing"] / 500
        ) / 3

        # Phase-domain (much more sensitive for breathing/heartbeat)
        phase_breathing = r["phase_breathing"] / 10
        phase_heartbeat = r["phase_heartbeat"] / 5

        # Subcarrier correlation: high = correlated movement across spectrum
        sc_corr = r["sc_correlation_mean"]

        # Combined scores
        motion_score = amp_motion + (0.2 if sc_corr > 0.6 else 0)
        breathing_score = max(amp_breathing, phase_breathing)
        heartbeat_score = phase_heartbeat

        var = r["sc_variance_mean"]

        if var < 0.5 and motion_score < 0.1 and breathing_score < 0.1:
            state, conf = "EMPTY ROOM", 1.0 - motion_score
        elif motion_score > 0.5 or var > 5.0 or r["change_points"] > 5:
            state, conf = "MOTION DETECTED", min(1.0, motion_score)
        elif breathing_score > 0.2 and motion_score < 0.3:
            state = "PERSON STILL"
            if heartbeat_score > 0.1:
                state += " (breathing + heartbeat)"
            else:
                state += " (breathing)"
            conf = min(1.0, breathing_score)
        elif motion_score > 0.1 or breathing_score > 0.1:
            state, conf = "PRESENCE LIKELY", max(motion_score, breathing_score)
        else:
            state, conf = "EMPTY ROOM", 1.0 - motion_score

        r["state"] = state
        r["confidence"] = max(0.0, min(1.0, conf))

        if state != self._prev_state and self._prev_state is not None:
            elapsed = time.time() - self._state_since
            self.alerts.appendleft(
                f"  {datetime.now().strftime('%H:%M:%S')} | "
                f"{self._prev_state} -> {state} ({elapsed:.0f}s)"
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


class MultitaperPSD:
    def __init__(self, nw=3.5, k=5):
        self.nw = nw
        self.k = k

    def estimate(self, x, fs):
        n = len(x)
        if n < 16:
            return np.array([]), np.array([])
        tapers = dpss(n, self.nw, self.k)
        freqs = sp_fft.rfftfreq(n, d=1.0 / fs)
        psd = np.zeros(len(freqs))
        for t in tapers:
            psd += np.abs(sp_fft.rfft(x * t)) ** 2
        return freqs, psd / self.k


# ---------------------------------------------------------------------------
# display
# ---------------------------------------------------------------------------

_COLORS = {
    "EMPTY ROOM":       "\033[1;32m",
    "MOTION DETECTED":  "\033[1;31m",
    "PRESENCE LIKELY":  "\033[1;36m",
    "CALIBRATING":      "\033[1;35m",
    "COLLECTING":       "\033[0;37m",
}
_RST = "\033[0m"
_DIM = "\033[2m"


def _bar(val, mx=100, width=15):
    filled = min(width, int(val / max(mx, 0.001) * width))
    return "█" * filled + "░" * (width - filled)


def _sc_heatmap(variance, n=64):
    """Single-line heatmap of subcarrier variance."""
    if len(variance) == 0:
        return ""
    blocks = " ░▒▓█"
    mx = variance.max() or 1
    # Compress to ~50 chars
    step = max(1, len(variance) // 50)
    out = ""
    for i in range(0, len(variance), step):
        chunk = variance[i:i + step]
        v = chunk.mean()
        out += blocks[min(4, int(v / mx * 4))]
    return out


def render(reader, analyzer, r):
    print("\033[2J\033[H")
    c = _COLORS.get(r["state"], _RST)
    # Check for partial matches (PERSON STILL variants)
    if "PERSON STILL" in r["state"]:
        c = "\033[1;33m"
    w = 74

    print(f"{'═' * w}")
    print(f"  wifi_sense CSI receiver — ESP32-S3 subcarrier analysis")
    print(
        f"  {datetime.now().strftime('%H:%M:%S')}  |  "
        f"frames: {r['n_frames']}  |  "
        f"subcarriers: {r['n_subcarriers']}  |  "
        f"ch{r['channel']}  |  "
        f"{r['sample_rate']:.1f} Hz  |  "
        f"total: {reader.frame_count}"
    )
    print(f"{'═' * w}")

    # state
    state_display = r["state"]
    print(f"\n  {c}{'━' * 58}")
    print(f"  ┃  {state_display:^54s}┃")
    print(f"  ┃  confidence: {r['confidence']:>5.0%}{'':>41s}┃")
    print(f"  {'━' * 58}{_RST}\n")

    # signal
    print(f"  {_DIM}signal{_RST}")
    print(
        f"  ├─ rssi: {r['rssi_now']} dBm  |  mean: {r['rssi_mean']:.1f}  |  "
        f"noise: {r['noise_floor']} dBm  |  snr: {r['rssi_now'] - r['noise_floor']} dB"
    )
    print(
        f"  └─ baseline deviation: {r['baseline_deviation']:.2f}  |  "
        f"change-points: {r['change_points']}"
    )

    # subcarrier analysis
    print(f"\n  {_DIM}subcarrier variance (64 subcarriers){_RST}")
    top = r["top_variance_scs"]
    print(f"  ├─ mean: {r['sc_variance_mean']:.2f}  |  max: {r['sc_variance_max']:.2f}")
    print(f"  ├─ hottest: {', '.join(f'sc{s}' for s in top)}")
    if r.get("sc_deviation") is not None:
        print(f"  └─ |{_sc_heatmap(r['sc_deviation'])}|")
    else:
        print(f"  └─ (calibrating...)")

    # correlation
    if r["sc_correlation_mean"] != 0:
        corr = r["sc_correlation_mean"]
        label = "strong" if corr > 0.7 else ("moderate" if corr > 0.4 else "weak")
        print(f"\n  {_DIM}subcarrier cross-correlation{_RST}")
        print(f"  └─ mean: {corr:+.3f} ({label} — {'real movement' if corr > 0.5 else 'noise/multipath'})")

    # spectral — amplitude domain
    print(f"\n  {_DIM}spectral analysis — amplitude domain{_RST}")
    print(f"  {'':>22s} {'FFT':>15s}  {'multi-taper':>15s}  {'wavelet':>15s}")
    print(
        f"  breathing (0.1-0.5Hz) {_bar(r['fft_breathing'])} "
        f"{_bar(r['mt_breathing'])} {_bar(r['wt_breathing'], 1000)}"
    )
    print(
        f"  heartbeat (0.8-2.0Hz) {_bar(r['fft_heartbeat'])} "
        f"{_bar(r['mt_heartbeat'])} {_bar(r['wt_heartbeat'], 1000)}"
    )
    print(
        f"  motion    (0.5-5.0Hz) {_bar(r['fft_motion'])} "
        f"{_bar(r['mt_motion'])} {_bar(r['wt_motion'], 1000)}"
    )
    print(
        f"  dom freq:             {r['fft_dom_freq']:>7.3f} Hz      "
        f"{r['mt_dom_freq']:>7.3f} Hz      {r['wt_peak_freq']:>7.3f} Hz"
    )

    # spectral — phase domain (more sensitive for vitals)
    print(f"\n  {_DIM}phase-domain vitals (more sensitive than amplitude){_RST}")
    print(f"  ├─ breathing: {_bar(r['phase_breathing'], 50)} {r['phase_breathing']:.1f}")
    print(f"  └─ heartbeat: {_bar(r['phase_heartbeat'], 20)} {r['phase_heartbeat']:.1f}")

    # spectrogram
    hm = r.get("spec_heatmap")
    if hm and len(hm) == 3:
        print(f"\n  {_DIM}spectrogram (time ->){_RST}")
        for label, row in zip(["  motion  ", "  heartbt ", "  breathe "], hm):
            print(f"  {label} |{row}|")
    if r["spec_trend"]:
        print(f"  {_DIM}  trend:    |{r['spec_trend']}|{_RST}")

    # waveforms
    if r["sparkline_amp"]:
        print(f"\n  {_DIM}amplitude waveform (top-K subcarriers){_RST}")
        print(f"  {r['sparkline_amp']}")
    if r["sparkline_phase"]:
        print(f"  {_DIM}phase rate (breathing visible here){_RST}")
        print(f"  {r['sparkline_phase']}")

    # alerts
    if analyzer.alerts:
        print(f"\n  {_DIM}state changes{_RST}")
        for a in list(analyzer.alerts)[:5]:
            print(a)

    print(f"\n{'─' * w}")
    engines = "fft + multi-taper(DPSS)"
    engines += " + wavelet(CWT)" if _HAS_WAVELETS else ""
    print(f"  {_DIM}{engines} | 64 subcarriers | phase + amplitude domain{_RST}")
    print(f"  {_DIM}ctrl+c to stop{_RST}")


def main():
    ap = argparse.ArgumentParser(description="CSI receiver for ESP32-S3")
    ap.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=int, default=300, help="run time (sec)")
    ap.add_argument("--interval", type=float, default=2.0, help="refresh interval")
    args = ap.parse_args()

    print(f"[*] csi_receiver — reading from {args.port}")
    print(f"    64 subcarriers x ~20 Hz = real CSI, not RSSI")

    reader = CSISerialReader(port=args.port, baud=args.baud)
    analyzer = CSIAnalyzer()

    def shutdown(_s, _f):
        reader.stop()
        print(f"\n[*] stopped. {reader.frame_count} frames captured.")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    reader.start()

    deadline = time.time() + args.duration
    while time.time() < deadline:
        r = analyzer.analyze(list(reader.frames))
        render(reader, analyzer, r)
        time.sleep(args.interval)

    reader.stop()


if __name__ == "__main__":
    main()
