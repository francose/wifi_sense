#!/usr/bin/env python3
"""
rssi_finder — scan all nearby APs and classify presence/motion on each.

Puts a WiFi adapter in monitor mode, sniffs beacon and probe response
frames from every visible access point, and runs RSSI-based feature
extraction + classification per BSSID. Useful for getting a broad view
of which APs in your environment are being affected by human movement.

This is the simpler, multi-AP scanning companion to wifi_sense.py
(which locks onto a single AP for deeper analysis).

Requires: scapy, numpy, scipy
Author: (jynx) sadik erisen
"""

import argparse
import logging
import signal
import sys
import threading
import time
import warnings
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional

warnings.filterwarnings("ignore")

import numpy as np
from scipy import fft as sp_fft

logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
from scapy.all import Dot11, Dot11Beacon, Dot11ProbeResp, RadioTap, sniff


# ---------------------------------------------------------------------------
# data structures
# ---------------------------------------------------------------------------

class MotionLevel(Enum):
    ABSENT = "ABSENT"
    PRESENT_STILL = "PRESENT_STILL"
    ACTIVE = "ACTIVE"


@dataclass
class RssiSample:
    timestamp: float
    rssi: float
    bssid: str


@dataclass
class Features:
    mean: float = 0.0
    variance: float = 0.0
    std: float = 0.0
    skewness: float = 0.0
    kurtosis: float = 0.0
    iqr: float = 0.0
    dominant_freq_hz: float = 0.0
    breathing_band_power: float = 0.0
    motion_band_power: float = 0.0
    total_spectral_power: float = 0.0
    change_points: int = 0
    n_samples: int = 0
    sample_rate_hz: float = 0.0


@dataclass
class SensingResult:
    motion_level: MotionLevel
    confidence: float
    presence: bool
    features: Features
    bssid: str
    timestamp: str


# ---------------------------------------------------------------------------
# feature extraction
# ---------------------------------------------------------------------------

def extract_features(rssi_values: np.ndarray, sample_rate: float) -> Features:
    """Time-domain stats + frequency-domain decomposition from RSSI series."""
    n = len(rssi_values)
    if n < 10:
        return Features(n_samples=n)

    mean = float(np.mean(rssi_values))
    var = float(np.var(rssi_values))
    std = float(np.std(rssi_values))
    q1, q3 = np.percentile(rssi_values, [25, 75])
    iqr = float(q3 - q1)

    centered = rssi_values - mean
    if std > 1e-9:
        skew = float(np.mean((centered / std) ** 3))
        kurt = float(np.mean((centered / std) ** 4) - 3.0)
    else:
        skew = kurt = 0.0

    # FFT
    freqs = sp_fft.rfftfreq(n, d=1.0 / sample_rate)
    fft_mag = np.abs(sp_fft.rfft(rssi_values - mean))
    psd = fft_mag ** 2

    total_power = float(np.sum(psd[1:]))
    dominant_idx = np.argmax(fft_mag[1:]) + 1
    dominant_freq = float(freqs[dominant_idx])

    breathing_mask = (freqs >= 0.1) & (freqs <= 0.5)
    motion_mask = (freqs >= 0.5) & (freqs <= 3.0)
    breathing_power = float(np.sum(psd[breathing_mask]))
    motion_power = float(np.sum(psd[motion_mask]))

    # CUSUM change-point detection
    change_points = 0
    if std > 1e-9:
        threshold = 3.0 * std
        drift = 0.5 * std
        s_pos = s_neg = 0.0
        for i in range(1, n):
            diff = rssi_values[i] - mean
            s_pos = max(0, s_pos + diff - drift)
            s_neg = max(0, s_neg - diff - drift)
            if s_pos > threshold or s_neg > threshold:
                change_points += 1
                s_pos = s_neg = 0.0

    return Features(
        mean=mean, variance=var, std=std,
        skewness=skew, kurtosis=kurt, iqr=iqr,
        dominant_freq_hz=dominant_freq,
        breathing_band_power=breathing_power,
        motion_band_power=motion_power,
        total_spectral_power=total_power,
        change_points=change_points,
        n_samples=n,
        sample_rate_hz=sample_rate,
    )


# ---------------------------------------------------------------------------
# classifier
# ---------------------------------------------------------------------------

VARIANCE_ABSENT = 0.5
VARIANCE_MOTION = 3.0
MOTION_BAND_THRESH = 50.0
BREATHING_BAND_THRESH = 20.0


def classify(features: Features) -> tuple:
    """Rule-based presence/motion classification from extracted features."""
    if features.n_samples < 10:
        return MotionLevel.ABSENT, 0.0

    if features.variance < VARIANCE_ABSENT and features.motion_band_power < 10:
        level = MotionLevel.ABSENT
        conf = min(1.0, (VARIANCE_ABSENT - features.variance) / VARIANCE_ABSENT)
    elif features.variance > VARIANCE_MOTION or features.motion_band_power > MOTION_BAND_THRESH:
        level = MotionLevel.ACTIVE
        conf = min(1.0, features.motion_band_power / (MOTION_BAND_THRESH * 2))
    else:
        level = MotionLevel.PRESENT_STILL
        conf = min(1.0, features.breathing_band_power / (BREATHING_BAND_THRESH * 2))

    if features.change_points > 0 and level == MotionLevel.ACTIVE:
        conf = min(1.0, conf + 0.15)

    return level, max(0.0, min(1.0, conf))


# ---------------------------------------------------------------------------
# packet sniffer
# ---------------------------------------------------------------------------

class RssiScanner:
    """Monitors all nearby APs and collects RSSI time-series per BSSID."""

    def __init__(self, iface: str = "wlan0"):
        self.iface = iface
        self.samples: Dict[str, deque] = defaultdict(lambda: deque(maxlen=500))
        self.ssid_map: Dict[str, str] = {}
        self._running = False
        self.pkt_count = 0

    def _on_packet(self, pkt):
        if not pkt.haslayer(RadioTap):
            return
        if not (pkt.haslayer(Dot11Beacon) or pkt.haslayer(Dot11ProbeResp)):
            return

        try:
            rssi = pkt[RadioTap].dBm_AntSignal
        except (AttributeError, TypeError):
            return

        bssid = pkt[Dot11].addr3
        if bssid is None:
            return

        if pkt.haslayer(Dot11Beacon):
            try:
                ssid = pkt[Dot11Beacon].network_stats().get("ssid", "")
                if ssid:
                    self.ssid_map[bssid] = ssid
            except Exception:
                pass

        self.samples[bssid].append(RssiSample(
            timestamp=time.time(), rssi=float(rssi), bssid=bssid,
        ))
        self.pkt_count += 1

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        sniff(
            iface=self.iface, prn=self._on_packet, store=False,
            stop_filter=lambda _: not self._running,
        )

    def stop(self):
        self._running = False

    def analyze(self, target_bssid: Optional[str] = None) -> List[SensingResult]:
        """Run feature extraction + classification on all collected APs."""
        results = []
        bssids = [target_bssid] if target_bssid else list(self.samples.keys())

        for bssid in bssids:
            samples = list(self.samples.get(bssid, []))
            if len(samples) < 10:
                continue

            rssi_vals = np.array([s.rssi for s in samples])
            timestamps = np.array([s.timestamp for s in samples])

            duration = timestamps[-1] - timestamps[0]
            if duration < 0.1:
                continue
            sample_rate = len(timestamps) / duration

            features = extract_features(rssi_vals, sample_rate)
            level, confidence = classify(features)

            results.append(SensingResult(
                motion_level=level,
                confidence=confidence,
                presence=level != MotionLevel.ABSENT,
                features=features,
                bssid=bssid,
                timestamp=datetime.now().strftime("%H:%M:%S"),
            ))

        results.sort(key=lambda r: r.features.n_samples, reverse=True)
        return results


# ---------------------------------------------------------------------------
# display
# ---------------------------------------------------------------------------

def render(scanner: RssiScanner, results: List[SensingResult]):
    print("\033[2J\033[H")
    print("=" * 78)
    print("  rssi_finder — multi-AP presence scanner")
    print(
        f"  {datetime.now().strftime('%H:%M:%S')}  |  "
        f"packets: {scanner.pkt_count}  |  "
        f"APs seen: {len(scanner.samples)}"
    )
    print("=" * 78)

    if not results:
        print("\n  collecting data... (need ~10 beacons per AP)\n")
        return

    for r in results[:8]:
        ssid = scanner.ssid_map.get(r.bssid, "???")

        if r.motion_level == MotionLevel.ACTIVE:
            color, icon = "\033[1;31m", "[!]"
        elif r.motion_level == MotionLevel.PRESENT_STILL:
            color, icon = "\033[1;33m", "[~]"
        else:
            color, icon = "\033[1;32m", "[ ]"
        rst = "\033[0m"

        print(f"\n  {color}{icon} {r.bssid}  {ssid:<20s}{rst}")
        print(
            f"      state: {color}{r.motion_level.value}{rst}  "
            f"conf: {r.confidence:.0%}  "
            f"samples: {r.features.n_samples}"
        )
        print(
            f"      rssi: mean={r.features.mean:.1f} dBm  "
            f"std={r.features.std:.2f}  iqr={r.features.iqr:.2f}"
        )
        print(
            f"      spectral: breathing={r.features.breathing_band_power:.1f}  "
            f"motion={r.features.motion_band_power:.1f}  "
            f"dom_freq={r.features.dominant_freq_hz:.3f} Hz"
        )
        print(
            f"      change-points: {r.features.change_points}  "
            f"skew: {r.features.skewness:.2f}  kurt: {r.features.kurtosis:.2f}"
        )

    print(f"\n{'─' * 78}")
    print("  walk around to see ACTIVE / stand still for PRESENT_STILL")
    print("  ctrl+c to stop")


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="scan all nearby APs and classify presence/motion on each"
    )
    ap.add_argument("--iface", default="wlan0", help="monitor-mode interface")
    ap.add_argument("--duration", type=int, default=120, help="run time in seconds")
    ap.add_argument("--target-bssid", default=None, help="focus on one AP")
    ap.add_argument("--interval", type=float, default=3.0, help="refresh interval")
    args = ap.parse_args()

    print(f"[*] starting rssi scanner on {args.iface} for {args.duration}s...")
    scanner = RssiScanner(iface=args.iface)

    def shutdown(_sig, _frame):
        print("\n[*] stopping...")
        scanner.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    scanner.start()

    # collect for a few seconds before first display
    time.sleep(3)

    deadline = time.time() + args.duration
    while time.time() < deadline and scanner._running:
        results = scanner.analyze(args.target_bssid)
        render(scanner, results)
        time.sleep(args.interval)

    scanner.stop()
    print("\n[*] final analysis:")
    for r in scanner.analyze(args.target_bssid)[:5]:
        ssid = scanner.ssid_map.get(r.bssid, "???")
        print(
            f"  {r.bssid} ({ssid}): {r.motion_level.value} "
            f"conf={r.confidence:.0%} samples={r.features.n_samples}"
        )


if __name__ == "__main__":
    main()
