#!/usr/bin/env python3
"""Live monitor for the G1 arm motor bus. Listens only, never transmits.

  monitor.py                      live from the RS-485 adapter
  monitor.py --replay FILE.bin    play back a capture made by sniff.py

Frame layout: see protocol/g1-arm-bus.md.
"""
import argparse
import math
import os
import re
import sys
import threading
import time

import numpy as np
import pyqtgraph as pg
import serial
from PySide6 import QtCore, QtWidgets

from sniff import PORT, set_baud

BAUD = 6_000_000

# One command (20 bytes) immediately followed by its reply (26 bytes).
PAIR = re.compile(rb"\xfe\xee[\x90-\x9f][\s\S]{17}\xfc\xee[\x10-\x1f][\s\S]{23}")
PAIR_DTYPE = np.dtype([
    ("c_hdr", "<u2"), ("c_id", "u1"), ("c_b3", "u1"),
    ("c_tau", "<i2"), ("c_vel", "<i2"), ("c_pos", "<i4"), ("kp", "<i2"), ("kd", "<i2"), ("c_crc", "<u4"),
    ("r_hdr", "<u2"), ("r_id", "u1"), ("temp_a", "u1"), ("temp_b", "u1"), ("b5", "u1"),
    ("tau", "<i2"), ("vel", "<i2"), ("pos", "<i4"), ("zero", "<u4"), ("u18", "<u2"),
    ("b20", "u1"), ("b21", "u1"), ("r_crc", "<u4"),
])
assert PAIR_DTYPE.itemsize == 46


def _crc_table() -> np.ndarray:
    t = np.zeros(256, dtype=np.uint32)
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = ((c << 1) ^ 0x04C11DB7 if c & 0x80000000 else c << 1) & 0xFFFFFFFF
        t[i] = c
    return t


CRC_TABLE = _crc_table()


def crc32_words(block: np.ndarray) -> np.ndarray:
    """Unitree crc32_core over little-endian 32-bit words, for many frames at once.

    block is uint8 with shape (frames, 4 * words). Equivalent to CRC-32/MPEG-2 with the
    bytes of every word taken most-significant first.
    """
    n, width = block.shape
    crc = np.full(n, 0xFFFFFFFF, dtype=np.uint32)
    for w in range(0, width, 4):
        for b in (3, 2, 1, 0):
            crc = (crc << np.uint32(8)) ^ CRC_TABLE[(crc >> np.uint32(24)) ^ block[:, w + b]]
    return crc


def valid_pairs(raw: np.ndarray) -> np.ndarray:
    """raw: uint8 (pairs, 46). True where both checksums are right."""
    c_ok = crc32_words(raw[:, 0:16]) == raw[:, 16:20].copy().view("<u4")[:, 0]    # whole command
    r_ok = crc32_words(raw[:, 22:42]) == raw[:, 42:46].copy().view("<u4")[:, 0]   # reply without header
    return c_ok & r_ok

# Scales. The position/velocity ratio (128 counts/s per velocity count) was checked
# against the capture; the absolute values are ASSUMED from the public GO-M8010-6 SDK
# and refer to the rotor side (before the gearbox).
POS_RAD = 2 * math.pi / 32768   # rad per position count
VEL_RAD = 2 * math.pi / 256     # rad/s per velocity count
TAU_NM = 1 / 256                # N*m per torque count

NOMINAL_DT = 1 / 1313.0   # per-motor sample period measured from the first captures
HISTORY = 1 << 17         # samples kept per motor (about 100 s)
ACC_WINDOW = 65           # samples in the line fit used for acceleration (about 50 ms)
GAP_RESYNC_S = 0.1        # timestamp jump that restarts the sample clock
FIELDS = ("t", "pos", "vel", "tau", "c_pos", "c_tau", "c_vel", "kp", "kd", "temp_a", "temp_b", "b5", "u18")


class MotorBuffer:
    """Ring buffer of decoded samples for one motor."""

    def __init__(self):
        self.data = {f: np.zeros(HISTORY) for f in FIELDS}
        self.n = 0            # total samples ever written
        self.t_last = None    # time of the newest sample
        self.dt = NOMINAL_DT

    def append(self, rows: np.ndarray, t_host: float) -> None:
        m = len(rows)
        if m == 0:
            return
        # The USB adapter delivers bytes in bursts, so host arrival time is too jittery to
        # differentiate. Samples are stamped on a steady clock that is slowly steered
        # toward the host time.
        if self.t_last is None or abs(t_host - (self.t_last + m * self.dt)) > GAP_RESYNC_S:
            t = t_host - self.dt * np.arange(m - 1, -1, -1)
        else:
            err = t_host - (self.t_last + m * self.dt)
            self.dt = min(max(self.dt + 1e-4 * err / m, 0.5 * NOMINAL_DT), 2 * NOMINAL_DT)
            t = self.t_last + self.dt * np.arange(1, m + 1) + 0.02 * err
        self.t_last = t[-1]
        idx = (self.n + np.arange(m)) % HISTORY
        self.data["t"][idx] = t
        for f in FIELDS[1:]:
            self.data[f][idx] = rows[f]
        self.n += m

    def tail(self, seconds: float) -> dict:
        count = min(self.n, HISTORY, int(seconds / self.dt) + ACC_WINDOW)
        idx = (self.n - count + np.arange(count)) % HISTORY
        return {f: self.data[f][idx] for f in FIELDS}


class Bus:
    """Reads bytes from the adapter (or a capture file) and fills per-motor buffers."""

    def __init__(self, port: str, replay: str | None):
        self.port, self.replay = port, replay
        self.motors: dict[int, MotorBuffer] = {}
        self.lock = threading.Lock()
        self.pairs = 0
        self.rejected = 0
        self.bytes = 0
        self.error = ""
        self.running = True
        self._left = b""
        threading.Thread(target=self._run, daemon=True).start()

    def _feed(self, chunk: bytes, t_host: float) -> None:
        buf = self._left + chunk
        matches = list(PAIR.finditer(buf))
        if matches:
            raw = np.frombuffer(b"".join(m.group() for m in matches), dtype=np.uint8).reshape(-1, 46)
            ok = valid_pairs(raw)
            rows = raw[ok].copy().view(PAIR_DTYPE)[:, 0]
            rows = rows[(rows["c_id"] & 0x0F) == (rows["r_id"] & 0x0F)]
            ids = rows["c_id"] & 0x0F
            with self.lock:
                for k in np.unique(ids):
                    k = int(k)
                    if k not in self.motors:
                        self.motors[k] = MotorBuffer()
                    self.motors[k].append(rows[ids == k], t_host)
                self.pairs += len(rows)
                self.rejected += len(raw) - len(rows)
            keep_from = max(matches[-1].end(), len(buf) - 45)
        else:
            keep_from = max(0, len(buf) - 45)
        self._left = buf[keep_from:]
        self.bytes += len(chunk)

    def _run(self) -> None:
        try:
            if self.replay:
                self._run_replay()
            else:
                self._run_serial()
        except Exception as e:  # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"

    def _run_serial(self) -> None:
        ser = serial.Serial(self.port, 4_000_000, timeout=0.05)
        set_baud(ser.fd, BAUD)
        try:
            ser.set_low_latency_mode(True)
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.1)
        ser.reset_input_buffer()
        while self.running:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                self._feed(chunk, time.monotonic())
            time.sleep(0.002)   # let a few milliseconds of data pile up; cheaper than tiny reads
        ser.close()

    def _run_replay(self) -> None:
        data = open(self.replay, "rb").read()
        idx_path = self.replay[:-4] + ".idx"
        if os.path.exists(idx_path):
            idx = np.loadtxt(idx_path)
            rate = idx[-1, 1] / max(idx[-1, 0] - idx[0, 0], 1e-3)   # bytes per second
        else:
            rate = 300_000.0   # typical for 5 motors at 1.3 kHz
        t0 = time.monotonic()
        sent = 0
        while self.running:
            due = int((time.monotonic() - t0) * rate)
            if due > sent:
                self._feed(data[sent:due], time.monotonic())
                sent = due
            if sent >= len(data):   # loop the file
                t0, sent, self._left = time.monotonic(), 0, b""
            time.sleep(0.002)


def acceleration(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Slope of a least-squares line through the last ACC_WINDOW velocity samples."""
    n = ACC_WINDOW
    if len(v) < n + 1:
        return np.zeros_like(v)
    dt = (t[-1] - t[0]) / (len(t) - 1)
    k = np.arange(n) - (n - 1) / 2
    kernel = (k / (k @ k) / dt)[::-1]
    out = np.zeros_like(v)
    out[n - 1:] = np.convolve(v, kernel, mode="valid")
    return out


class Window(QtWidgets.QWidget):
    def __init__(self, bus: Bus):
        super().__init__()
        self.bus = bus
        self.setWindowTitle("G1 arm motor monitor")
        self.resize(1100, 950)
        self.last_pairs, self.last_time = 0, time.monotonic()

        self.motor = QtWidgets.QComboBox()
        self.span = QtWidgets.QComboBox()
        for s in (2, 5, 10, 30, 60):
            self.span.addItem(f"{s} s", s)
        self.span.setCurrentIndex(2)
        self.units = QtWidgets.QComboBox()
        self.units.addItem("Rotor units (assumed GO-M8010 scale)", True)
        self.units.addItem("Raw counts", False)
        self.units.currentIndexChanged.connect(self._label_axes)
        self.pause = QtWidgets.QPushButton("Pause")
        self.pause.setCheckable(True)
        self.status = QtWidgets.QLabel("waiting for data")
        self.readout = QtWidgets.QLabel("")

        top = QtWidgets.QHBoxLayout()
        for label, w in (("Motor", self.motor), ("Window", self.span), ("Units", self.units)):
            top.addWidget(QtWidgets.QLabel(label))
            top.addWidget(w)
        top.addWidget(self.pause)
        top.addStretch(1)
        top.addWidget(self.status)

        pg.setConfigOptions(antialias=False)
        self.graphics = pg.GraphicsLayoutWidget()
        self.plots, self.curves = {}, {}
        specs = [
            ("pos", [("measured", "y"), ("commanded", "c")]),
            ("vel", [("measured", "y")]),
            ("acc", [("from velocity", "m")]),
            ("tau", [("measured", "y")]),
            ("temp", [("sensor A", "r"), ("sensor B", (255, 150, 0))]),
        ]
        prev = None
        for row, (name, lines) in enumerate(specs):
            p = self.graphics.addPlot(row=row, col=0)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.addLegend(offset=(5, 5))
            p.setDownsampling(auto=True, mode="peak")
            p.setClipToView(True)
            p.getAxis("left").enableAutoSIPrefix(False)
            if prev is not None:
                p.setXLink(prev)
            prev = p
            self.plots[name] = p
            self.curves[name] = [p.plot(pen=pg.mkPen(color, width=1), name=label) for label, color in lines]
        prev.setLabel("bottom", "time", units="s")
        self._label_axes()

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.readout)
        layout.addWidget(self.graphics, 1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        hz = QtWidgets.QApplication.primaryScreen().refreshRate() or 60.0
        self.timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.timer.start(max(1, round(1000 / hz)))   # redraw at the screen refresh rate
        self.frames = 0

    def _label_axes(self) -> None:
        si = self.units.currentData()
        self.plots["pos"].setLabel("left", "angle", units="rad" if si else "counts")
        self.plots["vel"].setLabel("left", "velocity", units="rad/s" if si else "counts")
        self.plots["acc"].setLabel("left", "acceleration", units="rad/s²" if si else "counts/s")
        self.plots["tau"].setLabel("left", "torque", units="N·m" if si else "counts")
        self.plots["temp"].setLabel("left", "temperature (unconfirmed)", units="°C")

    def refresh(self) -> None:
        with self.bus.lock:
            ids = sorted(self.bus.motors)
            known = [self.motor.itemData(i) for i in range(self.motor.count())]
            if ids != known:
                current = self.motor.currentData()
                self.motor.clear()
                for k in ids:
                    self.motor.addItem(f"Motor {k}", k)
                if current in ids:
                    self.motor.setCurrentIndex(ids.index(current))
            k = self.motor.currentData()
            d = self.bus.motors[k].tail(self.span.currentData()) if k is not None else None
            pairs, rejected = self.bus.pairs, self.bus.rejected

        now = time.monotonic()
        if now - self.last_time >= 1.0:
            rate = (pairs - self.last_pairs) / (now - self.last_time)
            src = "replay" if self.bus.replay else "live"
            fps = self.frames / (now - self.last_time)
            text = f"{src}: {rate:,.0f} frame pairs/s, {len(ids)} motors, {rejected} bad frames, {fps:.0f} fps"
            if self.bus.error:
                text = f"ERROR  {self.bus.error}"
            elif rate == 0:
                text = f"{src}: bus silent"
            self.status.setText(text)
            self.last_pairs, self.last_time, self.frames = pairs, now, 0

        if d is None or self.pause.isChecked() or len(d["t"]) < 2:
            return
        si = self.units.currentData()
        ps, vs, ts = (POS_RAD, VEL_RAD, TAU_NM) if si else (1.0, 1.0, 1.0)
        t = d["t"] - d["t"][-1]
        acc = acceleration(d["t"], d["vel"] * vs)
        keep = t >= -self.span.currentData()
        t = t[keep]
        self.curves["pos"][0].setData(t, d["pos"][keep] * ps)
        self.curves["pos"][1].setData(t, d["c_pos"][keep] * ps)
        self.curves["vel"][0].setData(t, d["vel"][keep] * vs)
        self.curves["acc"][0].setData(t, acc[keep])
        self.curves["tau"][0].setData(t, d["tau"][keep] * ts)
        self.curves["temp"][0].setData(t, d["temp_a"][keep])
        self.curves["temp"][1].setData(t, d["temp_b"][keep])
        self.frames += 1
        self.readout.setText(
            f"command: kp {d['kp'][-1]:.0f}   kd {d['kd'][-1]:.0f}   torque ff {d['c_tau'][-1]:.0f}   "
            f"velocity {d['c_vel'][-1]:.0f}   (raw counts)      "
            f"reply: byte5 {d['b5'][-1]:.0f}   word18 {d['u18'][-1]:.0f}"
        )

    def closeEvent(self, event) -> None:
        self.bus.running = False
        super().closeEvent(event)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--replay", help="capture .bin file to play back instead of the live bus")
    ap.add_argument("--no-gl", action="store_true", help="draw without the GPU (much slower)")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()
    # Software drawing manages under 10 fps on a HiDPI window; OpenGL gives about 50.
    pg.setConfigOptions(useOpenGL=not args.no_gl)

    app = QtWidgets.QApplication(sys.argv[:1])
    win = Window(Bus(args.port, args.replay))
    win.show()
    if args.screenshot:
        QtCore.QTimer.singleShot(6000, lambda: (win.grab().save(args.screenshot), app.quit()))
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
