#!/usr/bin/env python3
"""Live DLN-2 viewer in the browser (standard library only): ADXL345 over
spidev and the dln2-adc channels.

Usage: dln2_live.py [--spi /dev/spidevN.0] [--port 8000]
                    [--cal VREFINT_CAL,TS_CAL1,TS_CAL2]
Then open http://<host>:<port>/.

The ADC panel appears when a `dln2-adc` IIO device exists. --cal takes the
STM32F411 factory values from 0x1FFF7A2A/2C/2E (12-bit at 3.3 V); without it,
datasheet typical values are used for VDDA and temperature.
"""

import argparse
import collections
import glob
import http.server
import json
import socketserver
import struct
import threading
import time

ACC_HZ = 50
ADC_HZ = 10
ADC_NAMES = ["A0", "A1", "A2", "A3", "VREFINT", "TEMP"]

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>DLN-2 live</title>
<style>
  body { margin: 0; font: 14px system-ui, sans-serif; background: #111; color: #ddd; }
  section { padding: 12px 16px; border-bottom: 1px solid #222; }
  .row { display: flex; gap: 24px; align-items: baseline; flex-wrap: wrap; }
  h1 { font-size: 16px; margin: 0; }
  .v { font: 20px ui-monospace, monospace; min-width: 9ch; display: inline-block; }
  .c0 { color: #ff6b6b; } .c1 { color: #51cf66; } .c2 { color: #4dabf7; } .c3 { color: #fcc419; }
  canvas { display: block; width: 100%; height: 34vh; margin-top: 8px; }
  table { border-collapse: collapse; font: 14px ui-monospace, monospace; margin-top: 8px; }
  td, th { padding: 2px 12px 2px 0; text-align: right; }
  th { color: #888; font-weight: normal; }
  td:first-child, th:first-child { text-align: left; }
  .bar { width: 200px; height: 10px; background: #222; }
  .bar div { height: 100%; background: #4dabf7; }
  #st { color: #888; }
</style></head><body>
<section><div class="row"><h1>DLN-2 live</h1><span id="st">connecting...</span></div></section>

<section id="acc" hidden>
  <div class="row">
    <h1>ADXL345</h1>
    <span>X <span class="v c0" id="x">-</span></span>
    <span>Y <span class="v c1" id="y">-</span></span>
    <span>Z <span class="v c2" id="z">-</span></span>
    <span>|a| <span class="v" id="m">-</span></span>
  </div>
  <canvas id="acc_c"></canvas>
</section>

<section id="adc" hidden>
  <div class="row">
    <h1>ADC</h1>
    <span>VDDA <span class="v" id="vdda">-</span></span>
    <span>Temp <span class="v" id="temp">-</span></span>
    <span id="calsrc"></span>
  </div>
  <table>
    <tr><th>ch</th><th>input</th><th>raw</th><th>volts</th><th></th></tr>
    <tbody id="adc_t"></tbody>
  </table>
  <canvas id="adc_c"></canvas>
</section>

<script>
const CFG = __CFG__;
const el = id => document.getElementById(id);
const COLS = ["#ff6b6b", "#51cf66", "#4dabf7", "#fcc419"];
const ACC_N = 500, ADC_N = 300;
const acc = [[], [], []], adc = [[], [], [], []];
const f3 = v => (v >= 0 ? "+" : "") + v.toFixed(3);

function push(buf, v, n) { buf.push(v); if (buf.length > n) buf.shift(); }

function chart(canvas, bufs, n, lo, hi, step, unit) {
  const g = canvas.getContext("2d");
  const w = canvas.width = canvas.clientWidth * devicePixelRatio;
  const h = canvas.height = canvas.clientHeight * devicePixelRatio;
  const y = v => h - 8 - (v - lo) / (hi - lo) * (h - 16);
  g.clearRect(0, 0, w, h);
  g.strokeStyle = "#333"; g.fillStyle = "#666"; g.lineWidth = 1;
  g.font = 12 * devicePixelRatio + "px sans-serif";
  for (let v = lo; v <= hi + 1e-9; v += step) {
    g.beginPath(); g.moveTo(0, y(v)); g.lineTo(w, y(v)); g.stroke();
    g.fillText(+v.toFixed(2) + " " + unit, 4, y(v) - 3);
  }
  g.lineWidth = 2 * devicePixelRatio;
  bufs.forEach((b, i) => {
    g.strokeStyle = COLS[i]; g.beginPath();
    b.forEach((v, j) => {
      const px = w - (b.length - 1 - j) * w / (n - 1);
      j ? g.lineTo(px, y(v)) : g.moveTo(px, y(v));
    });
    g.stroke();
  });
}

function draw() {
  if (CFG.acc) chart(el("acc_c"), acc, ACC_N, -2, 2, 0.5, "g");
  if (CFG.adc) chart(el("adc_c"), adc, ADC_N, 0, 3.3, 0.5, "V");
  requestAnimationFrame(draw);
}

el("acc").hidden = !CFG.acc;
el("adc").hidden = !CFG.adc;
if (CFG.adc) {
  el("calsrc").textContent = CFG.cal ? "(factory calibration)" : "(typical values, no --cal)";
  el("adc_t").innerHTML = CFG.names.map((name, i) =>
    `<tr><td class="c${i}">${i}</td><td>${name}</td><td id="r${i}">-</td>` +
    `<td id="u${i}">-</td><td><div class="bar"><div id="b${i}" style="width:0"></div></div></td></tr>`
  ).join("");
}
requestAnimationFrame(draw);

const es = new EventSource("stream");
es.onopen = () => el("st").textContent = "live";
es.onerror = () => el("st").textContent = "disconnected, retrying...";
es.onmessage = e => {
  const m = JSON.parse(e.data);
  for (const s of m.acc) s.forEach((v, i) => push(acc[i], v, ACC_N));
  if (m.acc.length) {
    const [x, y, z] = acc.map(b => b[b.length - 1]);
    el("x").textContent = f3(x); el("y").textContent = f3(y); el("z").textContent = f3(z);
    el("m").textContent = Math.hypot(x, y, z).toFixed(3);
  }
  for (const s of m.adc) {
    s.volts.slice(0, 4).forEach((v, i) => push(adc[i], v, ADC_N));
    s.raw.forEach((r, i) => {
      el("r" + i).textContent = r;
      el("u" + i).textContent = s.volts[i].toFixed(3);
      el("b" + i).style.width = (r / 1023 * 100).toFixed(1) + "%";
    });
    el("vdda").textContent = s.vdda.toFixed(3) + " V";
    el("temp").textContent = s.temp.toFixed(1) + " \\u00b0C";
  }
};
</script></body></html>
"""

cond = threading.Condition()
acc_samples = collections.deque(maxlen=ACC_HZ * 10)
adc_samples = collections.deque(maxlen=ADC_HZ * 10)
seq = {"acc": 0, "adc": 0}


def publish(kind, buf, value):
    with cond:
        buf.append(value)
        seq[kind] += 1
        cond.notify_all()


def acc_sampler(dev):
    import adxl345_read as adxl
    with open(dev, "r+b", buffering=0) as f:
        fd = f.fileno()
        if adxl.read(fd, adxl.REG_DEVID)[0] != 0xE5:
            raise SystemExit("ADXL345 not found")
        adxl.write(fd, adxl.REG_DATA_FORMAT, 0x08)  # full res, +-2 g
        adxl.write(fd, 0x2C, 0x09)                  # BW_RATE: 50 Hz
        adxl.write(fd, adxl.REG_POWER_CTL, 0x08)    # measure
        period, next_t = 1.0 / ACC_HZ, time.monotonic()
        while True:
            x, y, z = struct.unpack("<hhh", adxl.read(fd, adxl.REG_DATAX0, 6))
            publish("acc", acc_samples, [round(v * 0.0039, 4) for v in (x, y, z)])
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))


def find_adc():
    for d in glob.glob("/sys/bus/iio/devices/iio:device*"):
        with open(f"{d}/name") as f:
            if f.read().strip() == "dln2-adc":
                return d
    return None


def adc_sampler(d, cal):
    period, next_t = 1.0 / ADC_HZ, time.monotonic()
    while True:
        raw = []
        for c in range(len(ADC_NAMES)):
            with open(f"{d}/in_voltage{c}_raw") as f:
                raw.append(int(f.read()))
        if cal:
            vref_cal, ts1, ts2 = cal
            vdda = 3.3 * (vref_cal / 4) / max(raw[4], 1)
            code = raw[5] * 4 * vdda / 3.3              # as measured at 3.3 V
            temp = 30 + (code - ts1) * 80 / (ts2 - ts1)
        else:
            vdda = 1.21 * 1024 / max(raw[4], 1)         # VREFINT typ 1.21 V
            temp = (raw[5] * vdda / 1024 - 0.76) / 0.0025 + 25   # V25 0.76 V, 2.5 mV/C
        volts = [round(r * vdda / 1024, 4) for r in raw]
        publish("adc", adc_samples, {"raw": raw, "volts": volts,
                                     "vdda": round(vdda, 4), "temp": round(temp, 2)})
        next_t += period
        time.sleep(max(0.0, next_t - time.monotonic()))


def newest(buf, count):
    count = min(count, len(buf))
    return list(buf)[len(buf) - count:] if count else []


def make_handler(page):
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body = page.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                seen = dict(seq)
                try:
                    while True:
                        with cond:
                            cond.wait_for(lambda: seq != seen, timeout=5)
                            msg = {"acc": newest(acc_samples, seq["acc"] - seen["acc"]),
                                   "adc": newest(adc_samples, seq["adc"] - seen["adc"])}
                            seen = dict(seq)
                        self.wfile.write(f"data: {json.dumps(msg)}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(0.05)  # batch up to ~20 updates/s
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass

    return Handler


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--spi", help="ADXL345 spidev node, e.g. /dev/spidev0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--cal", help="VREFINT_CAL,TS_CAL1,TS_CAL2 (decimal)")
    a = p.parse_args()

    cal = tuple(int(v) for v in a.cal.split(",")) if a.cal else None
    adc = find_adc()
    if not a.spi and not adc:
        raise SystemExit("nothing to show: no --spi and no dln2-adc IIO device")
    if a.spi:
        threading.Thread(target=acc_sampler, args=(a.spi,), daemon=True).start()
    if adc:
        threading.Thread(target=adc_sampler, args=(adc, cal), daemon=True).start()

    cfg = {"acc": bool(a.spi), "adc": bool(adc), "cal": bool(cal), "names": ADC_NAMES}
    page = PAGE.replace("__CFG__", json.dumps(cfg))
    print(f"accelerometer: {a.spi or 'off'}, adc: {adc or 'not found'}")
    print(f"open http://<this host>:{a.port}/")
    Server(("", a.port), make_handler(page)).serve_forever()


if __name__ == "__main__":
    main()
