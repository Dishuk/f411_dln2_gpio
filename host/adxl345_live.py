#!/usr/bin/env python3
"""Live ADXL345 plot in the browser (standard library only).

Usage: adxl345_live.py /dev/spidevN.0 [port]
Then open http://<host>:<port>/ (default port 8000).
"""

import collections
import http.server
import json
import socketserver
import struct
import sys
import threading
import time

import adxl345_read as adxl

REG_BW_RATE = 0x2C
RATE_HZ = 50

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>ADXL345 live</title>
<style>
  body { margin: 0; font: 14px system-ui, sans-serif; background: #111; color: #ddd; }
  header { padding: 12px 16px; display: flex; gap: 24px; align-items: baseline; flex-wrap: wrap; }
  h1 { font-size: 16px; margin: 0; }
  .v { font: 20px ui-monospace, monospace; min-width: 9ch; display: inline-block; }
  #x { color: #ff6b6b; } #y { color: #51cf66; } #z { color: #4dabf7; }
  canvas { display: block; width: 100%; height: calc(100vh - 60px); }
  #st { color: #888; }
</style></head><body>
<header>
  <h1>ADXL345</h1>
  <span>X <span class="v" id="x">-</span></span>
  <span>Y <span class="v" id="y">-</span></span>
  <span>Z <span class="v" id="z">-</span></span>
  <span>|a| <span class="v" id="m">-</span></span>
  <span id="st">connecting...</span>
</header>
<canvas id="c"></canvas>
<script>
const N = 500, cols = ["#ff6b6b", "#51cf66", "#4dabf7"];
const buf = [[], [], []];
const c = document.getElementById("c"), g = c.getContext("2d");
const el = id => document.getElementById(id);

function draw() {
  const w = c.width = c.clientWidth * devicePixelRatio;
  const h = c.height = c.clientHeight * devicePixelRatio;
  const R = 2.0, y = v => h / 2 - v / R * (h / 2 - 10);
  g.clearRect(0, 0, w, h);
  g.strokeStyle = "#333"; g.fillStyle = "#666"; g.lineWidth = 1;
  g.font = 12 * devicePixelRatio + "px sans-serif";
  for (let v = -2; v <= 2; v += 0.5) {
    g.beginPath(); g.moveTo(0, y(v)); g.lineTo(w, y(v)); g.stroke();
    if (v % 1 === 0) g.fillText(v + " g", 4, y(v) - 3);
  }
  g.lineWidth = 2 * devicePixelRatio;
  buf.forEach((b, i) => {
    g.strokeStyle = cols[i]; g.beginPath();
    b.forEach((v, j) => {
      const px = w - (b.length - 1 - j) * w / (N - 1);
      j ? g.lineTo(px, y(v)) : g.moveTo(px, y(v));
    });
    g.stroke();
  });
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

const es = new EventSource("stream");
es.onopen = () => el("st").textContent = "live";
es.onerror = () => el("st").textContent = "disconnected, retrying...";
es.onmessage = e => {
  for (const s of JSON.parse(e.data)) {
    s.forEach((v, i) => { buf[i].push(v); if (buf[i].length > N) buf[i].shift(); });
  }
  const [x, y, z] = buf.map(b => b[b.length - 1]);
  const f = v => (v >= 0 ? "+" : "") + v.toFixed(3);
  el("x").textContent = f(x); el("y").textContent = f(y); el("z").textContent = f(z);
  el("m").textContent = Math.hypot(x, y, z).toFixed(3);
};
</script></body></html>
"""

samples = collections.deque(maxlen=RATE_HZ * 10)
count = 0
cond = threading.Condition()


def sampler(dev):
    global count
    with open(dev, "r+b", buffering=0) as f:
        fd = f.fileno()
        if adxl.read(fd, adxl.REG_DEVID)[0] != 0xE5:
            sys.exit("ADXL345 not found")
        adxl.write(fd, adxl.REG_DATA_FORMAT, 0x08)  # full res, +-2 g
        adxl.write(fd, REG_BW_RATE, 0x09)           # 50 Hz output rate
        adxl.write(fd, adxl.REG_POWER_CTL, 0x08)    # measure
        period = 1.0 / RATE_HZ
        next_t = time.monotonic()
        while True:
            x, y, z = struct.unpack("<hhh", adxl.read(fd, adxl.REG_DATAX0, 6))
            with cond:
                samples.append([round(v * 0.0039, 4) for v in (x, y, z)])
                count += 1
                cond.notify_all()
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))


class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            body = PAGE.encode()
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
            seen = count
            try:
                while True:
                    with cond:
                        cond.wait_for(lambda: count > seen, timeout=5)
                        new = min(count - seen, len(samples))
                        batch = list(samples)[len(samples) - new:]
                        seen = count
                    self.wfile.write(f"data: {json.dumps(batch)}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.05)  # batch up to ~20 updates/s
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    dev = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
    threading.Thread(target=sampler, args=(dev,), daemon=True).start()
    print(f"open http://<this host>:{port}/")
    Server(("", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
