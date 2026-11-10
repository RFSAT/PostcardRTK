#!/usr/bin/env python3
"""Fake NTRIP caster for bench-testing PostcardRTK failover without touching real casters.

Mount points:
  /GOOD      ICY 200 OK, then synthetic RTCM 1005 + 1077 frames every second (or a recorded file, --rtcm)
  /STALL     ICY 200 OK, then nothing (tests the "no RTCM data" timeout)
  /DROP      streams for 20 s, then closes the socket (tests "connection broken")
  /AUTH      401 Unauthorized unless user:pass = test:test (then like /GOOD)
  anything else: SOURCETABLE 200 OK (unknown mount point)

Usage: python3 fake_caster.py --port 2101 [--rtcm recorded.rtcm3]
Point a PostcardRTK source at <this PC's IP>:2101 with one of the mount points above.
"""
import argparse, base64, socketserver, threading, time


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def rtcm_frame(msg_type: int, station: int, extra: int) -> bytes:
    payload = bytes([msg_type >> 4, ((msg_type & 0xF) << 4) | ((station >> 8) & 0xF), station & 0xFF]) + bytes(
        (i * 7) & 0xFF for i in range(extra))
    f = bytes([0xD3, len(payload) >> 8, len(payload) & 0xFF]) + payload
    c = crc24q(f)
    return f + bytes([c >> 16, (c >> 8) & 0xFF, c & 0xFF])


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        req = b""
        while b"\r\n\r\n" not in req and len(req) < 4096:
            chunk = self.request.recv(1024)
            if not chunk:
                return
            req += chunk
        head = req.decode(errors="replace")
        line = head.split("\r\n")[0]
        mount = line.split(" ")[1].lstrip("/") if " " in line else ""
        auth = ""
        for h in head.split("\r\n"):
            if h.lower().startswith("authorization: basic "):
                auth = base64.b64decode(h.split(" ", 2)[2]).decode(errors="replace")
        print(f"{self.client_address[0]} {line!r} auth={auth!r}")
        w = self.wfile
        if mount == "AUTH" and auth != "test:test":
            w.write(b"HTTP/1.0 401 Unauthorized\r\nWWW-Authenticate: Basic realm=\"x\"\r\n\r\n")
            return
        if mount not in ("GOOD", "STALL", "DROP", "AUTH"):
            w.write(b"SOURCETABLE 200 OK\r\nContent-Type: text/plain\r\n\r\nSTR;GOOD;...\r\nENDSOURCETABLE\r\n")
            return
        w.write(b"ICY 200 OK\r\n\r\n")
        start = time.time()
        recorded = self.server.rtcm
        pos = 0
        try:
            while True:
                # Read (and print) GGA sent by the rover, without blocking
                self.request.settimeout(0.0)
                try:
                    data = self.request.recv(512)
                    if data:
                        print("  GGA:", data.decode(errors="replace").strip())
                except (BlockingIOError, OSError):
                    pass
                self.request.settimeout(None)
                if mount == "STALL":
                    time.sleep(1)
                    continue
                if mount == "DROP" and time.time() - start > 20:
                    print("  dropping connection")
                    return
                if recorded:
                    w.write(recorded[pos:pos + 1200])
                    pos = (pos + 1200) % len(recorded)
                else:
                    w.write(rtcm_frame(1005, 2003, 16) + rtcm_frame(1077, 2003, 300) + rtcm_frame(1097, 2003, 250))
                w.flush()
                time.sleep(1)
        except (BrokenPipeError, ConnectionResetError):
            print("  client gone")


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--rtcm", help="recorded RTCM 3 file to replay on /GOOD")
    a = ap.parse_args()
    srv = Server(("0.0.0.0", a.port), Handler)
    srv.rtcm = open(a.rtcm, "rb").read() if a.rtcm else b""
    print(f"fake caster on port {a.port}: mounts GOOD, STALL, DROP, AUTH (test:test)")
    srv.serve_forever()
