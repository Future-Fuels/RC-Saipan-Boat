#!/usr/bin/env python3

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import re
from pathlib import Path
import threading
import time

import serial
from serial.tools import list_ports

STATIC = Path(__file__).resolve().parent / "static"
SIMULATOR_FILE = STATIC.parent / ".simulator.json"
MAKEFILE = STATIC.parent.parent / "Makefile"
BAUD_RATE = 115200
SERVER_ADDRESS = ("127.0.0.1", 8000)


def detect_port():
    try:
        simulator = json.loads(SIMULATOR_FILE.read_text())
        pid, port = simulator["pid"], simulator["port"]
        if (
            type(pid) is int
            and pid > 0
            and isinstance(port, str)
            and Path(port).exists()
        ):
            os.kill(pid, 0)
            return port
    except (OSError, ValueError, KeyError, TypeError):
        pass

    try:
        match = re.search(
            r"^PORT_TX\s*[:?+]?=\s*(\S+)", MAKEFILE.read_text(), re.MULTILINE
        )
        if match:
            configured = match.group(1)
            for port in (configured.replace("/dev/tty.", "/dev/cu.", 1), configured):
                if Path(port).exists():
                    return port
    except OSError:
        pass

    ports = [
        p.device
        for p in list_ports.comports()
        if p.vid is not None or re.search(r"usb|acm", p.device, re.IGNORECASE)
    ]
    if len(ports) == 1:
        return ports[0]
    if len(ports) > 1:
        raise ValueError(
            "Multiple serial devices found. Set PORT_TX in the Makefile to the TX device."
        )
    raise ValueError("Waiting for TX. Plug it in or start uv run simulate.py.")


class AutoConnector:
    def __init__(self, bridge):
        self.bridge = bridge
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            if not self.bridge.snapshot()["connected"]:
                try:
                    self.bridge.connect(detect_port())
                except (OSError, ValueError) as exc:
                    self.bridge.update(error=str(exc))
            self.stop.wait(2)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)


def validate_command(command):
    if not isinstance(command, str) or not 0 < len(command) < 64:
        raise ValueError("Invalid command")
    if not command.isascii() or any(char in command for char in "\r\n\0"):
        raise ValueError("Send one command at a time")
    command = " ".join(command.lower().split())
    if command in ("stop", "resume", "unlock", "gps reset"):
        return command
    match = re.fullmatch(r"(servo|motor|limit) (free|-?[0-9]+)", command)
    if match:
        name, argument = match.groups()
        if argument == "free":
            return command
        minimum, maximum = {"servo": (0, 180), "motor": (-255, 255), "limit": (0, 255)}[
            name
        ]
        if minimum <= int(argument) <= maximum:
            return command
    raise ValueError(
        "Use servo 0..180|free, motor -255..255|free, limit 0..255|free, stop, resume, unlock, or gps reset"
    )


class LineDecoder:
    def __init__(self, limit=8192):
        self.limit = limit
        self.buffer = bytearray()
        self.discarding = False

    def feed(self, chunk):
        for byte in chunk:
            if byte == 10:
                if self.discarding:
                    yield None
                elif self.buffer:
                    yield self.buffer.decode("utf-8", errors="replace").rstrip("\r")
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                self.buffer.append(byte)
                if len(self.buffer) > self.limit:
                    self.buffer.clear()
                    self.discarding = True


class SerialBridge:
    def __init__(self):
        self.condition = threading.Condition()
        self.operation = threading.Lock()
        self.device = None
        self.reader = None
        self.stop = threading.Event()
        self.revision = 0
        self.pending_ack = None
        self.state = dict(
            connected=False,
            port=None,
            error=None,
            packetCount=0,
            lastReceivedAt=None,
            telemetry=None,
        )

    def update(self, **changes):
        with self.condition:
            if all(self.state.get(key) == value for key, value in changes.items()):
                return
            self.state.update(changes)
            self.revision += 1
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return dict(self.state, revision=self.revision)

    def disconnect(self):
        with self.operation:
            self._disconnect()

    def _disconnect(self):
        self.stop.set()
        if self.reader:
            self.reader.join(timeout=2)
        if self.device:
            self.device.close()
        self.device = self.reader = None
        self.update(connected=False)

    def connect(self, port):
        with self.operation:
            self._disconnect()
            device = serial.Serial(
                baudrate=BAUD_RATE,
                timeout=0.2,
                write_timeout=1,
                exclusive=True,
            )
            device.port = port
            device.dtr = False
            device.rts = False
            try:
                device.open()
            except (OSError, ValueError) as exc:
                device.close()
                self.update(error=str(exc))
                raise
            self.device = device
            self.stop = threading.Event()
            self.update(
                connected=True,
                port=port,
                error=None,
                packetCount=0,
                lastReceivedAt=None,
                telemetry=None,
            )
            self.reader = threading.Thread(
                target=self._read, args=(device, self.stop), daemon=True
            )
            self.reader.start()

    def send_command(self, command):
        command = validate_command(command)
        with self.operation:
            if (
                not self.device
                or not self.device.is_open
                or not self.state["connected"]
            ):
                raise ValueError("Connect to TX first")
            ready, result = threading.Event(), {}
            with self.condition:
                self.pending_ack = (ready, result)
            try:
                payload = (command + "\n").encode("ascii")
                if self.device.write(payload) != len(payload):
                    raise OSError("Could not write the complete command")
                if not ready.wait(2):
                    raise ValueError(
                        "No acknowledgement from TX; check the latest packet before retrying"
                    )
                if not result["accepted"]:
                    raise ValueError("TX rejected the command")
            finally:
                with self.condition:
                    self.pending_ack = None
        return command

    def accept_line(self, line):
        try:
            packet = json.loads(line)
            json.dumps(packet, allow_nan=False)
        except (TypeError, ValueError, RecursionError):
            return
        if not isinstance(packet, dict):
            return
        if type(packet.get("commandAccepted")) is bool:
            with self.condition:
                if self.pending_ack:
                    ready, result = self.pending_ack
                    result["accepted"] = packet["commandAccepted"]
                    ready.set()
            return
        if (
            not {"j1x", "j1y", "j2x", "j2y", "speed", "connected", "telemetryFresh"}
            <= packet.keys()
        ):
            return
        with self.condition:
            self.update(
                telemetry=packet,
                lastReceivedAt=int(time.time() * 1000),
                packetCount=self.state["packetCount"] + 1,
            )

    def _read(self, device, stop):
        decoder = LineDecoder()
        try:
            while not stop.is_set():
                chunk = device.read(min(4096, device.in_waiting or 1))
                for line in decoder.feed(chunk):
                    if not stop.is_set():
                        self.accept_line(line)
        except OSError as exc:
            if not stop.is_set():
                self.update(connected=False, error=f"Serial connection lost: {exc}")
        finally:
            device.close()


class MonitorServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, bridge):
        self.bridge = bridge
        super().__init__(SERVER_ADDRESS, Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def local_request(self):
        hosts = {
            f"127.0.0.1:{self.server.server_port}",
            f"localhost:{self.server.server_port}",
        }
        return self.headers.get("Host") in hosts and self.headers.get("Origin") in (
            None,
            *(f"http://{host}" for host in hosts),
        )

    def respond(self, payload, status=200, content_type="application/json"):
        body = (
            payload
            if isinstance(payload, bytes)
            else json.dumps(payload, allow_nan=False).encode()
        )
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self.local_request():
            return self.respond({"error": "Use the local monitor URL."}, 403)
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            return self.respond(self.server.bridge.snapshot())
        if path == "/api/events":
            return self.events()
        assets = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
        }
        if path not in assets:
            return self.respond({"error": "Not found"}, 404)
        filename, mime = assets[path]
        self.respond((STATIC / filename).read_bytes(), content_type=mime)

    def do_POST(self):
        if not self.local_request():
            return self.respond({"error": "Use the local monitor URL."}, 403)
        if self.headers.get_content_type() != "application/json":
            return self.respond({"error": "Expected application/json"}, 415)
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 4096:
                raise ValueError("Expected a JSON body under 4096 bytes")
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError("Expected a JSON object")
            if self.path == "/api/connect":
                self.server.bridge.connect(detect_port())
            elif self.path == "/api/command":
                command = self.server.bridge.send_command(body.get("command"))
                return self.respond({"command": command, "accepted": True})
            else:
                return self.respond({"error": "Not found"}, 404)
            self.respond(self.server.bridge.snapshot())
        except (OSError, ValueError) as exc:
            self.respond({"error": str(exc)}, 400)

    def events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        revision = -1
        bridge = self.server.bridge
        try:
            while True:
                with bridge.condition:
                    bridge.condition.wait_for(
                        lambda: bridge.revision != revision, timeout=10
                    )
                    snapshot = (
                        bridge.snapshot() if revision != bridge.revision else None
                    )
                if snapshot is None:
                    self.wfile.write(b": keepalive\n\n")
                else:
                    revision = snapshot["revision"]
                    payload = json.dumps(snapshot, allow_nan=False)
                    self.wfile.write(f"event: update\ndata: {payload}\n\n".encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass


def main():
    bridge = SerialBridge()
    try:
        server = MonitorServer(bridge)
    except OSError as exc:
        raise SystemExit(f"could not start monitor: {exc}")
    connector = AutoConnector(bridge)
    connector.start()
    print(f"rc boat monitor: http://127.0.0.1:{server.server_port}", flush=True)
    print(
        "Press Ctrl+C to stop. Close other serial monitors before connecting.",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        connector.close()
        bridge.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
