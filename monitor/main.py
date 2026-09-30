import asyncio
import json
import os
import serial
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import websockets

PORT = "/dev/tty.usbserial-0001"
BAUD = 115200
HTTP_PORT = 8080
WS_PORT = 8765
MONITOR_DIR = os.path.dirname(os.path.abspath(__file__))
CLIENTS = set()
SER = None

REQUIRED_FIELDS = {
    "j1x",
    "j1y",
    "j2x",
    "j2y",
    "button",
    "speed",
    "calibrate",
    "accX",
    "accY",
    "accZ",
    "resG",
    "gyroX",
    "gyroY",
    "gyroZ",
    "connected",
}


def validate_packet(data):
    if not isinstance(data, dict):
        return False
    return REQUIRED_FIELDS.issubset(data.keys())


def handle_client_message(message):
    if SER is None or not SER.is_open:
        return

    text = message.strip()
    if text in {"c", "calibrate"}:
        SER.write(b"c")
        return

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return

    if isinstance(data, dict) and data.get("cmd") == "calibrate":
        SER.write(b"c")


async def serial_read():
    global SER
    try:
        print(f"connecting to serial {PORT}...")
        SER = serial.Serial(PORT, BAUD, timeout=0.1)
        ser = SER
        time.sleep(2)
        print("reading serial\n")

        while True:
            if ser.in_waiting:
                line = ser.readline()
                if not line:
                    continue

                try:
                    text = line.decode("utf-8").strip()
                    if not text:
                        continue
                    data = json.loads(text)
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue

                if not validate_packet(data):
                    continue

                if CLIENTS:
                    websockets.broadcast(CLIENTS, text)

            await asyncio.sleep(0.005)

    except serial.SerialException as e:
        print(f"error: {e}")
    except KeyboardInterrupt:
        print("\nclosing serial")
        if SER is not None and SER.is_open:
            SER.close()
            SER = None


async def websocket_handler(websocket):
    CLIENTS.add(websocket)
    try:
        async for message in websocket:
            handle_client_message(message)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CLIENTS.remove(websocket)


def start_http_server():
    handler = partial(SimpleHTTPRequestHandler, directory=MONITOR_DIR)
    httpd = ThreadingHTTPServer(("localhost", HTTP_PORT), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    print(f"http://localhost:{HTTP_PORT}/monitor.html")


async def main():
    start_http_server()
    server = await websockets.serve(websocket_handler, "localhost", WS_PORT)
    await serial_read()
    await server.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nstopped")
