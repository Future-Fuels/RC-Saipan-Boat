#!/usr/bin/env python3

import json
import math
import os
import pty
import threading
import time
import tty

from main import (
    LineDecoder,
    MonitorServer,
    SIMULATOR_FILE,
    SerialBridge,
    validate_command,
)


def make_packet(elapsed, sequence, controls):
    j1y = round(180 * math.sin(elapsed / 4))
    j2x = round(160 * math.sin(elapsed / 3))
    theta = elapsed / 12
    east, north = 12 * math.cos(theta), 12 * math.sin(theta)
    roll, pitch = 0.07 * math.sin(elapsed), 0.04 * math.cos(elapsed / 2)
    speed_limit = controls["limit"]
    effective_limit = 180 if speed_limit is None else speed_limit
    motor = controls["motor"]
    motor = round(j1y / 255 * effective_limit) if motor is None else motor
    servo = controls["servo"]
    servo = round(90 + j2x / 255 * 90) if servo is None else servo
    stopped = controls["stopped"]
    return dict(
        j1x=round(60 * math.sin(elapsed / 5)),
        j1y=j1y,
        j2x=j2x,
        j2y=round(40 * math.cos(elapsed / 4)),
        speed=180,
        requestedGpsResetId=controls["gpsResetId"],
        connected=True,
        telemetryFresh=True,
        accX=round(roll, 3),
        accY=round(pitch, 3),
        accZ=1.0,
        resG=round(math.sqrt(roll**2 + pitch**2 + 1), 3),
        gyroX=round(4 * math.cos(elapsed), 3),
        gyroY=round(-1.2 * math.sin(elapsed / 2), 3),
        gyroZ=4.775,
        linearAccX=round(roll * 9.81, 3),
        linearAccY=round(pitch * 9.81, 3),
        linearAccZ=0.0,
        quatW=round(math.cos(theta / 2), 4),
        quatX=0.0,
        quatY=0.0,
        quatZ=round(math.sin(theta / 2), 4),
        imuAvailable=True,
        systemCalibration=3,
        gyroCalibration=3,
        accelCalibration=3,
        magCalibration=3,
        gpsFix=True,
        gpsAgeMs=100,
        satellites=10,
        latitude=round(0.1 + north / 111320, 6),
        longitude=round(-140 + east / 111320, 6),
        altitude=0.0,
        gpsSpeed=1.0,
        gpsCourse=round((-math.degrees(theta)) % 360, 2),
        gpsSpeedValid=True,
        gpsCourseValid=True,
        gpsAltitudeValid=True,
        filterInitialized=True,
        east=round(east - controls["originEast"], 2),
        north=round(north - controls["originNorth"], 2),
        eastVelocity=round(-math.sin(theta), 3),
        northVelocity=round(math.cos(theta), 3),
        active=True,
        motorSpeed=0 if stopped else max(-effective_limit, min(effective_limit, motor)),
        servoAngle=90 if stopped else servo,
        controlPackets=sequence * 2,
        controlAgeMs=25,
        servoLocked=controls["servo"] is not None,
        motorLocked=controls["motor"] is not None,
        speedLimit=-1 if speed_limit is None else speed_limit,
        stopped=stopped,
        gpsResetId=controls["gpsResetId"],
    )


class VirtualTX:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.port = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.thread = None
        self.controls = dict(
            servo=None,
            motor=None,
            limit=None,
            stopped=False,
            gpsResetId=0,
            originEast=0,
            originNorth=0,
        )

    def apply_command(self, line, elapsed):
        try:
            command = validate_command(line)
        except ValueError:
            return False
        if command in ("stop", "resume"):
            self.controls["stopped"] = command == "stop"
        elif command == "unlock":
            self.controls.update(servo=None, motor=None, limit=None, stopped=False)
        elif command == "gps reset":
            self.controls["gpsResetId"] += 1
            self.controls["originEast"] = 12 * math.cos(elapsed / 12)
            self.controls["originNorth"] = 12 * math.sin(elapsed / 12)
        else:
            name, argument = command.split()
            self.controls[name] = None if argument == "free" else int(argument)
        return True

    def start(self):
        temporary = SIMULATOR_FILE.with_name(f".simulator-{os.getpid()}.tmp")
        temporary.write_text(json.dumps({"pid": os.getpid(), "port": self.port}))
        temporary.replace(SIMULATOR_FILE)
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def _write(self):
        start = time.monotonic()
        sequence = 0
        pending = b""
        decoder = LineDecoder(limit=64)
        while not self.stop.is_set():
            elapsed = time.monotonic() - start
            try:
                chunk = os.read(self.master, 4096)
            except BlockingIOError:
                chunk = b""
            except OSError:
                break
            for line in decoder.feed(chunk):
                accepted = line is not None and self.apply_command(line, elapsed)
                pending += (json.dumps({"commandAccepted": accepted}) + "\n").encode()
            if not pending:
                sequence += 1
                packet = make_packet(elapsed, sequence, self.controls)
                pending = (json.dumps(packet, separators=(",", ":")) + "\n").encode()
            try:
                pending = pending[os.write(self.master, pending) :]
            except BlockingIOError:
                pass
            except OSError:
                break
            self.stop.wait(0.1)

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)
        try:
            if json.loads(SIMULATOR_FILE.read_text()).get("port") == self.port:
                SIMULATOR_FILE.unlink(missing_ok=True)
        except (OSError, ValueError):
            pass


def main():
    simulator = None
    bridge = SerialBridge()
    server = None
    try:
        simulator = VirtualTX()
        server = MonitorServer(bridge)
        bridge.connect(simulator.port)
        simulator.start()
        print(
            f"rc boat monitor: http://127.0.0.1:{server.server_port} (simulated tx)",
            flush=True,
        )
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        raise SystemExit(f"could not start simulator: {exc}")
    finally:
        bridge.disconnect()
        if server:
            server.server_close()
        if simulator:
            simulator.close()


if __name__ == "__main__":
    main()
