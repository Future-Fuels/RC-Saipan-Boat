PIO           := pio
MONITOR_SPEED := 115200

PORT_TX       := /dev/tty.usbserial-0001
PORT_RX       := /dev/tty.usbserial-3

.PHONY: all build clean compiledb \
        tx-build tx-upload tx-monitor tx-run \
	rx-build rx-upload rx-monitor rx-run \
	tx-esp-now-build tx-esp-now-upload tx-esp-now-monitor \
	rx-esp-now-build rx-esp-now-upload rx-esp-now-monitor \
	servo-build servo-upload servo-monitor servo-run \
	sensor-build sensor-upload sensor-monitor sensor-run

all: tx-build rx-build

clean:
	$(PIO) run --target clean

compiledb:
	$(PIO) run -e rx --target compiledb


# tx
tx-build:
	$(PIO) run -e tx

tx-upload:
	$(PIO) run -e tx --target upload --upload-port $(PORT_TX)

tx-monitor:
	$(PIO) device monitor --environment tx --baud $(MONITOR_SPEED) --port $(PORT_TX)

tx-run:
	$(PIO) run -e tx --target upload --upload-port $(PORT_TX)
	$(PIO) device monitor --environment tx --baud $(MONITOR_SPEED) --port $(PORT_TX)

tx-clean:
	$(PIO) run -e tx --target clean


# rx
rx-build:
	$(PIO) run -e rx

rx-upload:
	$(PIO) run -e rx --target upload --upload-port $(PORT_RX)

rx-monitor:
	$(PIO) device monitor --environment rx --baud $(MONITOR_SPEED) --port $(PORT_RX)

rx-run:
	$(PIO) run -e rx --target upload --upload-port $(PORT_RX)
	$(PIO) device monitor --environment rx --baud $(MONITOR_SPEED) --port $(PORT_RX)

rx-clean:
	$(PIO) run -e rx --target clean


# ESP-NOW variants
tx-esp-now-build:
	$(PIO) run -e tx-esp-now

tx-esp-now-upload:
	$(PIO) run -e tx-esp-now --target upload --upload-port $(PORT_TX)

tx-esp-now-monitor:
	$(PIO) device monitor --environment tx-esp-now --baud $(MONITOR_SPEED) --port $(PORT_TX)

rx-esp-now-build:
	$(PIO) run -e rx-esp-now

rx-esp-now-upload:
	$(PIO) run -e rx-esp-now --target upload --upload-port $(PORT_RX)

rx-esp-now-monitor:
	$(PIO) device monitor --environment rx-esp-now --baud $(MONITOR_SPEED) --port $(PORT_RX)


# servo test
servo-build:
	$(PIO) run -e servo-test

servo-upload:
	$(PIO) run -e servo-test --target upload --upload-port $(PORT_RX)

servo-monitor:
	$(PIO) device monitor --environment servo-test --baud $(MONITOR_SPEED) --port $(PORT_RX)

servo-run:
	$(PIO) run -e servo-test --target upload --upload-port $(PORT_RX)
	$(PIO) device monitor --environment servo-test --baud $(MONITOR_SPEED) --port $(PORT_RX)


# sensor test
sensor-build:
	$(PIO) run -e sensor-test

sensor-upload:
	$(PIO) run -e sensor-test --target upload --upload-port $(PORT_RX)

sensor-monitor:
	$(PIO) device monitor --environment sensor-test --baud $(MONITOR_SPEED) --port $(PORT_RX)

sensor-run:
	$(PIO) run -e sensor-test --target upload --upload-port $(PORT_RX)
	$(PIO) device monitor --environment sensor-test --baud $(MONITOR_SPEED) --port $(PORT_RX)

