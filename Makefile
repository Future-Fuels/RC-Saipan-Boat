PIO           := pio
MONITOR_SPEED := 115200

PORT_TX       := /dev/cu.usbserial-3
PORT_RX       := /dev/cu.usbserial-0001

.PHONY: all build clean compiledb \
        tx-build tx-upload tx-monitor tx-run \
        rx-build rx-upload rx-monitor rx-run

all: tx-build rx-build

clean:
	$(PIO) run --target clean

compiledb:
	$(PIO) run --target compiledb


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

