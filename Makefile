PIO           := pio
MONITOR_SPEED := 460800

PORT_TX       := /dev/tty.usbserial-0001
PORT_RX       := /dev/tty.usbserial-3

.PHONY: all clean compiledb \
        tx-build tx-upload tx-monitor tx-run tx-clean \
        rx-build rx-upload rx-monitor rx-run rx-clean \
        imu-calibrate-build imu-calibrate-upload imu-calibrate-monitor imu-calibrate-run

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

tx-run: tx-upload
	$(MAKE) tx-monitor

tx-clean:
	$(PIO) run -e tx --target clean


# rx
rx-build:
	$(PIO) run -e rx

rx-upload:
	$(PIO) run -e rx --target upload --upload-port $(PORT_RX)

rx-monitor:
	$(PIO) device monitor --environment rx --baud $(MONITOR_SPEED) --port $(PORT_RX)

rx-run: rx-upload
	$(MAKE) rx-monitor

rx-clean:
	$(PIO) run -e rx --target clean


# imu calibration
imu-calibrate-build:
	$(PIO) run -e imu-calibrate

imu-calibrate-upload:
	$(PIO) run -e imu-calibrate --target upload --upload-port $(PORT_RX)

imu-calibrate-monitor:
	$(PIO) device monitor --environment imu-calibrate --baud $(MONITOR_SPEED) --port $(PORT_RX)

imu-calibrate-run: imu-calibrate-upload
	$(MAKE) imu-calibrate-monitor
