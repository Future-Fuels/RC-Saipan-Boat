PIO = pio
MONITOR_SPEED = 9600
PORT = 

.PHONY: all build upload monitor clean compiledb

all: build

build:
	$(PIO) run

upload:
	$(PIO) run --target upload

monitor:
	$(PIO) device monitor --baud $(MONITOR_SPEED)

run:
	$(PIO) run --target upload --target monitor

compiledb:
	$(PIO) run -t compiledb

clean:
	$(PIO) run --target clean

