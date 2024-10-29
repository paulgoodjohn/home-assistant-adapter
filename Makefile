.PHONY: all
all:
	@pio run

upload:
	@pio run -t upload

monitor:
	@pio device monitor

.PHONY: clean
clean:
	@pio run -t clean
