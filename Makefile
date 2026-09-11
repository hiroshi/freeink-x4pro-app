PIO := /Library/Frameworks/Python.framework/Versions/Current/bin/pio
FREEINK_SDK_URL := https://github.com/Free-Ink/freeink-sdk

# Reader feature secrets (WiFi + fetch URL) — see .env.example. Silently
# skipped if .env doesn't exist; real shell env vars still work either way,
# since platformio.ini reads whatever's in the environment at build time.
-include .env
export WIFI_SSID WIFI_PASSWORD FETCH_URL

.PHONY: build upload monitor

build: $(PIO) freeink-sdk
	$(PIO) run -e x4pro

# Compile + flash. The board only accepts a flash while it is awake: after PR #2
# there is a ~20 s stay-awake window after every boot, so `tap RESET, then make
# upload` lands it. Before that window exists (or on a first flash), fall back to
# the RESET-mash esptool loop in the usb-serial-jtag notes.
upload: $(PIO) freeink-sdk
	$(PIO) run -e x4pro -t upload

# Serial monitor (115200). Dies when the firmware light-sleeps; reconnect after
# the next reset.
monitor: $(PIO)
	$(PIO) device monitor -e x4pro

# Install the PlatformIO CLI if it isn't already on this machine. Real file
# target (not .PHONY) so make skips it once $(PIO) exists on disk.
$(PIO):
	pip3 install -U platformio

# Clone the FreeInk SDK checkout that platformio.ini's lib_deps symlink into.
# Real directory target (not .PHONY) so it's skipped once already cloned.
freeink-sdk:
	git clone $(FREEINK_SDK_URL) freeink-sdk
